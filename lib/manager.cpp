//
//
// Copyright 2025 University of Washington

#include "clearpath_docking/manager.h"

#include <iostream>
#include <string>
#include <vector>

namespace clearpath_docking {

using sFnd::IInfo;
using sFnd::INode;
using sFnd::IPort;
using sFnd::mnErr;
using sFnd::SysManager;

Manager::Manager() : _mgr(SysManager::Instance()) {}

Manager::~Manager() {
  // Axes try to shut themselves down during destruction
  // So manager must be running when they are destroyed
  _axes.clear();

  // If not done previously
  _mgr->PortsClose();
}

void Manager::initialize() {
  try {
    std::vector<std::string> portNames;
    SysManager::FindComHubPorts(portNames);

    // Editing of the list of portnames (e.g. ignoring particular
    // ports) could happen here.

    // See Tekinc Linux_Software/inc/inc-pub/pubMnNetDef.h
    const netRates DefaultNetRate = MN_BAUD_96X;  // 921600 baud

    _numPorts = portNames.size();
    for (size_t portNum = 0;
         portNum < portNames.size() && portNum < NET_CONTROLLER_MAX;
         portNum++) {
      _mgr->ComHubPort(portNum, portNames[portNum].c_str(), DefaultNetRate);
    }

    _mgr->PortsOpen(_numPorts);

    for (size_t portNum = 0; portNum < _numPorts; ++portNum) {
      IPort &port = _mgr->Ports(portNum);

      for (size_t nodeNum = 0; nodeNum < port.NodeCount(); nodeNum++) {
        INode &node = port.Nodes(nodeNum);

        // Make sure we are talking to a ClearPath SC (advanced or basic
        // model)
        if (node.Info.NodeType() != IInfo::CLEARPATH_SC_ADV &&
            node.Info.NodeType() != IInfo::CLEARPATH_SC) {
          //("---> ERROR: Uh-oh! Node %d is not a ClearPath-SC Motor\n",
          // iNode);
          continue;
        }

        _axes.push_back(std::make_shared<ClearpathMotor>(node));
      }
    }
  } catch (mnErr &theErr) {
    printf(
        "Exception in Manager::initialize(). Caught error: addr=%d, "
        "err=0x%08x\nmsg=%s\n",
        theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
  }
}

void Manager::getShutdownInfo() {
  try {
    for (size_t portNum = 0; portNum < _numPorts; ++portNum) {
      IPort &port = _mgr->Ports(portNum);

      for (size_t nodeNum = 0; nodeNum < port.NodeCount(); nodeNum++) {
        INode &node = port.Nodes(nodeNum);
        IInfo &info(node.Info);

        ShutdownInfo shutdownInfo;
        port.GrpShutdown.ShutdownWhenGet(nodeNum, shutdownInfo);

        char state[256];
        shutdownInfo.statusMask.cpm.StateStr(state, 255);
        std::cout << "Port " << nodeNum << ": S/N " << info.SerialNumber.Value()
                  << " Group shutdown: "
                  << (shutdownInfo.enabled ? "ENABLED" : "DISABLED") << ": "
                  << state << std::endl;
      }
    }
  } catch (mnErr &theErr) {
    printf(
        "Exception in Manager::getShutdownInfo(). Caught error: addr=%d, "
        "err=0x%08x\nmsg=%s\n",
        theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
  }
}

void Manager::setGlobalShutdown(bool enabled) {
  try {
    for (size_t portNum = 0; portNum < _numPorts; ++portNum) {
      IPort &port = _mgr->Ports(portNum);

      for (size_t nodeNum = 0; nodeNum < port.NodeCount(); nodeNum++) {
        INode &node = port.Nodes(nodeNum);
        ShutdownInfo shutdownInfo;
        port.GrpShutdown.ShutdownWhenGet(nodeNum, shutdownInfo);

        shutdownInfo.enabled = enabled;
        port.GrpShutdown.ShutdownWhen(nodeNum, shutdownInfo);
      }
    }
  } catch (mnErr &theErr) {
    printf(
        "Exception in Manager::setGlobalShutdown(). Caught error: addr=%d, "
        "err=0x%08x\nmsg=%s\n",
        theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
  }
}

}  // namespace clearpath_sc_ros