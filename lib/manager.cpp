#include "clearpath_docking/manager.h"

#include <iostream>
#include <string>
#include <vector>

namespace clearpath_docking
{

    using sFnd::IInfo;
    using sFnd::INode;
    using sFnd::IPort;
    using sFnd::mnErr;
    using sFnd::SysManager;
    Manager::Manager() : _mgr(SysManager::Instance()) {}
    Manager::~Manager()
    {
        _axes.clear();
        mgr->PortsClose();
    }

    void Manager::initialize()
    {
        try
        {
            std::vector<std::string> portNames;
            SysManager::FindComHubPorts(portNames);

            // See Tekinc Linux_Software/inc/inc-pub/pubMnNetDef.h
            const netRates DefaultNetRate = MN_BAUD_96X; // 921600 baud
            _numPorts = portNames.size();

            for (size_t portNum = 0; portNum < portNames.size() && portNum < NET_CONTROLLER_MAX; portNum++)
            {
                _mgr->ComHubPort(portNum, portNames[portNum].c_str(), DefaultNetRate);
            }
            _mgr->PortsOpen(_numPorts);

            for (size_t portNum = 0; portNum < _numPorts; ++portNum)
            {
                IPort &port = _mgr->Ports(portNum);

                printf("Try Open Port[%d]: state=%d, nodes=%d", port.NetNumber(), port.OpenState(), port.NodeCount());
                
                for (size_t nodeNum = 0; nodeNum < port.NodeCount(); nodeNum++)
                {
                    INode &node = port.Nodes(nodeNum);
                    if (node.Info.NodeType() != IInfo::CLEARPATH_SC_ADV &&
                        node.Info.NodeType() != IInfo::CLEARPATH_SC)
                    {
                        continue;
                    }

                    _axes.push_back(std::make_shared<ClearpathMotor>(node));
                }
            }
        }
        catch (mnErr &theErr)
        {
            printf(
                "Exception in Manager::initialize(). Caught error: addr=%d, "
                "err=0x%08x\nmsg=%s\n",
                theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        }
    }
}
