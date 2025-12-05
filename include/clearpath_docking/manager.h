//
//
// Copyright 2025 University of Washington

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "pubSysCls.h"
#include "clearpath_docking/clearpath_motor.h"

namespace clearpath_docking {

class Manager {
 public:
  Manager();
  Manager(const Manager &) = delete;

  ~Manager();

  // Opens all ports and build a list of all nodes
  void initialize();
  void close();

  size_t numPorts() const { return _numPorts; }

  const std::vector<ClearpathMotor::Ptr> &axes() const { return _axes; }

  void getShutdownInfo();
  void setGlobalShutdown(bool enabled);

 private:
  // Yes, a bare pointer.  Lifecycle of the SysManager singleton is
  // managed in the Clearpath library itself (?)
  sFnd::SysManager *_mgr;

  size_t _numPorts;
  std::vector<std::shared_ptr<ClearpathMotor>> _axes;
};

}  
