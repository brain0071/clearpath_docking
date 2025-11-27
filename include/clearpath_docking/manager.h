#pragma once
#include <memory>
#include <string>
#include <vector>
#include "Clearpath/inc-pub/pubSysCls.h"
#include "clearpath_docking/clearpath_motor.h"

namespace clearpath_docking
{

    class Manager

    {

    private:
        sFnd::sFnd::SysManager *_mgr;
        size_t _numPorts;
        std::vector<std::shared_ptr<ClearpathMotor>> _axes;

    public:
        Manager();
        Manager(const Manager &) = delete;
        ~Manager();

        void initialize();
        void close();

        size_t numPorts() const { return _numPorts; }

        // return a motor
        const std::vector<ClearpathMotor::Ptr> &axes() const { return _axes; }

        // void getShutdownInfo();
        // void setGlobalShutdown(bool enabled);
    };

}
