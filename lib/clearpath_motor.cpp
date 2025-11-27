#include "clearpath_docking/clearpath_motor.h"

#include <math.h>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <string>

namespace clearpath_docking
{
    std::string ClearpathMotor::rasToString(RASValue ras)
    {
        if (ras == RASValue::RAS_OFF)
            return "off";
        else if (ras == RASValue::RAS_4MS)
            return "4ms";
        else if (ras == RASValue::RAS_6MS)
            return "6ms";
        else if (ras == RASValue::RAS_10MS)
            return "10ms";
        else if (ras == RASValue::RAS_16MS)
            return "16ms";
        else if (ras == RASValue::RAS_25MS)
            return "25ms";

        return "(unknown)";
    }

    using sFnd::INode;
    using sFnd::mnErr;

    ClearpathMotor::ClearpathMotor(INode &node, const std::string name)
        : node_(node),
          name_(name),
          position_wrap_count_(0),
          serial_number_(node.Info.SerialNumber.value())
    {
        init();
    }

    ClearpathMotor::~ClearpathMotor() { disable(); }

    void ClearpathMotor::init()
    {
        try
        {
            // Always use counts and current internally in the motor
            node_.AccUnit(INode::COUNTS_PER_SEC2);
            node_.VelUnit(INode::COUNTS_PER_SEC);
            node_.TrqUnit(INode::AMPS);
            resolution_ = node_.Info.PositioningResolution.Value();

            counts_per_rad_ = resolution_ / (2 * M_PI);
            rad_per_count_ = 2 * M_PI / resolution_;

            node_.Motion.TrqMeasured.AutoRefresh(true);
            node_.Motion.VelMeasured.AutoRefresh(true);
            node_.Motion.PosnMeasured.AutoRefresh(true);
            node_.Motion.VelCommanded.AutoRefresh(true);

            node_.Motion.AccLimit.Refresh();
            acc_limit_ = node_.Motion.AccLimit.Value();
        }
        catch (mnErr &theErr)
        {
            printf("%s: Caught error in init(): addr=%d, err=0x%08x\nmsg=%s\n",
                   name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        }
    }

    bool ClearpathMotor::disable()
    {
        try
        {
            std::cout << name_ << ": --> Disabling Node <--" << std::endl;
            node_.EnableReq(false);
            time_t timeout;
            timeout = time(NULL) + 3;
            node_.Status.RT.Refresh();
            while (node_.Status.RT.Value().cpm.Enabled)
            {
                if (time(NULL) > timeout)
                {
                    std::cerr << name_ << ": Error: Timed out waiting for disable" << std::endl;
                    return false;
                }
                node_.Status.RT.Refresh();
            }
            std::cout << name_ << ": --> Disabled Node <--" << std::endl;
            return true;
        }
        catch (mnErr &theErr)
        {
            printf("%s: Caught error in disable(): addr=%d, err=0x%08x\nmsg=%s\n",
                   name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            return false;
        }
    }

}