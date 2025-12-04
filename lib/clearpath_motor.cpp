//
//
// Copyright 2025 University of Washington
//
// ClearpathMotor is a thin non-ROS abstraction around a single Clearpath-SC
// motor. It is largely an attempt to OOP the non-OOP Clearpath API It is based
// on the sample "ClearpathMotor.cpp" from the Clearpath SDK example
//

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
          serial_number_(node.Info.SerialNumber.Value())
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

            // Precalculated convenience constants
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

    void ClearpathMotor::setVelocityLimit(double newVelLimit)
    {
        try
        {
            node_.Motion.VelLimit = newVelLimit * counts_per_rad_;
        }
        catch (mnErr &theErr)
        {
            printf(
                "%s: Caught error in setVelocityLimit(): addr=%d, err=0x%08x\nmsg=%s\n",
                name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        }
    }

    double ClearpathMotor::velocityLimit()
    {
        try
        {
            node_.Motion.VelLimit.Refresh();
            return node_.Motion.VelLimit.Value() * rad_per_count_;
        }
        catch (mnErr &theErr)
        {
            printf("%s: Caught error in velocityLimit(): addr=%d, err=0x%08x\nmsg=%s\n",
                   name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            return -1;
        }
    }
    void ClearpathMotor::setAccelerationLimit(double newAccelLimit)
    {
        // Convert rad/sec^2 to counts/sec^2
        try
        {
            node_.Motion.AccLimit = newAccelLimit * counts_per_rad_;
        }
        catch (mnErr &theErr)
        {
            printf("%s: Caught error: addr=%d, err=0x%08x\nmsg=%s\n", name_.c_str(),
                   theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        }

        acc_limit_ = node_.Motion.AccLimit.Value();
    }

    double ClearpathMotor::accelerationLimit()
    {
        try
        {
            node_.Motion.AccLimit.Refresh();
            return node_.Motion.AccLimit.Value() * rad_per_count_;
        }
        catch (mnErr &theErr)
        {
            printf(
                "%s: Caught error in accelerationLimit(): addr=%d, "
                "err=0x%08x\nmsg=%s\n",
                name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            return -1;
        }
    }

    void ClearpathMotor::setMaxCurrent(double newTorqueLimit)
    {
        try
        {
            node_.Limits.TrqGlobal = newTorqueLimit;
        }
        catch (mnErr &theErr)
        {
            printf("%s, aught error: addr=%d, err=0x%08x\nmsg=%s\n", name_.c_str(),
                   theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        }
    }

    double ClearpathMotor::maxCurrent()
    {
        node_.Limits.TrqGlobal.Refresh();
        return node_.Limits.TrqGlobal.Value();
    }

    bool ClearpathMotor::setMaxTorquePct(double newTorqueLimitPct)
    {
        if ((newTorqueLimitPct < 0.0) || (newTorqueLimitPct > 100.0))
            return false;

        try
        {
            node_.TrqUnit(node_.PCT_MAX);
            node_.Limits.TrqGlobal = newTorqueLimitPct;
            node_.TrqUnit(node_.AMPS);
        }
        catch (mnErr &theErr)
        {
            printf("%s, Caught error: addr=%d, err=0x%08x\nmsg=%s\n", name_.c_str(),
                   theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            return false;
        }
        return true;
    }

    double ClearpathMotor::maxTorquePct()
    {
        node_.TrqUnit(node_.PCT_MAX);
        node_.Limits.TrqGlobal.Refresh();
        auto trq = node_.Limits.TrqGlobal.Value();
        node_.TrqUnit(node_.AMPS);

        return trq;
    }

    unsigned int ClearpathMotor::jerkLimit()
    {
        node_.Motion.JrkLimit.Refresh();
        return node_.Motion.JrkLimit.Value();
    }

    double ClearpathMotor::jerkLimitDelay()
    {
        node_.Motion.JrkLimitDelay.Refresh();
        return node_.Motion.JrkLimitDelay.Value();
    }

    ClearpathMotor::RASValue ClearpathMotor::rasValue()
    {
        unsigned int jerk = jerkLimit();
        double jerkDelayDbl = jerkLimitDelay();

        // This appears to be effective for all RAS values
        int jerkDelay = static_cast<int>(round(jerkDelayDbl));

        if (jerkDelay < 4)
            return RASValue::RAS_OFF;
        else if (jerkDelay == 4)
            return RASValue::RAS_4MS;
        else if (jerkDelay == 6)
            return RASValue::RAS_6MS;
        else if (jerkDelay == 10)
            return RASValue::RAS_10MS;
        else if (jerkDelay == 16)
            return RASValue::RAS_16MS;
        else if (jerkDelay == 25)
            return RASValue::RAS_25MS;

        return RASValue::RAS_UNKNOWN;
    }

    double ClearpathMotor::networkWatchdogMs()
    {
        node_.Setup.Ex.NetWatchdogMsec.Refresh();
        return node_.Setup.Ex.NetWatchdogMsec.Value();
    }

    bool ClearpathMotor::enable()
    {
        try
        {
            // Clear alerts and node stops

            checkErrors(true); // \todo{}  Shouldn't blanket cancel any error,
                               // some are OK to clear (low bus voltage)

            node_.Motion.NodeStopClear();

            std::cerr << name_ << ": --> Enabling Node <--" << std::endl;
            node_.EnableReq(true);

            // If the node is not currently ready, wait for it to get there
            time_t timeout;
            timeout = time(NULL) + 5;

            while (!node_.Status.IsReady())
            {
                if (time(NULL) > timeout)
                {
                    std::cout << name_ << ": Error: Timed out waiting for enable"
                              << std::endl;
                    return false;
                }
            }
        }
        catch (mnErr &theErr)
        {
            printf("%s: Caught error in enable(): addr=%d, err=0x%08x\nmsg=%s\n",
                   name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            return false;
        }
        catch (...)
        { // Catch any other unknown exceptions
            printf("Caught unknown exception during motor enable/disable");
            return false;
        }

        std::cerr << name_ << ": --> Enabled Node <--" << std::endl;
        return true;
    }

    bool ClearpathMotor::disable()
    {
        try
        {
            std::cout << name_ << ": --> Disabling Node <--" << std::endl;

            node_.EnableReq(false);

            // Poll the status register to confirm the node has disabled itself
            time_t timeout;
            timeout = time(NULL) + 3;
            node_.Status.RT.Refresh();
            while (node_.Status.RT.Value().cpm.Enabled)
            {
                if (time(NULL) > timeout)
                {
                    std::cerr << name_ << ": Error: Timed out waiting for disable"
                              << std::endl;
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

    bool ClearpathMotor::isEnabled() { return node_.EnableReq(); }

    bool ClearpathMotor::isReady() { return node_.Motion.IsReady(); }

    bool ClearpathMotor::positionMove(double targetRad, bool isAbsolute,
                                      bool addDwell)
    {
        if (!isReady())
        {
            std::cout << name_ << ": Motor is not ready, not moving" << std::endl;
            return false;
        }

        int64_t count = targetRad * counts_per_rad_;

        try
        {
            node_.Motion.MovePosnStart(count, isAbsolute, addDwell);
        }
        catch (mnErr &theErr)
        {
            printf("%s: Caught error in positionMove: addr=%d, err=0x%08x\nmsg=%s\n",
                   name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

            if (theErr.ErrorCode == MN_ERR_CMD_MV_FULL)
            {
                std::cout << name_ << ": Move buffer full ... " << std::endl;
            }

            return false;
        }

        return true;
    }

    bool ClearpathMotor::velocityMove(double target_vel, double max_allowed_dt,
                                      bool scale_if_exceeds_dt)
    {
        if (!isReady())
        {
            std::cerr << name_ << ": Motor is not ready, not moving" << std::endl;
            return false;
        }

        int64_t cps = target_vel * counts_per_rad_;

#ifdef USE_VEL_MEASURED
        int64 current_vel = node_.Motion.VelMeasured.Value();
#else
        int64 current_vel = node_.Motion.VelCommanded.Value();
#endif

        try
        {
            // If target_vel is within deadband of zero, use NodeStop instead

            // Currently an arbitrary constant, could be tuned
            const double vel_deadband = 0.02; // rad/sec.
            if (fabs(target_vel) < vel_deadband)
            {
                node_.Motion.NodeStop(STOP_TYPE_RAMP_AT_DECEL);
                return true;
            }

            // if (!node_.Motion.VelocityAtTarget()) {
            //   std::cerr << name_ << ": !! Not at velocity target, dropping command"
            //             << std::endl;
            // }

            if (max_allowed_dt > 0.0)
            {
                // double time_to_vel = node_.Motion.MoveVelDurationMsec(cps-current_vel)
                // / 1000.0;

                const double delta_v = cps - current_vel;

                double time_to_vel = delta_v / acc_limit_;
                if (time_to_vel > max_allowed_dt)
                {
                    std::cerr << name_ << ": Requested velocity " << cps << " from "
                              << current_vel << " would take " << time_to_vel * 1000.0
                              << " ms with accel limit " << acc_limit_
                              << ", greater than allowed interval of "
                              << max_allowed_dt * 1000.0 << " ms" << std::endl;

                    if (scale_if_exceeds_dt)
                    {
                        const double VEL_FUDGE = 1.1;
                        const double vel_scale = VEL_FUDGE * max_allowed_dt / time_to_vel;

                        // std::cerr << "  vel_scale "
                        //           << vel_scale << "  --  " << vel_scale*(cps -
                        //           current_vel)+current_vel << std::endl;

                        // This simple linear scaling of the target velocity is probably
                        // not accurate, but expedient.
                        //
                        cps = static_cast<int64_t>((vel_scale * delta_v) + current_vel);
                        std::cerr << name_ << ": .. scaled velocity request to "
                                  << static_cast<double>(cps) / counts_per_rad_ << " rad/s "
                                  << cps << " counts/sec)" << std::endl;
                    }
                    else
                    {
                        // Do not move
                        return false;
                    }
                }
            }

            // std::cout << name_ << ": Commanding velocity " << cps << " counts/sec" <<
            // std::endl;

            node_.Motion.MoveVelStart(cps);
        }
        catch (mnErr &theErr)
        {
            printf("%s: Caught error in velocityMove: addr=%d, err=0x%08x\nmsg=%s\n",
                   name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

            if (theErr.ErrorCode == MN_ERR_CMD_MV_FULL)
            {
                std::cout << name_ << ": Move buffer full" << std::endl;
                return false;
            }

            return true;
        }

        return true;
    }

    void ClearpathMotor::stop(bool abrupt)
    {
        try
        {
            if (abrupt)
            {
                std::cerr << name_ << ": ** Node stop abrupt!" << std::endl;
                node_.Motion.NodeStop(STOP_TYPE_ABRUPT);
            }
            else
            {
                std::cerr << name_ << ": ** Node stop ramp!" << std::endl;
                node_.Motion.NodeStop(STOP_TYPE_RAMP);
            }
        }
        catch (mnErr &theErr)
        {
            printf("%s: Caught error in stop(): addr=%d, err=0x%08x\nmsg=%s\n",
                   name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        }
    }

    EncoderValue ClearpathMotor::position(bool includeWraps)
    {
        // n.b. PosnMeasured is set to AutoRefresh
        int64_t position_counts =
            static_cast<int64_t>(node_.Motion.PosnMeasured.Value());

        if (includeWraps)
        {
            position_counts += static_cast<int64_t>(position_wrap_count_) << 32;
        }

        return EncoderValue(position_counts, rad_per_count_);
    }

    EncoderValue ClearpathMotor::velocity()
    {
        // n.b. VelMeasured is set to AutoRefresh
        return EncoderValue(static_cast<int64_t>(node_.Motion.VelMeasured.Value()),
                            rad_per_count_);
    }

    double ClearpathMotor::current()
    {
        // n.b. TrqMeasured is set to AutoRefresh
        return static_cast<double>(node_.Motion.TrqMeasured.Value());
    }

    EncoderValue ClearpathMotor::positionCmd(bool includeWraps)
    {
        node_.Motion.PosnCommanded.Refresh();

        int64_t position_counts =
            static_cast<int64_t>(node_.Motion.PosnCommanded.Value());

        if (includeWraps)
        {
            position_counts += static_cast<int64_t>(position_wrap_count_) << 32;
        }

        return EncoderValue(position_counts, rad_per_count_);
    }

    EncoderValue ClearpathMotor::velocityCmd()
    {
        node_.Motion.VelCommanded.Refresh();

        return EncoderValue(static_cast<int64_t>(node_.Motion.VelCommanded.Value()),
                            rad_per_count_);
    }

    double ClearpathMotor::currentCmd()
    {
        node_.Motion.TrqCommanded.Refresh();
        return static_cast<int64_t>(node_.Motion.TrqCommanded.Value());
    }

    bool ClearpathMotor::checkErrors(bool try_clear)
    {
        try
        {
            char alertList[256];
            bool has_errors = false;

            // node_.Status.RT.Refresh();
            node_.Status.Alerts.Refresh();
            node_.Status.Warnings.Refresh();

            // Check to see if the node experienced torque saturation
            //  Commented out; internally this calls Warnings.refresh() and takes ~2ms
            //
            // if (node_.Status.HadTorqueSaturation()) {
            //   std::cout
            //       << name_
            //       << ":      Node has experienced torque saturation since last
            //       checking"
            //       << std::endl;
            //   has_errors = true;
            // }

            auto alert_value = node_.Status.Alerts.Value();
            auto warn_value = node_.Status.Warnings.Value();

            // get an alert register reference, check the alert register directly for
            // alerts
            if (alert_value.isInAlert())
            {
                alert_value.StateStr(alertList, 256);
                std::cout << name_ << ":   Node has alerts! Alerts:" << alertList
                          << std::endl;

                // can access specific alerts using the method below
                if (alert_value.cpm.Common.EStopped)
                {
                    std::cout << name_ << ":      Node is e-stopped" << std::endl;
                    has_errors = true;
                    node_.Motion.NodeStopClear();
                }
                if (alert_value.cpm.TrackingShutdown)
                {
                    std::cout << name_ << ":      Node exceeded Tracking error limit"
                              << std::endl;
                    has_errors = true;
                }

                if (try_clear)
                {
                    clearErrors();
                }
            }

            return has_errors;
        }
        catch (mnErr &theErr)
        {
            printf("%s: Caught error in checkErrors(): addr=%d, err=0x%08x\nmsg=%s\n",
                   name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            return true;
        }
    }

    void ClearpathMotor::clearErrors()
    {
        try
        {
            char alertList[256];

            std::cout << name_ << ":      Clearing non-serious alerts" << std::endl;
            node_.Status.AlertsClear();
            node_.Motion.NodeStopClear();

            // Are there still alerts?
            node_.Status.Alerts.Refresh();

            if (node_.Status.Alerts.Value().isInAlert())
            {
                node_.Status.Alerts.Value().StateStr(alertList, 256);
                std::cout << name_
                          << ":   Node has serious, non-clearable alerts: " << alertList
                          << std::endl;
            }
            else
            {
                std::cout << name_ << ":   Node " << node_.Info.Ex.Addr()
                          << ": all alerts have been cleared" << std::endl;
            }
        }
        catch (mnErr &theErr)
        {
            printf("%s: Caught error in clearErrors(): addr=%d, err=0x%08x\nmsg=%s\n",
                   name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        }
    }

    void ClearpathMotor::setPosition(double offset)
    {
        try
        {
            node_.Motion.AddToPosition(-node_.Motion.PosnMeasured.Value() + offset);
        }
        catch (mnErr &theErr)
        {
            printf("%s: Caught error in setPosition(): addr=%d, err=0x%08x\nmsg=%s\n",
                   name_.c_str(), theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        }
    }

    void ClearpathMotor::initiateHoming(void)
    {
        std::cerr << name_ << ": This function doesn't do anything" << std::endl;
    }
    

}