//
// C++ class for a single Clearpath Motor
// This is a "medium-weight" wrapper around the
// COM-style abstractions offered by the SFoundation
// with some added functionality.
//
// Base on the "ClearpathMotor.h" from the Clearpath SDK examples
//
// Copyright 2025 University of Washington
//

#pragma once

#include <time.h>
#include <memory>
#include <string>
#include "pubSysCls.h"
#include "clearpath_docking/encoder_value.h"

namespace clearpath_docking
{
    using clearpath_docking::EncoderValue;
    class ClearpathMotor
    {
    public:
        typedef std::shared_ptr<ClearpathMotor> Ptr;
        enum class RASValue
        {
            RAS_OFF,
            RAS_4MS,
            RAS_6MS,
            RAS_10MS,
            RAS_16MS,
            RAS_25MS,
            // There are additional values which I don't have noted here, can be added
            // as needed
            RAS_UNKNOWN
        };

        static std::string rasToString(const RASValue ras);
        explicit ClearpathMotor(sFnd::INode &node, const std::string name = std::string());
        ~ClearpathMotor();

        void setName(const std::string &new_name) { name_ = new_name; }
        sFnd::IInfo &getInfo() { return node_.Info; }
        int32_t serialNumber() const { return serial_number_; }

        bool enable();
        // "enabled" is the specific case of "has the user enabled to motor"?
        // e.g. a motor can be enabled and yet also be e-stopped
        bool isEnabled();

        // "ready" is the broader case of "will the motor move"?
        bool isReady();

        bool disable();
        // Set velocity limit in rad/sec
        void setVelocityLimit(double newVelLimit);
        double velocityLimit();

        // Set acceleration limit in rad/sec^2
        void setAccelerationLimit(double newAccLimit);
        double accelerationLimit();

        // Set max torque in Amps
        void setMaxCurrent(double newTorqueLimit);
        double maxCurrent();

        // Get set max torque as a pct (0-100) of max torque
        bool setMaxTorquePct(double newTorqueLimitPct);
        double maxTorquePct();

        // Query jerk limit (units?) and jerk limit delay ONLY
        // Setting jerk/jerk limit interferes with RAS
        unsigned int jerkLimit();
        double jerkLimitDelay();
        RASValue rasValue();

        // Return motor state (in radians, rad/sec, amps)
        EncoderValue position(bool includeWraps = true);
        EncoderValue velocity();
        double current();

        // Return commanded position / velocity / torque
        EncoderValue positionCmd(bool includeWraps = true);
        EncoderValue velocityCmd();
        double currentCmd();

        // Initiate a move
        bool positionMove(double targetPosn, bool isAbsolute = false,
                          bool addDwell = false);

        // Commands the motor to move to the target_vel (given in rad/sec)
        //
        // If max_allowed_dt > 0, the API will calculate a predicted time
        // to achieve the target vel.  If that time is greater than max_allowed_dt
        // (in seconds),
        //
        //    if scale_if_exceeds_dt is false, no velocity is sent to the motor
        //       and the function returns false
        //
        //    if scale_if_exceeds_dt is true, target_vel is scaled to an
        //       achievable value and sent to the motor
        bool velocityMove(double target_vel, double max_allowed_dt = -1.0,
                          bool scale_if_exceeds_dt = false);

        void stop(bool abrupt = false);

        // A blunt hammer of a function right now
        bool checkErrors(bool try_clear = false);
        void clearErrors(void);

        // This is effectively "reset zero so the current position is <offset>"
        void setPosition(double offset = 0.0);
        // Starts a homing procedure --- homing behaviors **must** be
        // pre-programmed into the motor using the Windows GUI
        void initiateHoming();

        double countsToRad(double c) const { return c * rad_per_count_; }

        void setNetworkWatchdogMs(double val);
        double networkWatchdogMs();

    private:
        sFnd::INode &node_; // The ClearPath-SC for this ClearpathMotor
        std::string name_;

        int32_t serial_number_;

        uint32_t resolution_;     // Counts per rotation
        uint32_t counts_per_rad_; // Counts per radians
        float rad_per_count_;     // Radians per count

        double acc_limit_;

        int32_t position_wrap_count_;

        void init();
    };
}
