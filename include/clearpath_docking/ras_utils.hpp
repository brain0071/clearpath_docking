#pragma once

#include "clearpath_docking/clearpath_motor.hpp"

namespace clearpath_docking
{

    inline ClearpathMotor::RASValue rasFromRosConfig(int ras)
    {
        switch (ras)
        {
        case -1:
            return ClearpathMotor::RASValue::RAS_UNKNOWN;
        case 0:
            return ClearpathMotor::RASValue::RAS_OFF;
        case 4:
            return ClearpathMotor::RASValue::RAS_4MS;
        case 6:
            return ClearpathMotor::RASValue::RAS_6MS;
        case 10:
            return ClearpathMotor::RASValue::RAS_10MS;
        case 16:
            return ClearpathMotor::RASValue::RAS_16MS;
        case 25:
            return ClearpathMotor::RASValue::RAS_25MS;
        default:
            return ClearpathMotor::RASValue::RAS_UNKNOWN;
        }
    }

}