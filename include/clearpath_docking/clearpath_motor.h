#pragma once
#include <time.h>
#include <memory>
#include <string>
#include "Clearpath/inc-pub/pubSysCls.h"

namespace clearpath_docking
{

    class ClearpathMotor
    {
    private:
        sFnd::INode &node_; // The ClearPath-SC for this ClearpathMotor
        std::string name_;
        int32_t serial_number_;
        uint32_t resolution_;
        uint32_t counts_per_rad_;
        float rad_per_count_;
        double acc_limit_;
        int32_t position_wrap_count_;
        void init();

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
        explicit ClearpathMotor(Fnd::INode &node, // NOLINT [runtime/references]
                                const std::string name = std::string());
        ~ClearpathMotor();

        // Regressive Auto-Spline (RAS) 
        static std::string rasToString(const RASValue ras);
        void setName(const std::string &new_name) { name_ = new_name; }

        bool disable();
    };
}