#include "clearpath_docking/clearpath_motor_hw.h"

#include <chrono>
#include <memory>
#include <string>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "clearpath_docking/msg/clearpath_state.hpp"

namespace clearpath_docking
{
    using clearpath_docking::MotorHwBase;
    using clearpath_docking::MotorState;
    constexpr double ClearpathMotorHw::DiagnosticUpdateTimerPeriod;

    ClearpathMotorHw::ClearpathMotorHw(const std::string &name,
                                       const rclcpp::Node::SharedPtr &node,
                                       const ClearpathMotor::Ptr &axis)
        : MotorHwBase(name, node),
          node_(node),
          name_(name), 
          clock_(node_->get_clock()),
          axis_(axis)
    {
        auto logger = node_->get_logger();
        RCLCPP_INFO_STREAM(logger, " === Configuring " << name_ << " ===");
        RCLCPP_INFO_STREAM(logger, " ..     Initial velocity limit "
                                       << axis_->velocityLimit() << " rad/s");
        RCLCPP_INFO_STREAM(logger, " .. Initial acceleration limit "
                                       << axis_->accelerationLimit() << " rad/s^2");
        RCLCPP_INFO_STREAM(logger, " ..       Initial torque limit "
                                       << axis_->maxTorquePct() << "%");
        RCLCPP_INFO_STREAM(logger, " ..        Initial max current "
                                       << axis_->maxCurrent() << " A");
        RCLCPP_INFO_STREAM(logger, " ..         Initial jerk limit "
                                       << axis_->jerkLimit());
        RCLCPP_INFO_STREAM(logger, " ..   Initial jerk limit delay "
                                       << axis_->jerkLimitDelay());

        declare_and_setup_parameters();
        diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(node_,
                                                                            DiagnosticUpdateTimerPeriod);
        diagnostic_updater_->setHardwareID(name_);

        diagnostic_updater_->add("state", [this](diagnostic_updater::DiagnosticStatusWrapper &stat)
                                 { stat.add("state", stateAsString()); });

        clearpath_publisher_ = node_->create_publisher<clearpath_docking::msg::ClearpathState>(name_ + "/clearpath_state", 10);

        double min_freq = 0.5;
        double max_freq = 2.0;
        state_freq_updater_ = std::make_shared<diagnostic_updater::TopicDiagnostic>("state",
                                                                                    *diagnostic_updater_,
                                                                                    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10),
                                                                                    diagnostic_updater::TimeStampStatusParam(),
                                                                                    clock_);
        RCLCPP_INFO(logger, "ClearpathMotorHw for '%s' initialized.", name_.c_str());
    }

    ClearpathMotorHw::~ClearpathMotorHw()
    {
        auto logger = node_->get_logger();
        RCLCPP_DEBUG_STREAM(logger, "Disabling " << name_);
        disable();
    }

    void ClearpathMotorHw::declare_and_setup_parameters()
    {
        double vel_limit_default = axis_->velocityLimit();
        double accel_limit_default = axis_->accelerationLimit();
        double torque_limit_default = axis_->maxTorquePct() / 100.0;
        int ras_expected_default = -1;

        node_->declare_parameter<double>("vel_limit", vel_limit_default);
        node_->declare_parameter<double>("accel_limit", accel_limit_default);
        node_->declare_parameter<double>("torque_limit", torque_limit_default);
        node_->declare_parameter<int>("ras_expected", ras_expected_default);

        param_callback_handle_ = node_->add_on_set_parameters_callback(
            std::bind(&ClearpathMotorHw::on_parameters_changed,
                      this,
                      std::placeholders::_1));
    }

    rcl_interfaces::msg::SetParametersResult ClearpathMotorHw::on_parameters_changed(
        const std::vector<rclcpp::Parameter> &params)
    {
        auto logger = node_->get_logger();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        double new_vel_limit = axis_->velocityLimit();
        double new_accel_limit = axis_->accelerationLimit();
        double new_torque_limit_pct = axis_->maxTorquePct();
        int new_ras_expected = -1;

        bool ras_check_requested = false;

        for (const auto &p : params)
        {
            const auto &name = p.get_name();

            if (name == "vel_limit")
            {
                double v = p.as_double();
                if (v >= 0.0)
                {
                    RCLCPP_INFO_STREAM(logger, " .. Setting velocity limit to " << v);
                    axis_->setVelocityLimit(v);
                    new_vel_limit = v;
                }
                else
                {
                    result.successful = false;
                    result.reason = "vel_limit must be >= 0";
                    return result;
                }
            }
            else if (name == "accel_limit")
            {
                double a = p.as_double();
                if (a >= 0.0)
                {
                    RCLCPP_INFO_STREAM(logger, " .. Setting acceleration limit to " << a);
                    axis_->setAccelerationLimit(a);
                    new_accel_limit = a;
                }
                else
                {
                    result.successful = false;
                    result.reason = "accel_limit must be >= 0";
                    return result;
                }
            }
            else if (name == "torque_limit")
            {
                double t = p.as_double();
                if (t >= 0.0)
                {
                    double pct = t * 100.0;
                    RCLCPP_INFO_STREAM(logger, " .. Setting torque_limit to " << pct);
                    axis_->setMaxTorquePct(pct);
                    new_torque_limit_pct = pct;
                }
                else
                {
                    result.successful = false;
                    result.reason = "torque_limit must be >= 0";
                    return result;
                }
            }
            else if (name == "ras_expected")
            {
                int r = p.as_int();
                new_ras_expected = r;
                ras_check_requested = (r >= 0);
            }
        }

        RCLCPP_INFO_STREAM(logger, " ++     New velocity limit "
                                       << axis_->velocityLimit() << " rad/s");
        RCLCPP_INFO_STREAM(logger, " ++ New acceleration limit "
                                       << axis_->accelerationLimit()
                                       << " rad/s^2");
        RCLCPP_INFO_STREAM(logger, " ++       New torque limit "
                                       << axis_->maxTorquePct() << "%");
        RCLCPP_INFO_STREAM(logger, " ++        New max current "
                                       << axis_->maxCurrent() << " A");

        auto motor_ras = axis_->rasValue();
        RCLCPP_INFO_STREAM(logger, " ..      Current RAS value "
                                       << ClearpathMotor::rasToString(motor_ras));

        if (ras_check_requested)
        {
            RCLCPP_INFO(logger, "Checking motor RAS value");

            if (motor_ras == ClearpathMotor::RASValue::RAS_UNKNOWN)
            {
                RCLCPP_WARN(logger, " !! Cannot determine current RAS value for motor");
            }
            else
            {
                auto ras_from_config = rasFromRosConfig(new_ras_expected);

                if (ras_from_config == motor_ras)
                {
                    RCLCPP_INFO_STREAM(
                        logger, " ++ Motor RAS "
                                    << ClearpathMotor::rasToString(motor_ras)
                                    << " matches the expected RAS "
                                    << ClearpathMotor::rasToString(ras_from_config));
                }
                else
                {
                    RCLCPP_WARN_STREAM(
                        logger, " !! Motor RAS "
                                    << ClearpathMotor::rasToString(motor_ras)
                                    << " does not match the expected RAS "
                                    << ClearpathMotor::rasToString(ras_from_config));
                }
            }
        }
        else
        {
            RCLCPP_INFO(logger, " --      (Not validating RAS value)");
        }

        return result;
    }

    bool ClearpathMotorHw::enable() { return axis_->enable(); }

    bool ClearpathMotorHw::disable() { return axis_->disable(); }

    bool ClearpathMotorHw::isEnabled() { return axis_->isEnabled(); }

    void ClearpathMotorHw::setPosition(double p) { axis_->positionMove(p); }

    void ClearpathMotorHw::setVelocity(double v, double max_allowed_dt, bool scale_if_exceeds_dt)
    {
        axis_->velocityMove(v, max_allowed_dt, scale_if_exceeds_dt);
    }

    void ClearpathMotorHw::stop(bool abrupt) { axis_->stop(abrupt); }

    MotorState ClearpathMotorHw::state()
    {
        auto logger = node_->get_logger();
        auto const start = clock_->now();
        axis_->checkErrors(true);
        auto const after_check_errors = clock_->now();

        MotorState state;
        state.position = axis_->position();
        auto const after_check_pos = clock_->now();

        state.velocity = axis_->velocity();
        auto const after_check_vel = clock_->now();

        state.current = axis_->current();
        auto const after_check_current = clock_->now();

        clearpath_docking::msg::ClearpathState cp_msg;
        cp_msg.header.stamp = clock_->now();
        cp_msg.serial_number = axis_->serialNumber();
        cp_msg.position = state.position.trueRaw();
        cp_msg.position_resolution = state.position.rawToMetric();
        cp_msg.velocity = state.velocity.trueRaw();
        cp_msg.velocity_resolution = state.velocity.rawToMetric();
        cp_msg.current = state.current.trueRaw();
        cp_msg.enabled = axis_->isEnabled();

        clearpath_publisher_->publish(cp_msg);

        auto const after_publish = clock_->now();

        RCLCPP_DEBUG_STREAM(logger, name_ << "  Check errors "
                                          << (after_check_errors - start).seconds());
        RCLCPP_DEBUG_STREAM(logger, name_ << "     Check pos "
                                          << (after_check_pos - after_check_errors)
                                                 .seconds());
        RCLCPP_DEBUG_STREAM(logger, name_ << "     Check vel "
                                          << (after_check_vel - after_check_pos)
                                                 .seconds());
        RCLCPP_DEBUG_STREAM(logger,
                            name_ << " Check current "
                                  << (after_check_current - after_check_vel)
                                         .seconds());
        RCLCPP_DEBUG_STREAM(logger,
                            name_ << "       Publish "
                                  << (after_publish - after_check_current)
                                         .seconds());
        RCLCPP_DEBUG_STREAM(logger,
                            name_ << "         Total "
                                  << (after_publish - start).seconds());

        return state;
    }

    bool ClearpathMotorHw::publish(const rclcpp::Time &stamp,
                                   const MotorState &state)
    {
        clearpath_docking::msg::ClearpathState cp_msg;
        cp_msg.header.stamp = stamp;

        cp_msg.serial_number = axis_->serialNumber();
        cp_msg.enabled = axis_->isEnabled();

        cp_msg.position = state.position.trueRaw();
        cp_msg.position_resolution = state.position.rawToMetric();
        cp_msg.velocity = state.velocity.trueRaw();
        cp_msg.velocity_resolution = state.velocity.rawToMetric();
        cp_msg.current = state.current.trueRaw();
        clearpath_publisher_->publish(cp_msg);

        return true;
    }

    std::string ClearpathMotorHw::stateAsString() const
    {
        return axis_->isEnabled() ? "enabled" : "disabled";
    }

}