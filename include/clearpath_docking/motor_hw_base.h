#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "clearpath_docking/motor_state.hpp"
#include "clearpath_docking/srv/axis_enable.hpp"

namespace clearpath_docking
{
    class MotorHwBase
    {
        using SharedPtr = std::shared_ptr<MotorHwBase>;
        using AxisEnable = clearpath_docking::srv::AxisEnable;

        explicit MotorHwBase(const std::string &name,
                             const rclcpp::Node::SharedPtr &node);

        virtual ~MotorHwBase();

        virtual bool enable() = 0;
        virtual bool disable() = 0;
        virtual bool isEnabled() = 0;
        virtual void setPosition(double p) = 0;
        virtual void setVelocity(double v, double max_allowed_dt = 0.0, bool scale_if_exceeds_dt = false) = 0;
        virtual void stop(bool abrupt = false)
        {
            (void)abrupt;
            setVelocity(0.0);
        }

        virtual MotorState state() = 0;
        virtual EncoderValue position() = 0;
        virtual EncoderValue velocity() = 0;
        virtual EncoderValue current() = 0;
        virtual bool publish(const rclcpp::Time &stamp, const MotorState &state)
        {
            (void)stamp;
            (void)state;
            return true;
        }

    protected:
        std::string name_;
        rclcpp::Node::SharedPtr node_;

        void axisEnableSrvCallback(
            const std::shared_ptr<AxisEnable::Request> request,
            std::shared_ptr<AxisEnable::Response> response);

        rclcpp::Service<AxisEnable>::SharedPtr axis_enable_server_;
    };
}