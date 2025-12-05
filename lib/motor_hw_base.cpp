#include "clearpath_docking/motor_hw_base.h"

namespace clearpath_docking
{
    MotorHwBase::MotorHwBase(const std::string &name,
                             const rclcpp::Node::SharedPtr &node)
        : name_(name), node_(node)
    {
        auto service_name = name_ + "/enable";
        axis_enable_server_ = node_->create_service<AxisEnable>(service_name, std::bind(
                                                                                  &MotorHwBase::axisEnableSrvCallback,
                                                                                  this,
                                                                                  std::placeholders::_1,
                                                                                  std::placeholders::_2));
        RCLCPP_INFO(node_->get_logger(),
                    "MotorHwBase for '%s' advertising service '%s'",
                    name_.c_str(), service_name.c_str());
    }

    MotorHwBase::~MotorHwBase() = default;

    void MotorHwBase::axisEnableSrvCallback(
        const std::shared_ptr<AxisEnable::Request> request,
        std::shared_ptr<AxisEnable::Response> response)
    {
        bool ok = false;

        if (request->enable)
        {
            ok = enable();
            response->message = ok ? "Motor enabled successfully."
                                   : "Failed to enable motor.";
        }
        else
        {
            ok = disable();
            response->message = ok ? "Motor disabled successfully."
                                   : "Failed to disable motor.";
        }

        response->success = ok;
    }

}