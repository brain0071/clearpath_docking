#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "clearpath_docking/motor_hw_base.h"
#include "clearpath_docking/clearpath_motor.h"
#include "clearpath_docking/ras_utils.h"
#include "clearpath_docking/msg/clearpath_state.hpp"

namespace clearpath_docking
{
    class ClearpathMotorHw : public clearpath_docking::MotorHwBase
    {
    public:
        using Ptr = std::shared_ptr<ClearpathMotorHw>;

        static constexpr double DiagnosticUpdateTimerPeriod = 0.1;
        ClearpathMotorHw(const std::string &name,
                         const rclcpp::Node::SharedPtr &node,
                         const ClearpathMotor::Ptr &axis);

        ~ClearpathMotorHw() override;
        bool enable() override;
        bool disable() override;
        bool isEnabled() override;
        void setPosition(double p) override;
        void setVelocity(double v, double max_allowed_dt = -0.0,
                         bool scale_if_exceeds_dt = false) override;

        void stop(bool abrupt = false) override;

        clearpath_docking::MotorState state() override;

        EncoderValue position() override { return axis_->position(); }
        EncoderValue velocity() override { return axis_->velocity(); }
        EncoderValue current() override { return axis_->current(); }

        bool publish(const rclcpp::Time &stamp,
                     const clearpath_docking::MotorState &state) override;

    protected:
        void declare_and_setup_parameters();
        rcl_interfaces::msg::SetParametersResult on_parameters_changed(const std::vector<rclcpp::Parameter> &params);
        std::string stateAsString() const;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Clock::SharedPtr clock_;
        ClearpathMotor::Ptr axis_;
        std::string name_;

        std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
        std::shared_ptr<diagnostic_updater::TopicDiagnostic> state_freq_updater_;

        rclcpp::Publisher<clearpath_docking::msg::ClearpathState>::SharedPtr clearpath_publisher_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        std::mutex block_mutex_;
        std::condition_variable block_cv_;
    };

}
