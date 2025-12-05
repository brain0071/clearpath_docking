#include "clearpath_docking/clearpath_robot_hw.h"

#include <cstdio>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "controller_manager/controller_manager.hpp"

namespace clearpath_docking
{
    using sFnd::IInfo;
    using sFnd::mnErr;

    ClearpathRobotHW::ClearpathRobotHW()
        : manager_(), control_mode_(Control_Idle) {}

    ClearpathRobotHW::~ClearpathRobotHW()
    {
        if (motor_)
            motor_->disable();
    }

    const std::array<std::string, ClearpathRobotHW::NumJoints>
        ClearpathRobotHW::JointNames = {"motor"};

    hardware_interface::CallbackReturn ClearpathRobotHW::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ClearpathRobotHW"), "Clearpath Robot on_init() failed");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!node_)
        {
            node_ = std::make_shared<rclcpp::Node>("clearpath_motor_hw");
        }

        int sn = -1;
        auto it = info.hardware_parameters.find("sn");

        if (it == info.hardware_parameters.end())
        {
            RCLCPP_FATAL(rclcpp::get_logger("ClearpathRobotHW"),
                         "Motor serial number must be specified as hardware parameter 'sn'");
            return hardware_interface::CallbackReturn::ERROR;
        }

        try
        {
            sn = std::stoi(it->second);
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("ClearpathRobotHW"),
                         "Failed to parse hardware param 'sn': %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (sn < 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("ClearpathRobotHW"),
                         "Motor serial number must be positive, got %d", sn);
            return hardware_interface::CallbackReturn::ERROR;
        }

        try
        {
            manager_.initialize();
            RCLCPP_INFO(rclcpp::get_logger("ClearpathRobotHW"),
                        "Found %zu nodes on %zu ports",
                        manager_.axes().size(), manager_.numPorts());

            for (auto const &axis : manager_.axes())
            {
                IInfo &info_axis(axis->getInfo());
                RCLCPP_INFO(rclcpp::get_logger("ClearpathRobotHW"),
                            "  %s : s/n %d : f/w %s",
                            info_axis.Model.Value(),
                            static_cast<int>(info_axis.SerialNumber.Value()),
                            info_axis.FirmwareVersion.Value());

                if (static_cast<int>(info_axis.SerialNumber.Value()) == sn)
                {
                    RCLCPP_INFO(rclcpp::get_logger("ClearpathRobotHW"),
                                "Found motor with s/n %d", sn);
                    // TODO: just one motor (three)
                    motor_ = std::make_shared<ClearpathMotorHw>(JointNames[0], node_, axis);
                    break;
                }
            }
        }
        catch (mnErr &theErr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ClearpathRobotHW"),
                         "Caught error while initializing manager");
            std::printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n",
                        theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!motor_)
        {
            RCLCPP_FATAL(rclcpp::get_logger("ClearpathRobotHW"),
                         "Unable to find motor with s/n '%d'", sn);
            return hardware_interface::CallbackReturn::ERROR;
        }

        joint_position_.fill(0.0);
        joint_velocity_.fill(0.0);
        joint_current_.fill(0.0);
        joint_position_command_.fill(0.0);
        joint_velocity_command_.fill(0.0);

        control_mode_ = JointControlMode::Control_Idle;

        RCLCPP_INFO(rclcpp::get_logger("ClearpathRobotHW"),
                    "ClearpathRobotHW on_init() finished successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ClearpathRobotHW::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < NumJoints; ++i)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    JointNames[i], hardware_interface::HW_IF_POSITION, &joint_position_[i]));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    JointNames[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i]));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    JointNames[i], "current", &joint_current_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ClearpathRobotHW::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (size_t i = 0; i < NumJoints; ++i)
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    JointNames[i], hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    JointNames[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type ClearpathRobotHW::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        if (!motor_)
        {
            //
            return hardware_interface::return_type::OK;
        }

        const clearpath_docking::MotorState state = motor_->state();
        joint_position_[0] = state.position.raw();
        joint_velocity_[0] = state.velocity.raw();
        joint_current_[0] = state.current.raw();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ClearpathRobotHW::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        if (!motor_)
        {
            return hardware_interface::return_type::OK;
        }

        if (motor_->isEnabled())
        {
            if (control_mode_ == JointControlMode::Control_Position)
            {
                RCLCPP_DEBUG(rclcpp::get_logger("ClearpathRobotHW"),
                             "Setting Clearpath position to %f rad",
                             joint_position_command_[0]);
                motor_->setPosition(joint_position_command_[0]);
            }
            else if (control_mode_ == JointControlMode::Control_Velocity)
            {
                RCLCPP_DEBUG(rclcpp::get_logger("ClearpathRobotHW"),
                             "Setting Clearpath velocity to %f rad/s",
                             joint_velocity_command_[0]);
                motor_->setVelocity(joint_velocity_command_[0]);
            }
            else
            {
                motor_->stop();
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn ClearpathRobotHW::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("ClearpathRobotHW"), "Activating ClearpathRobotHW");
        joint_position_command_[Clearpath] = joint_position_[Clearpath];
        joint_velocity_command_[Clearpath] = 0.0;
        control_mode_ = JointControlMode::Control_Idle;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ClearpathRobotHW::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("ClearpathRobotHW"), "Deactivating ClearpathRobotHW");
        if (motor_)
        {
            motor_->stop();
        }
        control_mode_ = JointControlMode::Control_Idle;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    clearpath_docking::ClearpathRobotHW,
    hardware_interface::SystemInterface)