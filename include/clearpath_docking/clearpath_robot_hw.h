#include <memory>
#include <string>
#include <vector>
#include <array>
#include <list>

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include "clearpath_docking/srv/axis_enable.hpp"
#include "clearpath_docking/manager.h"
#include "clearpath_docking/clearpath_motor_hw.h"

namespace clearpath_docking
{

    class ClearpathRobotHW : public hardware_interface::SystemInterface
    {
    public:
        
        enum Joint_t
        {
            Clearpath = 0,
            NumJoints = 1
        };
        enum JointControlMode
        {
            Control_Idle = 0,
            Control_Velocity = 1,
            Control_Position = 2
        };
        static const std::array<std::string, NumJoints> JointNames;

        ClearpathRobotHW();
        ~ClearpathRobotHW();

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // bool enableSrvCallback(const std::shared_ptr<clearpath_docking::srv::AxisEnable::Request> req,
        //    std::shared_ptr<clearpath_docking::srv::AxisEnable::Response> res);

        rclcpp::Service<clearpath_docking::srv::AxisEnable>::SharedPtr _enableAllServer;
        clearpath_docking::Manager manager_;
        JointControlMode control_mode_;
        ClearpathMotorHw::Ptr motor_;
        rclcpp::Node::SharedPtr node_;

        std::array<double, NumJoints> joint_position_;
        std::array<double, NumJoints> joint_velocity_;
        std::array<double, NumJoints> joint_current_;

        std::array<double, NumJoints> joint_position_command_;
        std::array<double, NumJoints> joint_velocity_command_;

        rclcpp::TimerBase::SharedPtr control_loop_timer_;
        // rclcpp::Duration elapsed_time_;
        std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    };

}
