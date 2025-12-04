#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "clearpath_docking/clearpath_robot_hw.hpp"

using clearpath_docking::ClearpathRobotHW;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.namespace_("clearpath_robot");
    auto clearpath_robot_node = std::make_shared<rclcpp::Node>("clearpath_robot_node", options);
    auto clearpath_robot = std::make_shared<ClearpathRobotHW>();
    if (!clearpath_robot->init(clearpath_robot_node))
    {
        RCLCPP_FATAL(clearpath_robot_node->get_logger(), "clearpath->init() failed");
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(clearpath_robot_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}