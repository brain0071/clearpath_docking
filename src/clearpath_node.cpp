#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "clearpath_docking/clearpath_motor.h"


using clearpath_docking::ClearpathMotor;


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("clearpath_node");
    
    ClearpathMotor clearpath_motor;

    if (!clearpath_motor.init(node)) {
        RCLCPP_FATAL(node->get_logger(), "clearpath.init() failed");
        rclcpp::shutdown();
        return -1;
    }
        
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();   
    rclcpp::shutdown();
    return 0;

}


