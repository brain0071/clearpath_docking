#pragma once
#include <rclcpp/rclcpp.hpp>

namespace clearpath_docking {

    class ClearpathMotor
    {
    private:
        rclcpp::Node::SharedPtr node_;
        
    public:
        ClearpathMotor(/* args */);
        ~ClearpathMotor();
        bool init(const rclcpp::Node::SharedPtr & node);
    };
}