#include "clearpath_docking/clearpath_motor.h"

namespace clearpath_docking {


    ClearpathMotor::ClearpathMotor(/* args */)
    {
    }
    
    ClearpathMotor::~ClearpathMotor()
    {
    }
    
    bool ClearpathMotor::init(const rclcpp::Node::SharedPtr & node)
    {
        node_ = node;
        RCLCPP_INFO(node_->get_logger(), "ClearpathMotor init() called.");
        return true;
    }


}