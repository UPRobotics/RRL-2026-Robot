#ifndef BODY_HPP
#define BODY_HPP

#include "robot_pkg/VESC.hpp"
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>

class BODY{
    private:
        std::vector<std::unique_ptr<VESC>> motors;
        rclcpp::Logger logger;
    
    public:
        BODY();
        ~BODY();
};

#endif