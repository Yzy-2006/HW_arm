#include <iostream>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"

#include "ros_controller/rosLoop.h"

int main(int argc, char ** argv){
    (void) argc;
    (void) argv;

    std::cout << "机器人控制程序开启" << std::endl;
    std::cout << "===================================" << std::endl;

    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<ServoNode>();
        node->run();
    } catch (const std::exception& error) {
        std::cerr << "Servo operation failed: " << error.what() << std::endl;
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
    if (rclcpp::ok()) rclcpp::shutdown();

    return 0;
}
