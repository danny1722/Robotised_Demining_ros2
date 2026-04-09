#ifndef DRIVE_MODE_HPP
#define DRIVE_MODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
//#include <libserial/SerialPort.h>
//#include <libserial/SerialStream.h>

class DigMode : public rclcpp::Node
{
public:
    DigMode();

private:
    void switch_mode(const std_msgs::msg::String::SharedPtr msg);
    void dig_mode_logic(const sensor_msgs::msg::Joy::SharedPtr msg);
    std::string is_ready_to_switch();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_data;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_mode;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr confirmation_publisher;
    //LibSerial::SerialPort serial_port_rover;

    // Variables
    std::string current_mode;
    sensor_msgs::msg::Joy::SharedPtr latest_joy_msg;

    bool debug_mode = false; // Set to true to enable debug logs
};

#endif // DRIVE_MODE_HPP
