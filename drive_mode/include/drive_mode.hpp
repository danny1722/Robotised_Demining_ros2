#ifndef DRIVE_MODE_HPP
#define DRIVE_MODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

class DriveMode : public rclcpp::Node
{
public:
    DriveMode();

private:
    void switch_mode(const std_msgs::msg::String::SharedPtr msg);
    void drive_mode_logic(const sensor_msgs::msg::Joy::SharedPtr msg);
    void sensor_msg_callback(const std_msgs::msg::String::SharedPtr msg);
    std::string is_ready_to_switch();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_data;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_mode;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_sensor_data;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_debug_data;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_error_data;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_serial_status;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr confirmation_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drive_command_publisher;

    // Variables
    std::string current_mode;
    sensor_msgs::msg::Joy::SharedPtr latest_joy_msg;

    bool debug_mode = true; // Set to true to enable debug logs
    bool serial_alive = false; // Track the status of the serial connection
    std::string debug_data = ""; // Variable to store the latest debug data received from the Arduino

    int current_left_motor_speed = 0;
    int current_right_motor_speed = 0;

    // Stabilizer (anchor/stake) limit switch calibration - Left and Right independent
    bool stabilizer_left_calibrated = false; // Track if left stabilizer calibration is complete
    bool stabilizer_right_calibrated = false; // Track if right stabilizer calibration is complete
    bool stabilizer_left_limit_hit = false; // Track if left limit switch has been hit during raise
    bool stabilizer_right_limit_hit = false; // Track if right limit switch has been hit during raise
    bool stabilizer_left_soft_limit_set = false; // Track if soft limit has been sent to Arduino
    bool stabilizer_right_soft_limit_set = false; // Track if soft limit has been sent to Arduino
};

#endif // DRIVE_MODE_HPP
