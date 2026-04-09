#ifndef MODE_SELECTOR_HPP
#define MODE_SELECTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

class ModeSelector : public rclcpp::Node
{
public:
    ModeSelector();

    const double mode_publish_rate = 10; // How often the current mode is published per second

private:
    void select_mode(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publish_mode();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystcik_subscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr confirmation_subscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    std::string current_mode;
    std::string switch_confirmation_msg;
    bool buttons_pressed;       // Tracks if buttons are currently pressed
    bool mode_switched_on_hold; // Tracks if mode has already been switched during the current button hold
    rclcpp::Time press_start_time;
};

#endif // MODE_SELECTOR_HPP
