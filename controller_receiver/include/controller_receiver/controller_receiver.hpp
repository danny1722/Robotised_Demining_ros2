#ifndef CONTROLLER_RECEIVER_HPP
#define CONTROLLER_RECEIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class ControllerReceiver : public rclcpp::Node
{
public:
    ControllerReceiver();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    enum class ControllerType
    {
        NVIDIA,
        XBOX
    };

    ControllerType controller_type;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
};

#endif // CONTROLLER_RECEIVER_HPP