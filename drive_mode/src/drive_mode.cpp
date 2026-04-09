///////////////////////////////////////////////////////////////////////////////////
// File description:
///////////////////////////////////////////////////////////////////////////////////

#include "drive_mode/drive_mode.hpp"

DriveMode::DriveMode() : Node("drive_mode"), latest_joy_msg(nullptr)
{
    subscription_data = this->create_subscription<sensor_msgs::msg::Joy>(
        "/controller_data",
        10,
        std::bind(&DriveMode::drive_mode_logic, this, std::placeholders::_1)
    );

    subscription_mode = this->create_subscription<std_msgs::msg::String>(
        "/selected_mode",
        10,
        std::bind(&DriveMode::switch_mode, this, std::placeholders::_1)
    );

    confirmation_publisher = this->create_publisher<std_msgs::msg::String>("/mode_switch_confirmation", 10);

    /*
    // Initialize the serial communication
    try {
		// Change when you're using a different port (type: ls /dev/ttyA* to find the port number)
		serial_port_rover.Open("/dev/ttyACM1"); 
		serial_port_rover.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    } catch (const LibSerial::OpenFailed&) {
        RCLCPP_ERROR(this->get_logger(), "Can't open serial port");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Arduino serial communication initialized.");
    */

    RCLCPP_INFO(this->get_logger(), "Drive Mode Node Started");
}

void DriveMode::switch_mode(const std_msgs::msg::String::SharedPtr msg)
{
    std::string requested_mode = msg->data;

    if (requested_mode == "drive_to_dig" && current_mode == "drive") {
        std_msgs::msg::String confirm_msg;
        confirm_msg.data = is_ready_to_switch();
        confirmation_publisher->publish(confirm_msg);
    }
    else if (requested_mode == "drive") {
        current_mode = "drive";
    }
    else if (requested_mode == "dig") {
        current_mode = "dig";
    }
}

std::string DriveMode::is_ready_to_switch()
{
    // Check if rover is stationary by examining joystick input
    if (!latest_joy_msg) {
        if (debug_mode) {
            RCLCPP_WARN(this->get_logger(), "No joystick data available");
        }
        return "No joystick data available";

    }

    // All drive control axes must be 0 for the rover to be considered stationary
    if (std::abs(latest_joy_msg->axes[0]) > 0 || 
        std::abs(latest_joy_msg->axes[1]) > 0 || 
        std::abs(latest_joy_msg->axes[2]) > 0 || 
        std::abs(latest_joy_msg->axes[3]) > 0) {

        if (debug_mode) {
            RCLCPP_WARN(this->get_logger(), "Rover inputs is not zero, cannot switch modes");
        }
        return "Rover inputs is not zero";
    }

    // Add additional conditions here:
    // - Check arm position is safe
    // - Check sensors are ready
    // etc.

    return "Switch confirmed";
}

void DriveMode::drive_mode_logic(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    latest_joy_msg = msg;
    if (current_mode == "drive") {
        // Implement the logic for controlling the excavator in drive mode based on joystick input
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveMode>());
    rclcpp::shutdown();
    return 0;
}
