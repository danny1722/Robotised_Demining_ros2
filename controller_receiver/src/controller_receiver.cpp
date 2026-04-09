///////////////////////////////////////////////////////////////////////////////////
// File description:
///////////////////////////////////////////////////////////////////////////////////

#include "controller_receiver/controller_receiver.hpp"

ControllerReceiver::ControllerReceiver() : Node("controller_receiver")
{
    controller_type = ControllerType::NVIDIA;

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy",
        10,
        std::bind(&ControllerReceiver::joy_callback, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/controller_data", 10);

    RCLCPP_INFO(this->get_logger(), "Controller Receiver Node Started");
}

void ControllerReceiver::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    sensor_msgs::msg::Joy new_msg;

    new_msg.axes.resize(6, 0.0);
    new_msg.buttons.resize(10, 0);

    if (controller_type == ControllerType::NVIDIA)
    {
        // AXES REMAPPING
        // 0: Right joystick up/down     → axes[3]
        // 1: Right joystick left/right  → axes[2]
        // 2: Left joystick up/down      → axes[1]
        // 3: Left joystick left/right   → axes[0]
        // 4: Right trigger              → axes[4]
        // 5: Left trigger               → axes[5]

        new_msg.axes[0] = msg->axes[3];
        new_msg.axes[1] = msg->axes[2];
        new_msg.axes[2] = msg->axes[1];
        new_msg.axes[3] = msg->axes[0];
        new_msg.axes[4] = msg->axes[4];
        new_msg.axes[5] = msg->axes[5];

        // BUTTON REMAPPING

        // D-pad (axes → buttons)
        new_msg.buttons[0] = (msg->axes[7] == 1.0) ? 1 : 0;   // up
        new_msg.buttons[1] = (msg->axes[7] == -1.0) ? 1 : 0;  // down
        new_msg.buttons[2] = (msg->axes[6] == 1.0) ? 1 : 0;   // left
        new_msg.buttons[3] = (msg->axes[6] == -1.0) ? 1 : 0;  // right

        // Face buttons
        new_msg.buttons[4] = msg->buttons[3]; // Y
        new_msg.buttons[5] = msg->buttons[0]; // A
        new_msg.buttons[6] = msg->buttons[2]; // X
        new_msg.buttons[7] = msg->buttons[1]; // B

        // Shoulder buttons
        new_msg.buttons[8] = msg->buttons[4]; // Left shoulder
        new_msg.buttons[9] = msg->buttons[5]; // Right shoulder
    }

    publisher_->publish(new_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerReceiver>());
    rclcpp::shutdown();
    return 0;
}
