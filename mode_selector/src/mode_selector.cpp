///////////////////////////////////////////////////////////////////////////////////
// File description:
///////////////////////////////////////////////////////////////////////////////////

#include "mode_selector/mode_selector.hpp"

ModeSelector::ModeSelector() : Node("mode_selector"), current_mode("drive"), switch_confirmation_msg(""), buttons_pressed(false), mode_switched_on_hold(false)
{
    joystcik_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "/controller_data",
        10,
        std::bind(&ModeSelector::select_mode, this, std::placeholders::_1)
    );

    confirmation_subscription = this->create_subscription<std_msgs::msg::String>(
        "/mode_switch_confirmation",
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            switch_confirmation_msg = msg->data;
            //RCLCPP_DEBUG(this->get_logger(), "Received confirmation message: %s", switch_confirmation_msg.c_str());
        }
    );

    publisher_ = this->create_publisher<std_msgs::msg::String>("/selected_mode", 10);

    timer_ = this->create_wall_timer(std::chrono::duration<double>(1/mode_publish_rate),
                                     std::bind(&ModeSelector::publish_mode, this));

    RCLCPP_INFO(this->get_logger(), "Mode Selector Node Started");
}

void ModeSelector::select_mode(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    bool pressed = (msg->buttons[8] == 1 && msg->buttons[9] == 1);
    bool switch_confirmed = (switch_confirmation_msg == "Switch confirmed");

    auto now = this->get_clock()->now();

    if (pressed) {
        if (!buttons_pressed) {
            press_start_time = now;
            buttons_pressed = true;         // Start tracking button hold
            mode_switched_on_hold = false;  // Reset hold state for new button press
        } 
        else {
            double duration = (now - press_start_time).seconds();

            if (duration < 2.0) {
                if (current_mode == "drive") {
                    current_mode = "drive_to_dig";
                }
                if (current_mode == "dig") {
                    current_mode = "dig_to_drive";
                }
            }
            
            // If buttons have been held for 2 seconds, check if switch was confirmed and switch modes if so
            if (duration >= 2.0 && !mode_switched_on_hold && switch_confirmed) {
                if (current_mode == "drive_to_dig") {
                    current_mode = "dig";
                } 
                else if (current_mode == "dig_to_drive") {
                    current_mode = "drive";
                }

                RCLCPP_INFO(this->get_logger(), "Mode switched to: %s", current_mode.c_str());

                mode_switched_on_hold = true;
                switch_confirmation_msg = ""; // Reset confirmation for next switch
            }

            if (duration >= 2.0 && !mode_switched_on_hold && !switch_confirmed) {
                RCLCPP_WARN(this->get_logger(), "Mode switch not confirmed, cannot switch modes");
                if (switch_confirmation_msg.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Confirmation message is empty");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Confirmation message: %s", switch_confirmation_msg.c_str());
                }

                mode_switched_on_hold = true; // Prevents repeated warnings
                switch_confirmation_msg = ""; // Reset confirmation for next attempt

                if (current_mode == "drive_to_dig") {
                    current_mode = "drive"; // Revert to original mode if switch not confirmed
                } 
                else if (current_mode == "dig_to_drive") {
                    current_mode = "dig"; // Revert to original mode if switch not confirmed
                }
            }
        }
    } 
    else {
        buttons_pressed = false;
    }
}

void ModeSelector::publish_mode()
{
    std_msgs::msg::String msg_out;
    msg_out.data = current_mode;
    publisher_->publish(msg_out);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeSelector>());
    rclcpp::shutdown();
    return 0;
}
