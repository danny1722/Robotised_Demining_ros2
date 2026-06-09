///////////////////////////////////////////////////////////////////////////////////
// File description: This file implements the DriveMode ROS2 node, responsible for handling rover
// movement in "drive" mode. It subscribes to joystick input and selected mode
// topics, processes user commands, and publishes motor and actuator commands
// to the rover via a serial interface.
///////////////////////////////////////////////////////////////////////////////////

#include "drive_mode.hpp"

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

    subscription_serial_status = this->create_subscription<std_msgs::msg::Bool>(
        "status",
        10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            //if (debug_mode) {
            //    RCLCPP_INFO(this->get_logger(), "Serial status updated: %s", msg->data ? "Alive" : "Dead");
            //}
            serial_alive = msg->data;
        }
    );

    subscription_debug_data = this->create_subscription<std_msgs::msg::String>(
        "debug",
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            if (debug_mode) {
                RCLCPP_INFO(rclcpp::get_logger("DriveMode"), "Debug: %s", msg->data.c_str());
            }
            debug_data = msg->data;
        }
    );

    subscription_sensor_data = this->create_subscription<std_msgs::msg::String>(
        "sensor",
        10,
        std::bind(&DriveMode::sensor_msg_callback, this, std::placeholders::_1)
    );

    subscription_error_data = this->create_subscription<std_msgs::msg::String>(
        "error",
        10,
        [](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_ERROR(rclcpp::get_logger("DriveMode"), "Error: %s", msg->data.c_str());
        }
    );

    confirmation_publisher = this->create_publisher<std_msgs::msg::String>("/mode_switch_confirmation", 10);
    drive_command_publisher = this->create_publisher<std_msgs::msg::String>("cmd", 10);

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

    if (current_left_motor_speed != 0 || current_right_motor_speed != 0) {
        if (debug_mode) {
            RCLCPP_WARN(this->get_logger(), "Rover is still moving, cannot switch modes");
        }
        return "Rover is still moving";
    }

    // Check if both stabilizers (left and right) are calibrated before switching modes
    if (!stabilizer_left_calibrated) {
        if (debug_mode) {
            RCLCPP_WARN(this->get_logger(), "Left stabilizer is not calibrated, cannot switch modes");
        }
        return "Left stabilizer is not calibrated";
    }

    if (!stabilizer_right_calibrated) {
        if (debug_mode) {
            RCLCPP_WARN(this->get_logger(), "Right stabilizer is not calibrated, cannot switch modes");
        }
        return "Right stabilizer is not calibrated";
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
        // Send drive commands to the rover based on joystick input
        
        float left_motor_speed  = msg->axes[1] + msg->axes[0];  // forward + turn
        float right_motor_speed = msg->axes[1] - msg->axes[0];  // forward - turn

        // Find the maximum magnitude
        float max_val = std::max(std::abs(left_motor_speed), std::abs(right_motor_speed));

        // Normalize if needed
        if (max_val > 1.0f) {
            left_motor_speed  /= max_val;
            right_motor_speed /= max_val;
        }

        // Scale to the motor command range (e.g., -255 to 255)
        left_motor_speed  = left_motor_speed  * 255.0f;
        right_motor_speed = right_motor_speed * 255.0f;

        // Create a command string to send to the rover (e.g., "DRIVE,100,100" for full speed forward)
        std_msgs::msg::String drive_command;
        drive_command.data = "DRIVE," + std::to_string(left_motor_speed) + "," + std::to_string(right_motor_speed) + "\n";

        // Send the command over serial
        drive_command_publisher->publish(drive_command);
        
        // Create commands for left and right stabilizers independently
        // Button mapping:
        // buttons[0] = A = Right lower
        // buttons[1] = B = Right raise
        // buttons[2] = X = Left lower
        // buttons[3] = Y = Left raise
        
        // Right stabilizer control (A = lower, B = raise)
        if (msg->buttons[0] == 1) {
            // Right lower (A button)
            std_msgs::msg::String right_command;
            right_command.data = "STABILIZER_RIGHT,LOWER\n";
            drive_command_publisher->publish(right_command);
        }
        else if (msg->buttons[1] == 1) {
            // Right raise (B button)
            std_msgs::msg::String right_command;
            right_command.data = "STABILIZER_RIGHT,RAISE\n";
            drive_command_publisher->publish(right_command);
        }
        else {
            // Right maintain
            std_msgs::msg::String right_command;
            right_command.data = "STABILIZER_RIGHT,MAINTAIN\n";
            drive_command_publisher->publish(right_command);
        }
        
        // Left stabilizer control (X = lower, Y = raise)
        if (msg->buttons[2] == 1) {
            // Left lower (X button)
            std_msgs::msg::String left_command;
            left_command.data = "STABILIZER_LEFT,LOWER\n";
            drive_command_publisher->publish(left_command);
        }
        else if (msg->buttons[3] == 1) {
            // Left raise (Y button)
            std_msgs::msg::String left_command;
            left_command.data = "STABILIZER_LEFT,RAISE\n";
            drive_command_publisher->publish(left_command);
        }
        else {
            // Left maintain
            std_msgs::msg::String left_command;
            left_command.data = "STABILIZER_LEFT,MAINTAIN\n";
            drive_command_publisher->publish(left_command);
        }
    }
}

void DriveMode::sensor_msg_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // Process incoming sensor data if needed
    
    // Split the sensor data string into components ("TYPE,VALUE")
    std::stringstream ss(msg->data);
    std::string type;

    std::getline(ss, type, ',');  // ONLY parse type here

    // Detect when left stabilizer limit switch is hit during raise
    if (type == "STABILIZER_LEFT_LIMIT_HIT") {
        if (!stabilizer_left_limit_hit) {
            stabilizer_left_limit_hit = true;

            if (debug_mode) {
                RCLCPP_INFO(this->get_logger(), "Left stabilizer limit switch detected - fully raised. Lowering slightly to release...");
            }

            // Send command to lower the left stabilizer slightly to release the limit switch
            std_msgs::msg::String lower_command;
            lower_command.data = "STABILIZER_LEFT,LOWER_CALIBRATE\n";
            drive_command_publisher->publish(lower_command);
        }
        return;
    }

    // Detect when right stabilizer limit switch is hit during raise
    if (type == "STABILIZER_RIGHT_LIMIT_HIT") {
        if (!stabilizer_right_limit_hit) {
            stabilizer_right_limit_hit = true;

            if (debug_mode) {
                RCLCPP_INFO(this->get_logger(), "Right stabilizer limit switch detected - fully raised. Lowering slightly to release...");
            }

            // Send command to lower the right stabilizer slightly to release the limit switch
            std_msgs::msg::String lower_command;
            lower_command.data = "STABILIZER_RIGHT,LOWER_CALIBRATE\n";
            drive_command_publisher->publish(lower_command);
        }
        return;
    }

    // Detect when left stabilizer calibration is complete (after lowering from limit)
    if (type == "STABILIZER_LEFT_CALIBRATED") {
        stabilizer_left_calibrated = true;
        stabilizer_left_limit_hit = false;

        if (debug_mode) {
            RCLCPP_INFO(this->get_logger(), "Left stabilizer calibration complete - setting soft limit");
        }

        // Send command to set the calibrated position as soft limit (won't try to raise beyond this)
        if (!stabilizer_left_soft_limit_set) {
            std_msgs::msg::String soft_limit_command;
            soft_limit_command.data = "STABILIZER_LEFT,SET_CALIBRATED_LIMIT\n";
            drive_command_publisher->publish(soft_limit_command);
            stabilizer_left_soft_limit_set = true;
        }
        return;
    }

    // Detect when right stabilizer calibration is complete (after lowering from limit)
    if (type == "STABILIZER_RIGHT_CALIBRATED") {
        stabilizer_right_calibrated = true;
        stabilizer_right_limit_hit = false;

        if (debug_mode) {
            RCLCPP_INFO(this->get_logger(), "Right stabilizer calibration complete - setting soft limit");
        }

        // Send command to set the calibrated position as soft limit (won't try to raise beyond this)
        if (!stabilizer_right_soft_limit_set) {
            std_msgs::msg::String soft_limit_command;
            soft_limit_command.data = "STABILIZER_RIGHT,SET_CALIBRATED_LIMIT\n";
            drive_command_publisher->publish(soft_limit_command);
            stabilizer_right_soft_limit_set = true;
        }
        return;
    }

    if (type == "MOTOR_SPEED") {
        std::string left_speed_str, right_speed_str;

        std::getline(ss, left_speed_str, ',');
        std::getline(ss, right_speed_str, ',');

        if (!left_speed_str.empty()) {
            try {
                current_left_motor_speed = std::stoi(left_speed_str);
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Invalid left speed: '%s'", left_speed_str.c_str());
            }
        }

        if (!right_speed_str.empty()) {
            try {
                current_right_motor_speed = std::stoi(right_speed_str);
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Invalid right speed: '%s'", right_speed_str.c_str());
            }
        }

        if (debug_mode) {
            RCLCPP_INFO(this->get_logger(),
                "Current Motor Speeds - Left: %d, Right: %d",
                current_left_motor_speed,
                current_right_motor_speed);
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveMode>());
    rclcpp::shutdown();
    return 0;
}
