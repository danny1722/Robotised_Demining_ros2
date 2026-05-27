///////////////////////////////////////////////////////////////////////////////////
// File description:
// This file implements the SerialNode ROS2 node, responsible for handling
// bidirectional serial communication between the ROS2 system and an Arduino.
//
// Key responsibilities:
// - Initializing and managing the serial connection to the Arduino
// - Sending commands from ROS2 topics to the Arduino via serial
// - Receiving and parsing serial messages from the Arduino
// - Publishing system status (alive/dead) using a watchdog mechanism
// - Relaying debug, sensor, and error messages to dedicated ROS2 topics
// - Periodically sending heartbeat (PING) messages to verify connectivity
// - Using thread-safe serial writes to prevent data corruption
//
// Communication protocol:
// - Outgoing: "COMMAND,arg1,arg2,...\n"
// - Incoming: prefixed messages such as:
//     * "PONG"                → heartbeat response
//     * "DEBUG,<message>"     → debug information
//     * "SENSOR,<data>"       → sensor readings
//     * "ERROR,<message>"     → error reporting
//
///////////////////////////////////////////////////////////////////////////////////

#include "serial_node.hpp"

SerialNode::SerialNode() : Node("serial_node")
{
    // Declare parameters with defaults (type: ls /dev/ttyA* to find the port number)
    port = this->declare_parameter<std::string>("port", "/dev/ttyACM0");

    RCLCPP_INFO(this->get_logger(), "Using port: %s", port.c_str());

    // Initialize the serial communication
    try {
        // Change when you're using a different port (type: ls /dev/ttyA* to find the port number)
        serial_port.Open(port); 
        serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    } 
    catch (const LibSerial::OpenFailed&) {
        RCLCPP_ERROR(this->get_logger(), "Can't open serial port");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Arduino serial communication initialized.");

    status_pub = create_publisher<std_msgs::msg::Bool>("status", 10);
    debug_pub = create_publisher<std_msgs::msg::String>("debug", 10);
    sensor_pub = create_publisher<std_msgs::msg::String>("sensor", 10);
    error_pub = create_publisher<std_msgs::msg::String>("error", 10);   
    cmd_sub = create_subscription<std_msgs::msg::String>(
        "cmd", 10,
        std::bind(&SerialNode::cmdCallback, this, std::placeholders::_1));

    ping_timer = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&SerialNode::sendPing, this));
    watchdog_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SerialNode::checkConnection, this));

    read_thread = std::thread(&SerialNode::readSerial, this);

    RCLCPP_INFO(this->get_logger(), "Serial Node Started");
}

void writeSerial(LibSerial::SerialPort &port, std::mutex &mtx, const std::string &msg)
{
    std::lock_guard<std::mutex> lock(mtx);
    port.Write(msg);
}

void SerialNode::cmdCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!arduino_alive) return;

    writeSerial(serial_port, serial_mutex, msg->data + "\n");
}

void SerialNode::sendPing()
{
    writeSerial(serial_port, serial_mutex, "PING\n");
}

void SerialNode::checkConnection()
{
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_response_time).count();

    bool alive = dt < 500;

    if (alive != arduino_alive) {
        RCLCPP_WARN(this->get_logger(), alive ? "Arduino connected" : "Arduino lost");
    }

    arduino_alive = alive;

    std_msgs::msg::Bool msg;
    msg.data = alive;
    status_pub->publish(msg);
}

void SerialNode::readSerial()
{
    while (rclcpp::ok()) {
        try {
            std::string line;
            serial_port.ReadLine(line, '\n', 100);

            if (line.empty()) continue;

            last_response_time = std::chrono::steady_clock::now();

            std::stringstream ss(line);
            std::string type;
            std::getline(ss, type, ',');

            if (type == "PONG") {
                arduino_alive = true;
            }
            // Message format: "DEBUG,Some debug info here" or "SENSOR,Some sensor data here"
            // Usefull information for debugguing and monitoring the system
            else if (type == "DEBUG") {
                std::string payload;
                std::getline(ss, payload);

                std_msgs::msg::String msg;
                msg.data = payload;
                debug_pub->publish(msg);
            }
            // Sensor data
            else if (type == "SENSOR") {
                std::string payload;
                std::getline(ss, payload);

                std_msgs::msg::String msg;
                msg.data = payload;
                sensor_pub->publish(msg);
            }

            else if (type == "ERROR") {
                std::string payload;
                std::getline(ss, payload);

                std_msgs::msg::String msg;
                msg.data = payload;
                error_pub->publish(msg);
            }

        } catch (...) {
            // Ignore timeout
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialNode>());
    rclcpp::shutdown();
    return 0;
}
