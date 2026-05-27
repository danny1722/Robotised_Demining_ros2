#ifndef ARM_MODE_HPP
#define ARM_MODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

class ArmMode : public rclcpp::Node
{
public:
    ArmMode();

private:
    void switch_mode(const std_msgs::msg::String::SharedPtr msg);
    void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void sensor_msg_callback(const std_msgs::msg::String::SharedPtr msg);
    void control_loop();

    std::array<double, 4> inverse_kinematics(double x, double y, double z);
    std::array<double, 3> calculate_external_force_from_voltage(double bridge_voltage);
    std::array<double, 4> convert_sim_to_real_joint_values(const std::array<double, 4> &q_sim);
    std::array<double, 4> convert_sim_to_real_joint_velocities(
        const std::array<double, 4> &q_sim,
        const std::array<double, 4> &q_sim_previous);

    double joint4_angle_to_length_mm(double angle_deg);
    double clamp(double value, double min_value, double max_value);
    std::vector<std::string> split_string(const std::string &text, char delimiter);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_joystick;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_mode;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_sensor_data;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_debug_data;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_error_data;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_serial_status;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_command_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr confirmation_publisher;

    rclcpp::TimerBase::SharedPtr control_timer;

    std::string current_mode = "drive";

    bool debug_mode = true;
    bool serial_alive = false;
    bool arm_homed = false;
    bool homing_command_sent = false;

    std::string debug_data = "";
    sensor_msgs::msg::Joy::SharedPtr latest_joy_msg;

    // Robot geometry [m]
    const double l1 = 0.0713632;
    const double l2 = 0.3;
    const double l3 = 0.3;
    const double lee = 0.0943548;

    // Control loop: 50 Hz
    const double Ts = 0.02;

    // Admittance control parameters
    const double zeta = 0.8;
    const double alpha = 1.6;
    const double F_tsd = 20.0;

    const std::array<double, 3> Md0 = {3.0, 3.0, 3.0};
    const std::array<double, 3> Kd0 = {300.0, 300.0, 300.0};

    const double Fext_max = 20.0;
    const double vmax = 0.9;
    const double beta_f = 0.85;

    // Desired end-effector target position [m]
    double xf = 0.5;
    double yf = 0.0;
    double zf = 0.3;

    // Joystick movement speed [m/s]
    const double joystick_position_speed = 0.15;

    // Admittance state
    std::array<double, 3> x_a = {0.5, 0.0, 0.3};
    std::array<double, 3> dx_a = {0.0, 0.0, 0.0};
    std::array<double, 3> Fext_previous = {0.0, 0.0, 0.0};

    // Arduino feedback
    std::array<double, 4> real_joint_position_feedback = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> real_joint_velocity_feedback = {0.0, 0.0, 0.0, 0.0};

    // Raspberry Pi receives only bridge output voltage U_A
    double bridge_voltage_UA = 0.0;

    // Previous simulated joint values for velocity calculation
    std::array<double, 4> previous_q_sim = {0.0, 0.0, 0.0, 0.0};

    // Simulated-to-real conversion factors
    const double joint1_real_factor = 4.0;
    const double joint23_real_factor = 99.5 * 3.0 * 2.0;

    // Joint 4 actuator pitch [mm/revolution]
    const double joint4_pitch_mm = 1.25;

    // Strain gauge / force calculation constants
    const double poisson_ratio = 0.33;
    const double gauge_factor_k = 2.1;
    const double excitation_voltage_UE = 5.0;

    // Aluminium Young's modulus [Pa]
    const double youngs_modulus_E = 69.0e9;

    // Temporary values until real geometry is measured
    const double second_moment_I = 1.06e-6; // [kg*m^2] 
    const double distance_to_neutral_axis_y = 0.00625; // [m], half of 12.5 mm height
    const double force_arm_delta_l = 0.137; // [m]
};

#endif // ARM_MODE_HPP
