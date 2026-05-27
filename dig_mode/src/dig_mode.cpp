#include "dig_mode.hpp"

ArmMode::ArmMode() : Node("arm_mode")
{
    subscription_joystick = this->create_subscription<sensor_msgs::msg::Joy>(
        "/controller_data",
        10,
        std::bind(&ArmMode::joystick_callback, this, std::placeholders::_1)
    );

    subscription_mode = this->create_subscription<std_msgs::msg::String>(
        "/selected_mode",
        10,
        std::bind(&ArmMode::switch_mode, this, std::placeholders::_1)
    );

    subscription_serial_status = this->create_subscription<std_msgs::msg::Bool>(
        "status",
        10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            serial_alive = msg->data;
        }
    );

    subscription_debug_data = this->create_subscription<std_msgs::msg::String>(
        "debug",
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            debug_data = msg->data;

            if (debug_mode) {
                RCLCPP_INFO(this->get_logger(), "Debug: %s", msg->data.c_str());
            }
        }
    );

    subscription_error_data = this->create_subscription<std_msgs::msg::String>(
        "error",
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_ERROR(this->get_logger(), "Arduino error: %s", msg->data.c_str());
        }
    );

    subscription_sensor_data = this->create_subscription<std_msgs::msg::String>(
        "sensor",
        10,
        std::bind(&ArmMode::sensor_msg_callback, this, std::placeholders::_1)
    );

    arm_command_publisher = this->create_publisher<std_msgs::msg::String>("cmd", 10);
    confirmation_publisher = this->create_publisher<std_msgs::msg::String>("/mode_switch_confirmation", 10);

    control_timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ArmMode::control_loop, this)
    );

    previous_q_sim = inverse_kinematics(x_a[0], x_a[1], x_a[2]);

    RCLCPP_INFO(this->get_logger(), "Arm Mode Node Started");
}

void ArmMode::switch_mode(const std_msgs::msg::String::SharedPtr msg)
{
    const std::string requested_mode = msg->data;

    if (requested_mode == "dig") {
        current_mode = "dig";

        // When entering dig mode, the arm must first move upward until
        // all four Arduino limit switches are pressed.
        arm_homed = false; // when homing sequence is turned on make sure this is set to false
        homing_command_sent = false; // when homing sequence is turned on make sure this is set to false

        std_msgs::msg::String confirm_msg;
        confirm_msg.data = "Dig mode selected, arm homing started";
        confirmation_publisher->publish(confirm_msg);
    }
    else if (requested_mode == "drive") {
        current_mode = "drive";
        arm_homed = false; // when homing sequence is turned on make sure this is set to false
        homing_command_sent = false; // when homing sequence is turned on make sure this is set to false
    }
}

void ArmMode::joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    latest_joy_msg = msg;

    if (current_mode != "dig" || !arm_homed) {
        return;
    }

    // This node receives the already remapped joystick message:
    //
    // axes[0] = right joystick up/down
    // axes[1] = right joystick left/right
    // axes[2] = left joystick up/down
    // axes[3] = left joystick left/right
    // axes[4] = right trigger
    // axes[5] = left trigger
    //
    // End-effector control:
    // axes[1] controls x
    // axes[2] controls y
    // axes[0] controls z

    if (msg->axes.size() >= 6) {
        xf += msg->axes[1] * joystick_position_speed * Ts;
        yf += msg->axes[2] * joystick_position_speed * Ts;
        zf += msg->axes[0] * joystick_position_speed * Ts;
    }

    // Simple workspace limits.
    xf = clamp(xf, 0.10, 0.65);
    yf = clamp(yf, -0.45, 0.45);
    zf = clamp(zf, 0.05, 0.70);
}

void ArmMode::control_loop()
{
    if (current_mode != "dig") {
        return;
    }

    // First step after switching to dig mode:
    // tell the Arduino to move the arm up until all four limit switches are hit.
    if (!arm_homed) {
        if (!homing_command_sent) {
            std_msgs::msg::String home_msg;
            home_msg.data = "ARM_HOME_UP";
            arm_command_publisher->publish(home_msg);

            homing_command_sent = true;

            RCLCPP_INFO(this->get_logger(), "Sent ARM_HOME_UP command");
        }

        return;
    }

    const std::array<double, 3> x_d = {xf, yf, zf};

    // Force from strain gauge bridge voltage.
    std::array<double, 3> Fext_raw = calculate_external_force_from_voltage(bridge_voltage_UA);

    // Low-pass filter the force.
    std::array<double, 3> Fext;
    for (size_t i = 0; i < 3; ++i) {
        Fext[i] = beta_f * Fext_previous[i] + (1.0 - beta_f) * Fext_raw[i];
        Fext_previous[i] = Fext[i];
    }

    // Limit force magnitude.
    const double force_norm = std::sqrt(
        Fext[0] * Fext[0] +
        Fext[1] * Fext[1] +
        Fext[2] * Fext[2]
    );

    if (force_norm > Fext_max && force_norm > 0.0) {
        for (double &value : Fext) {
            value = (Fext_max / force_norm) * value;
        }
    }

    // Adaptive admittance values.
    std::array<double, 3> Md;
    std::array<double, 3> Kd;
    std::array<double, 3> Bd;

    if (force_norm > F_tsd) {
        Md = Md0;
        Kd = Kd0;
    } else {
        for (size_t i = 0; i < 3; ++i) {
            const double v_abs = std::abs(dx_a[i]);

            Md[i] = clamp(Md0[i] * std::exp(-v_abs / alpha), 0.1, Md0[i]);
            Kd[i] = clamp(alpha * Kd0[i] * std::exp(v_abs), 5.0, Kd0[i]);
        }
    }

    for (size_t i = 0; i < 3; ++i) {
        Bd[i] = 2.0 * zeta * std::sqrt(Md[i] * Kd[i]);
    }

    // Admittance equation:
    // Md*ddx = Fext - Bd*dx - Kd*(x - xd)
    for (size_t i = 0; i < 3; ++i) {
        const double ddx_a = (Fext[i] - Bd[i] * dx_a[i] - Kd[i] * (x_a[i] - x_d[i])) / Md[i];
        dx_a[i] += ddx_a * Ts;
    }

    // Limit velocity.
    const double velocity_norm = std::sqrt(
        dx_a[0] * dx_a[0] +
        dx_a[1] * dx_a[1] +
        dx_a[2] * dx_a[2]
    );

    if (velocity_norm > vmax && velocity_norm > 0.0) {
        for (double &value : dx_a) {
            value = (vmax / velocity_norm) * value;
        }
    }

    // Integrate position.
    for (size_t i = 0; i < 3; ++i) {
        x_a[i] += dx_a[i] * Ts;
    }

    const std::array<double, 4> q_sim = inverse_kinematics(x_a[0], x_a[1], x_a[2]);
    const std::array<double, 4> q_real = convert_sim_to_real_joint_values(q_sim);
    const std::array<double, 4> dq_real = convert_sim_to_real_joint_velocities(q_sim, previous_q_sim);

    previous_q_sim = q_sim;

    std_msgs::msg::String command_msg;
    command_msg.data =
        "ARM_CMD," +
        std::to_string(q_real[0]) + "," +
        std::to_string(q_real[1]) + "," +
        std::to_string(q_real[2]) + "," +
        std::to_string(q_real[3]) + "," +
        std::to_string(dq_real[0]) + "," +
        std::to_string(dq_real[1]) + "," +
        std::to_string(dq_real[2]) + "," +
        std::to_string(dq_real[3]) + "\n";

    arm_command_publisher->publish(command_msg);
}

void ArmMode::sensor_msg_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "ARM_HOME_DONE") {
        arm_homed = true;

        std_msgs::msg::String confirm_msg;
        confirm_msg.data = "Arm homing complete, dig mode ready";
        confirmation_publisher->publish(confirm_msg);

        RCLCPP_INFO(this->get_logger(), "Arduino confirmed all four limit switches");
        return;
    }

    const std::vector<std::string> data = split_string(msg->data, ',');

    if (data.empty() || data[0] != "ARM_STATE") {
        return;
    }

    if (data.size() < 10) {
        RCLCPP_WARN(this->get_logger(), "Invalid ARM_STATE message: %s", msg->data.c_str());
        return;
    }

    try {
        for (size_t i = 0; i < 4; ++i) {
            real_joint_position_feedback[i] = std::stod(data[1 + i]);
            real_joint_velocity_feedback[i] = std::stod(data[5 + i]);
        }

        // The Raspberry Pi only receives bridge output voltage U_A.
        bridge_voltage_UA = std::stod(data[9]);
    }
    catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Could not parse ARM_STATE message: %s", msg->data.c_str());
    }
}

std::array<double, 3> ArmMode::calculate_external_force_from_voltage(double bridge_voltage)
{
    // Strain calculation:
    //
    // epsilon = epsilon_b
    //         = 1/2 * (1 - v) * 4/k * U_A/U_E
    //
    // v   = Poisson ratio
    // k   = strain gauge factor
    // U_A = measured bridge output voltage
    // U_E = excitation voltage

    const double strain =
        0.5 *
        (1.0 - poisson_ratio) *
        (4.0 / gauge_factor_k) *
        (bridge_voltage / excitation_voltage_UE);

    // Bending moment:
    //
    // M_x = epsilon * E * I / y
    //
    // E = Young's modulus of aluminium
    // I = second moment of area
    // y = distance from neutral axis
    //
    // I and y are placeholders for now.

    const double M_x =
        strain *
        youngs_modulus_E *
        second_moment_I /
        distance_to_neutral_axis_y;

    // Load force:
    //
    // F_load = M_x / delta_l
    //
    // Force is applied in one direction only.
    // Here that direction is Z.

    const double F_load = M_x / force_arm_delta_l;

    return {0.0, 0.0, F_load};
}

std::array<double, 4> ArmMode::inverse_kinematics(double x, double y, double z)
{
    const double q1 = std::atan2(y, x);

    const double r = std::sqrt(x * x + y * y);
    const double z_rel = z - l1;
    const double phi = std::atan2(z_rel, r);

    const double yn = r - lee * std::cos(phi);
    const double zn = z_rel - lee * std::sin(phi);

    double c2 = (yn * yn + zn * zn - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);
    c2 = clamp(c2, -1.0, 1.0);

    const double s2 = -std::sqrt(std::max(0.0, 1.0 - c2 * c2));
    const double q3 = std::atan2(s2, c2);

    const double k1 = l2 + l3 * std::cos(q3);
    const double k2 = l3 * std::sin(q3);

    const double q2 = std::atan2(zn, yn) - std::atan2(k2, k1);
    const double q4 = phi - (q2 + q3);

    return {q1, q2, q3, q4};
}

std::array<double, 4> ArmMode::convert_sim_to_real_joint_values(const std::array<double, 4> &q_sim)
{
    std::array<double, 4> q_real;

    q_real[0] = q_sim[0] * joint1_real_factor;
    q_real[1] = q_sim[1] * joint23_real_factor;
    q_real[2] = q_sim[2] * joint23_real_factor;

    const double q4_deg = q_sim[3] * 180.0 / M_PI;
    const double L_old = joint4_angle_to_length_mm(0.0);
    const double L_new = joint4_angle_to_length_mm(q4_deg);

    q_real[3] = 2.0 * M_PI * (L_new - L_old) / joint4_pitch_mm;

    return q_real;
}

std::array<double, 4> ArmMode::convert_sim_to_real_joint_velocities(
    const std::array<double, 4> &q_sim,
    const std::array<double, 4> &q_sim_previous)
{
    const std::array<double, 4> q_real_now = convert_sim_to_real_joint_values(q_sim);
    const std::array<double, 4> q_real_previous = convert_sim_to_real_joint_values(q_sim_previous);

    std::array<double, 4> dq_real;

    for (size_t i = 0; i < 4; ++i) {
        dq_real[i] = (q_real_now[i] - q_real_previous[i]) / Ts;
    }

    return dq_real;
}

double ArmMode::joint4_angle_to_length_mm(double angle_deg)
{
const std::array<double, 82> angles = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
    70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81
};

const std::array<double, 82> lengths = {
    170.0000,
    170.5133,
    171.0396,
    171.5671,
    172.0957,
    172.6252,
    173.1556,
    173.6865,
    174.2178,
    174.7495,
    175.2812,
    175.8128,
    176.3443,
    176.8753,
    177.4059,
    177.9357,
    178.4647,
    178.9927,
    179.5195,
    180.0000,
    180.5692,
    181.0918,
    181.6126,
    182.1316,
    182.6485,
    183.1633,
    183.6759,
    184.1860,
    184.6936,
    185.1985,
    185.7007,
    186.1999,
    186.6961,
    187.1891,
    187.6789,
    188.1652,
    188.6481,
    189.1273,
    189.6027,
    190.0743,
    190.5420,
    191.0055,
    191.4649,
    191.9201,
    192.3708,
    192.8171,
    193.2587,
    193.6957,
    194.1280,
    194.5553,
    194.9777,
    195.3951,
    195.8073,
    196.2143,
    196.6161,
    197.0124,
    197.4032,
    197.7885,
    198.1681,
    198.5421,
    198.9103,
    199.2726,
    199.6289,
    199.9793,
    200.3236,
    200.6618,
    200.9937,
    201.3194,
    201.6388,
    201.9518,
    202.2583,
    202.5583,
    202.8517,
    203.1385,
    203.4187,
    203.6920,
    203.9587,
    204.2184,
    204.4713,
    204.7173,
    204.9563,
    205.1883
};

    angle_deg = clamp(angle_deg, angles.front(), angles.back());

    for (size_t i = 0; i < angles.size() - 1; ++i) {
        if (angle_deg >= angles[i] && angle_deg <= angles[i + 1]) {
            const double t = (angle_deg - angles[i]) / (angles[i + 1] - angles[i]);
            return lengths[i] + t * (lengths[i + 1] - lengths[i]);
        }
    }

    return lengths.back();
}

double ArmMode::clamp(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

std::vector<std::string> ArmMode::split_string(const std::string &text, char delimiter)
{
    std::vector<std::string> result;
    std::stringstream ss(text);
    std::string item;

    while (std::getline(ss, item, delimiter)) {
        result.push_back(item);
    }

    return result;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmMode>());
    rclcpp::shutdown();
    return 0;
}
