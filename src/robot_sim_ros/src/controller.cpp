#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "state.hpp"
#include "engine.hpp"

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("controller_node")
    {
        latest_pilot_msg_ = geometry_msgs::msg::Wrench{};
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&Controller::odom_callback, this, std::placeholders::_1));

        command_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
            "cmd_target", 10, std::bind(&Controller::pilot_callback, this, std::placeholders::_1));

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Wrench>("cmd_force", 10);

        RCLCPP_INFO(this->get_logger(), "Controller Node is active!");
    }

private:
    void pilot_callback(const geometry_msgs::msg::Wrench::SharedPtr pilot_msg)
    {
        latest_pilot_msg_ = *pilot_msg;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_rx_ = msg->twist.twist.angular.x;
        current_ry_ = msg->twist.twist.angular.y;
        current_rz_ = msg->twist.twist.angular.z;

        // Vertical position and velocity (world frame, z-up).
        current_z_  = msg->pose.pose.position.z;
        current_vz_ = msg->twist.twist.linear.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        current_orientation_ = q;
        tf2::Matrix3x3 m(q);
        m.getRPY(current_roll_, current_pitch_, current_yaw_);

        rclcpp::Time now = this->now();
        double dt = first_run_? nominal_dt_ : (now - last_time_).seconds();
        last_time_ = now;

        if (first_run_) {
            // Match "hold current altitude" when throttle is neutral: avoid fighting z=0 vs actual spawn height.
            target_z_ = current_z_;
            z_integral_ = 0.0;
        }
        first_run_ = false;

        compute_and_publish(dt);
    }

    void compute_and_publish(double dt) {

        geometry_msgs::msg::Wrench final_cmd;

        // Desired attitude and yaw rate from pilot (body frame).
        double desired_roll = latest_pilot_msg_.torque.x;
        double desired_pitch = latest_pilot_msg_.torque.y;
        double desired_yaw_rate = latest_pilot_msg_.torque.z;

        // Altitude command: when stick is centered, hold the current altitude.
        // Integrate pilot z as a slow bias on the target altitude.
        // Read pilot Z input
        double z_cmd_input = latest_pilot_msg_.force.z; // [-1, 1]

        // Compute desired vertical velocity based on stick
        double desired_vz = z_cmd_input * z_rate_gain_;

        // Integrate to update target altitude
        target_z_ += desired_vz * dt;

        double z_error = target_z_ - current_z_;
        double vz_error = desired_vz - current_vz_;

        // Integral update with anti-windup
        z_integral_ += z_error * dt;
        z_integral_ = std::clamp(z_integral_, -z_integral_limit_, z_integral_limit_);

        // PID altitude correction
        double alt_correction = alt_kp_ * z_error
                              + alt_kd_ * vz_error
                              + alt_ki_ * z_integral_;

        double hover_thrust = core_.mass * 9.81;
        double collective_thrust = hover_thrust + alt_correction;
        collective_thrust = std::clamp(collective_thrust, 0.0, max_collective_thrust_);

        double roll_error = desired_roll - current_roll_;
        double pitch_error = desired_pitch - current_pitch_;
        double tau_x = (att_kp_ * roll_error) - (att_kd_ * current_rx_);
        double tau_y = (att_kp_ * pitch_error) - (att_kd_ * current_ry_);
        double tau_z = (yaw_kp_ * desired_yaw_rate) - (yaw_kd_ * current_rz_);

        //double vel_damping = 0.3;
//
        //tau_x -= vel_damping * current_ry_; // couples motion damping
        //tau_y -= vel_damping * current_rx_;

        double l = core_.arm_length / std::sqrt(2.0);
        double c_yaw = yaw_moment_per_thrust_;
        if (std::abs(l) < 1e-6 || std::abs(c_yaw) < 1e-6) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Invalid mixer parameters (arm_length or yaw_moment_per_thrust too small).");
            return;
        }
        if (core_.motor_thrust_coeff <= 0.0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Invalid motor_thrust_coeff (<= 0).");
            return;
        }

        std::array<double, 4> rotor_thrusts{};
        rotor_thrusts[0] = collective_thrust / 4.0 
                            + tau_x / (4.0 * l) 
                            + tau_y / (4.0 * l) 
                            - tau_z / (4.0 * c_yaw); // front-left

        rotor_thrusts[1] = collective_thrust / 4.0 
                            - tau_x / (4.0 * l) 
                            + tau_y / (4.0 * l) 
                            + tau_z / (4.0 * c_yaw); // front-right

        rotor_thrusts[2] = collective_thrust / 4.0 
                            + tau_x / (4.0 * l) 
                            - tau_y / (4.0 * l) 
                            + tau_z / (4.0 * c_yaw); // rear-left

        rotor_thrusts[3] = collective_thrust / 4.0 
                            - tau_x / (4.0 * l) 
                            - tau_y / (4.0 * l) 
                            - tau_z / (4.0 * c_yaw); // rear-right

        // clmaping due to yaw
        for (double &t : rotor_thrusts) {
                t = std::clamp(t, 0.0, max_rotor_thrust_);
            }

        rotor_speeds_[0] = std::sqrt(rotor_thrusts[0] / core_.motor_thrust_coeff);
        rotor_speeds_[1] = std::sqrt(rotor_thrusts[1] / core_.motor_thrust_coeff);
        rotor_speeds_[2] = std::sqrt(rotor_thrusts[2] / core_.motor_thrust_coeff);
        rotor_speeds_[3] = std::sqrt(rotor_thrusts[3] / core_.motor_thrust_coeff);

        const double total_thrust = rotor_thrusts[0] + rotor_thrusts[1] + rotor_thrusts[2] + rotor_thrusts[3];
        // Publish total thrust in body frame (+Z).
        final_cmd.force.x = 0.0;
        final_cmd.force.y = 0.0;
        final_cmd.force.z = total_thrust;

        final_cmd.torque.x = l * ((rotor_thrusts[0] + rotor_thrusts[2]) - (rotor_thrusts[1] + rotor_thrusts[3]));
        final_cmd.torque.y = l * ((rotor_thrusts[0] + rotor_thrusts[1]) - (rotor_thrusts[2] + rotor_thrusts[3]));
        final_cmd.torque.z = c_yaw * ((rotor_thrusts[1] + rotor_thrusts[2]) - (rotor_thrusts[0] + rotor_thrusts[3]));

        pub_cmd_->publish(final_cmd);
    }

    geometry_msgs::msg::Wrench latest_pilot_msg_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr command_sub_;

    double att_kp_ = 2.0; 
    double att_kd_ = 0.5; 
    double yaw_kp_ = 1.0; 
    double yaw_kd_ = 1.3; 

    // Altitude hold gains
    double alt_kp_ = 10.0;
    double alt_kd_ = 2.0;
    double alt_ki_ = 1.0; 
    double z_rate_gain_ = 5.0;     // m/s per full-stick deflection
    double nominal_dt_ = 0.01;     // seconds, matches 100 Hz update

    double z_integral_ = 0.0;  // accumulated error
    double z_integral_limit_ = 2.0; // anti-windup clamp

    double max_collective_thrust_ = 25.0;
    double max_rotor_thrust_ = 8.0;
    double yaw_moment_per_thrust_ = 0.02;

    double current_rx_ = 0.0, current_ry_ = 0.0, current_rz_ = 0.0;
    double current_roll_ = 0.0, current_pitch_ = 0.0;
    double current_yaw_ = 0.0;
    double current_z_ = 0.0;
    double current_vz_ = 0.0;
    double target_z_ = 0.0;
    tf2::Quaternion current_orientation_{0.0, 0.0, 0.0, 1.0};
    std::array<double, 4> rotor_speeds_ = {0.0, 0.0, 0.0, 0.0};
    rclcpp::Time last_time_;
    bool first_run_ = true;

    modular_robot::PhysicsCore core_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
