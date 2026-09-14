#ifndef SIM_NODE_HPP
#define SIM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <random>

#include "engine.hpp"

class SimEngineNode : public rclcpp::Node {
public:
    SimEngineNode();

private:
    void timer_callback();
    void physics_engine(double dt);
    void broadcast_tf(const rclcpp::Time & stamp);
    void odom_publisher(const rclcpp::Time & stamp);
    void force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg);

    // ROS 2 Components
    rclcpp::Time last_time_step_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_;
    geometry_msgs::msg::Vector3 current_force_cmd_;
    geometry_msgs::msg::Vector3 current_torque_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr acc_pub_;

    // Physics Engine
    modular_robot::PhysicsCore core_;
    modular_robot::RobotState current_state_;
    Eigen::Vector3d last_acc_world_;

    // Parameters
    double dt_ = 0.01; // 100Hz

    // Noise
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr noisy_odom_pub_;
    std::mt19937 rdn_;
}; 

#endif