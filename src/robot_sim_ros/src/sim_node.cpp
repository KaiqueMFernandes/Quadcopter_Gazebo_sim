#include "sim_node.hpp"
#include "state.hpp"   
#include "engine.hpp"
#include "integrator.hpp"

SimEngineNode::SimEngineNode() : Node("sim_engine_node") {
    // 1. Initialize the Publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ground_truth", 10);

    // 2. Initialize the TF Broadcaster
    tf_broadcaster_ptr_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Subscribe to cmd_force
    sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "cmd_force", 10, std::bind(&SimEngineNode::force_callback, this, std::placeholders::_1));

    // Acceleration Publisher
    acc_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "lin_accel", 10);

    // 3. Create the Wall Timer
    auto period = std::chrono::milliseconds(10);
    timer_ = this->create_wall_timer(period, std::bind(&SimEngineNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Simulation Node started!");
}

void SimEngineNode::force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
    current_force_cmd_ = msg->force;
    current_torque_cmd_ = msg->torque;
}

void SimEngineNode::timer_callback() {
    const auto now = this->get_clock()->now();

    if (last_time_step_.nanoseconds() == 0) {
        last_time_step_ = now;
        //current_state_.resetState();
    }

    double dt = (now - last_time_step_).seconds();
    last_time_step_ = now;
    dt = std::clamp(dt, 0.001, 0.03);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Sim Pos: x=%.2f, y=%.2f, z=%.2f", 
            current_state_.position.x(), 
            current_state_.position.y(), 
            current_state_.position.z());

    SimEngineNode::physics_engine(dt);
    //auto current_time = this->get_clock()->now();
    SimEngineNode::odom_publisher(now);
    SimEngineNode::broadcast_tf(now);
}

void SimEngineNode::physics_engine(double dt) {
    // 1. Physics Step (RK4)
    auto physics_lambda = [&](const modular_robot::RobotState::Vector13d& v, double /*time*/) {
        modular_robot::RobotState state;
        state.fromVector(v);

        modular_robot::RobotState::Vector13d derivative = modular_robot::RobotState::Vector13d::Zero();

         // Position change = velocity
        derivative.segment(0, 3) = state.lin_vel;

          // Orientation change (Indices 3-6)
        Eigen::Quaterniond q_dot = core_.computeQuaternionDot(state);
        derivative(3) = q_dot.w();
        derivative(4) = q_dot.x();
        derivative(5) = q_dot.y();
        derivative(6) = q_dot.z();

        // Capture body-frame force and torque from controller
        Eigen::Vector3d f_body(current_force_cmd_.x, 
                               current_force_cmd_.y, 
                               current_force_cmd_.z);
        core_.ext_force = f_body;

        // derivative.segment(7, 3) = core_.computeLinearAcceleration(state);
        Eigen::Vector3d acc_world = core_.computeLinearAcceleration(state);
        derivative.segment(7, 3) = acc_world;
        last_acc_world_ = acc_world;

        Eigen::Vector3d controller_torque(current_torque_cmd_.x,
                                          current_torque_cmd_.y,
                                          current_torque_cmd_.z);
        core_.ext_moment = controller_torque;

        derivative.segment(10, 3) = core_.computeAngAcceleration(state);

        return derivative;
    };

    current_state_ = modular_robot::RK4Integrator<modular_robot::RobotState, modular_robot::RobotState::Vector13d, decltype(physics_lambda)>::integrate(
        current_state_, 
        dt, 
        physics_lambda);
}

void SimEngineNode::odom_publisher(const rclcpp::Time & stamp) {
    // 2. Publish Odometry
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "world";

    // Linear Acceleration
    Eigen::Vector3d acc_world = core_.computeLinearAcceleration(current_state_);
    geometry_msgs::msg::Vector3Stamped acc_msg;
    acc_msg.header.stamp = stamp;
    acc_msg.header.frame_id = "world";
    acc_msg.vector.x = acc_world.x();
    acc_msg.vector.y = acc_world.y();
    acc_msg.vector.z = acc_world.z();
    acc_pub_->publish(acc_msg);
    
    // Position
    odom_msg.pose.pose.position.x = current_state_.position.x();
    odom_msg.pose.pose.position.y = current_state_.position.y();
    odom_msg.pose.pose.position.z = current_state_.position.z();

    // Orientation
    odom_msg.pose.pose.orientation.w = current_state_.orientation.w();
    odom_msg.pose.pose.orientation.x = current_state_.orientation.x();
    odom_msg.pose.pose.orientation.y = current_state_.orientation.y();
    odom_msg.pose.pose.orientation.z = current_state_.orientation.z();

    // Velocities
    odom_msg.twist.twist.linear.x = current_state_.lin_vel.x();
    odom_msg.twist.twist.linear.y = current_state_.lin_vel.y();
    odom_msg.twist.twist.linear.z = current_state_.lin_vel.z();
    odom_msg.twist.twist.angular.x = current_state_.ang_vel.x();
    odom_msg.twist.twist.angular.y = current_state_.ang_vel.y();
    odom_msg.twist.twist.angular.z = current_state_.ang_vel.z();
    
    odom_pub_->publish(odom_msg);
} 

void SimEngineNode::broadcast_tf(const rclcpp::Time & stamp) {
    // 3. Broadcast TF (For Visualization)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";

    t.transform.translation.x = current_state_.position.x();
    t.transform.translation.y = current_state_.position.y();
    t.transform.translation.z = current_state_.position.z();
    
    t.transform.rotation.w = current_state_.orientation.w();
    t.transform.rotation.x = current_state_.orientation.x();
    t.transform.rotation.y = current_state_.orientation.y();
    t.transform.rotation.z = current_state_.orientation.z();

    tf_broadcaster_ptr_->sendTransform(t);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimEngineNode>());
    rclcpp::shutdown();

    return 0;
}
