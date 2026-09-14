#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/wrench.hpp> 
#include <algorithm>
#include <cmath>

#include "state.hpp"

class TeleOp : public rclcpp::Node
{

public:
    TeleOp() : Node("tele_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&TeleOp::translator_callback, this, std::placeholders::_1));

        // Publish Force/Torque commands back to the Sim Engine
        pub_pilot_force_ = this->create_publisher<geometry_msgs::msg::Wrench>("cmd_target", 10);

        RCLCPP_INFO(this->get_logger(), "Controller Node is active!");

    }

private:
    void translator_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {
        geometry_msgs::msg::Wrench pilot_cmd;

        // [0]LEFTX; [1]LEFTY; [2]RIGHTX; [5]RIGHTY
        constexpr double yaw_rate = 1.57; // rad/s (90 deg/s)
        constexpr double max_tilt = 0.35; // rad (~20 deg)
        constexpr double deadband = 0.10;
        constexpr double alpha = 0.95;

        auto apply_deadband = [&](double x) {
            // Use <= so values exactly at the deadband edge count as centered (avoids stick bias ~0.1 leaking through).
            return (std::abs(x) <= deadband) ? 0.0 : x;
        };

        const double z_raw = apply_deadband(joy->axes[1]);

        const double roll_raw = apply_deadband(joy->axes[3]);
        const double pitch_raw = apply_deadband(joy->axes[4]);
        const double yaw_raw = apply_deadband(joy->axes[0]);

        z_filt_     = alpha * z_filt_      + (1.0 - alpha) * z_raw;
        roll_filt_  = alpha * roll_filt_   + (1.0 - alpha) * roll_raw; 
        pitch_filt_ = alpha * pitch_filt_  + (1.0 - alpha) * pitch_raw; 
        yaw_filt_   = alpha * yaw_filt_    + (1.0 - alpha) * yaw_raw; 

        // Throttle command in [-1, 1], controller maps this to collective thrust.
        pilot_cmd.force.z = z_filt_;

        // Desired attitude commands in body frame.
        pilot_cmd.torque.x = roll_filt_ * max_tilt * -1; // roll  
        pilot_cmd.torque.y = pitch_filt_ * max_tilt; // pitch 
        pilot_cmd.torque.z = yaw_filt_ * yaw_rate; // yaw rate

        pub_pilot_force_->publish(pilot_cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_pilot_force_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    double z_filt_{0.0}, roll_filt_{0.0}, pitch_filt_{0.0}, yaw_filt_{0.0};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleOp>());
    rclcpp::shutdown();
    return 0;
}
