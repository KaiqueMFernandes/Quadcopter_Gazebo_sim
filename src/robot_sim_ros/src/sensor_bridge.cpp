#include <random>

#include "sim_node.hpp"
#include "engine.hpp"
#include "state.hpp"

class SensorBridge : public rclcpp::Node 
{
public:
    SensorBridge() : Node("sensor_node") 
    {
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ground_truth", 10, std::bind(&SensorBridge::controller_callback, this, std::placeholders::_1));

        // 2. PUBLISH the noisy version
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // Initialize your random engine for noise
        rdn_ = std::mt19937(std::random_device{}());
        dist_ = std::normal_distribution<double>(0.0, 0.01); // 1cm standard deviation

        RCLCPP_INFO(this->get_logger(), "Sensor Bridge is listening to Ground Truth...");
    }

private:

    void controller_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto noisy_msg = *msg;

        // Noise
        noisy_msg.pose.pose.position.x += dist_(rdn_);
        noisy_msg.pose.pose.position.y += dist_(rdn_);
        noisy_msg.pose.pose.position.z += dist_(rdn_);

        noisy_msg.header.frame_id = "odom";
        noisy_msg.child_frame_id = "base_link";
        
        pub_->publish(noisy_msg);
    } 

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    std::normal_distribution<double> dist_;
    
    // Member variables for noise
    std::mt19937 rdn_;

}; 

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorBridge>());
    rclcpp::shutdown();

    return 0;
}