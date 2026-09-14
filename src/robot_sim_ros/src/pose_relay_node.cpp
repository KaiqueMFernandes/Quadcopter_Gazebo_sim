#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

class PoseRelayNode : public rclcpp::Node
{
public:
    PoseRelayNode() : Node("pose_relay_node")
    {
        client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(
            "/world/default/set_pose");

        sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "ground_truth", 10,
            std::bind(&PoseRelayNode::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Pose relay node started");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!client_->service_is_ready()) return;

        auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
        request->entity.name = "quadcopter";
        request->entity.type = 2; // MODEL
        request->pose.position    = msg->pose.pose.position;
        request->pose.orientation = msg->pose.pose.orientation;

        // Fire-and-forget: don't block on the response at 100Hz
        client_->async_send_request(request);
    }

    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseRelayNode>());
    rclcpp::shutdown();
    return 0;
}