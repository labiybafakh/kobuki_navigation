#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class OdomTfBroadcaster : public rclcpp::Node
{
public:
    OdomTfBroadcaster()
        : Node("odom_tf_broadcaster")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
            std::bind(&OdomTfBroadcaster::odometryCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";

        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;
        transformStamped.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
