#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"


class JoyToVelocity : public rclcpp::Node
{
public:
  JoyToVelocity() : Node("JoyToVelocity")
  {
    this->declare_parameter<double>("scale_linear", 0.6);
    this->declare_parameter<double>("scale_angular", 0.6);
    this->get_parameter("scale_linear", scale_linear_);
    this->get_parameter("scale_angular", scale_angular_);

    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyToVelocity::joy_callback, this, std::placeholders::_1));

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("commands/velocity", 100);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x= scale_linear_ * joy_msg->axes[1];
    twist_msg.angular.z= scale_angular_ * joy_msg->axes[0];
    twist_publisher_->publish(twist_msg);

    RCLCPP_INFO(rclcpp::get_logger("callback-joy"), "linear: %.2f, angular %.2f", joy_msg->axes[1], joy_msg->axes[0]);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  double scale_linear_;
  double scale_angular_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToVelocity>());
  rclcpp::shutdown();
  return 0;
}
