#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Bridges FAST-LIO's non-standard odometry output to the frame names Nav2 expects.
//
// FAST-LIO publishes:  /Odometry  parent=camera_init  child=body
// This node re-publishes: /odom    parent=odom         child=base_link
// and broadcasts the corresponding odom → base_link TF.
//
// No pose math — just a frame rename. The position/orientation data passes
// through unchanged.

class FastLioRemapper : public rclcpp::Node
{
public:
  FastLioRemapper() : Node("fast_lio_remapper")
  {
    // Match FAST-LIO's default publisher QoS (RELIABLE, KeepLast 10)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", qos);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", qos,
      std::bind(&FastLioRemapper::onOdometry, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "fast_lio_remapper ready: /Odometry (camera_init->body) → /odom (odom->base_link)");
  }

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // --- republish with corrected frame names ---
    auto odom = *msg;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";
    odom_pub_->publish(odom);

    // --- broadcast odom → base_link TF ---
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = msg->header.stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id  = "base_link";

    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;
    tf.transform.rotation      = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastLioRemapper>());
  rclcpp::shutdown();
  return 0;
}
