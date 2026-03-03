#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class SlamToVisionNode : public rclcpp::Node
{
public:
  SlamToVisionNode() : Node("slam_to_vision_cpp")
  {
    // 订阅 FAST-LIO2 输出的 /Odometry（nav_msgs::msg::Odometry）
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&SlamToVisionNode::odomCallback, this, std::placeholders::_1));

    // 发布到 MAVROS 的 vision_pose 话题（geometry_msgs::msg::PoseStamped）
    vision_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/vision_pose/pose", 10);

    RCLCPP_INFO(this->get_logger(), "slam_to_vision_cpp node started");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose_msg;

    // 使用 SLAM 的时间戳（也可以用 this->now()）
    pose_msg.header.stamp = msg->header.stamp;

    // 这里 frame_id 用 SLAM 的全局坐标系名称（例如 "map" 或 "odom"）
    // MAVROS 的 vision 插件内部会做 ENU -> NED 的转换
    pose_msg.header.frame_id = "map";

    // 直接拷贝姿态（位置+四元数），不做坐标系变换（先跑通，后面需要时再细调）
    pose_msg.pose = msg->pose.pose;

    vision_pub_->publish(pose_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamToVisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
