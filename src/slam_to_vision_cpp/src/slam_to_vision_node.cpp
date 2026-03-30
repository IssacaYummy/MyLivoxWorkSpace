#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>           
#include <tf2/LinearMath/Matrix3x3.h>           
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <cmath>                                   

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

    // 使用 SLAM 的时间戳
    pose_msg.header.stamp = msg->header.stamp;

    // frame_id 使用全局坐标系名称，MAVROS vision 插件内部会处理 ENU 到 NED 的转换
    pose_msg.header.frame_id = "map";

    const double xs = msg->pose.pose.position.x;
    const double ys = msg->pose.pose.position.y;
    const double zs = msg->pose.pose.position.z;

    pose_msg.pose.position.x = ys;   
    pose_msg.pose.position.y = -xs;   
    pose_msg.pose.position.z = zs;

    const double qw = msg->pose.pose.orientation.w;
    const double qx = msg->pose.pose.orientation.x;
    const double qy = msg->pose.pose.orientation.y;
    const double qz = msg->pose.pose.orientation.z;

    // 1. 构建原始四元数
    tf2::Quaternion original_q(qx, qy, qz, qw);

    // 2. 将四元数转换为欧拉角 (Roll, Pitch, Yaw)
    double roll, pitch, yaw;
    tf2::Matrix3x3(original_q).getRPY(roll, pitch, yaw);

    // 3. 反转 roll 轴
    roll = -roll;
    pitch = -pitch;

    // 4. 将修改后的欧拉角转换回四元数
    tf2::Quaternion modified_q;
    modified_q.setRPY(roll, pitch, yaw);

    // 5. 赋给 pose_msg
    pose_msg.pose.orientation.w = modified_q.w();
    pose_msg.pose.orientation.x = modified_q.x();
    pose_msg.pose.orientation.y = modified_q.y();
    pose_msg.pose.orientation.z = modified_q.z();

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
