#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>   
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class SlamToVisionNode : public rclcpp::Node
{
public:
  SlamToVisionNode() : Node("slam_to_vision_cpp")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&SlamToVisionNode::odomCallback, this, std::placeholders::_1));

    vision_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/vision_pose/pose", 10);

    // 速度话题，让 EKF 直接融合速度，避免对位置微分引入噪声
    // vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    //   "/mavros/vision_speed/speed_twist_stamped", 10);

    RCLCPP_INFO(this->get_logger(), "slam_to_vision_cpp node started");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id = "map";

    const double xs = msg->pose.pose.position.x;
    const double ys = msg->pose.pose.position.y;
    const double zs = msg->pose.pose.position.z;

    // SLAM 世界系 → ENU（MAVROS 内部负责 ENU→NED 转换）
    pose_msg.pose.position.x =  -ys;
    pose_msg.pose.position.y =  xs;
    pose_msg.pose.position.z =  zs;

    const double qw = msg->pose.pose.orientation.w;
    const double qx = msg->pose.pose.orientation.x;
    const double qy = msg->pose.pose.orientation.y;
    const double qz = msg->pose.pose.orientation.z;

    // tf2::Quaternion original_q(qx, qy, qz, qw);
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(original_q).getRPY(roll, pitch, yaw);

    // // roll  = -roll;
    // // pitch = -pitch;

    // tf2::Quaternion modified_q;
    // modified_q.setRPY(roll, pitch, yaw);

    tf2::Quaternion original_q(qx, qy, qz, qw);
    
    // 创建一个绕 Z 轴旋转 90度 (M_PI/2) 的旋转四元数
    tf2::Quaternion q_rot;
    q_rot.setRPY(0, 0, M_PI / 2.0);
     
    // 左乘进行坐标系旋转 (新姿态 = 旋转矩阵 * 原姿态)
    tf2::Quaternion modified_q = q_rot * original_q;
    modified_q.normalize(); // 归一化防止浮点误差
    
    pose_msg.pose.orientation.w = modified_q.w();
    pose_msg.pose.orientation.x = modified_q.x();
    pose_msg.pose.orientation.y = modified_q.y();
    pose_msg.pose.orientation.z = modified_q.z();

    pose_msg.header.stamp = this->now(); 

    vision_pub_->publish(pose_msg);

    // // ==================== 速度发布 ====================
    // // FAST-LIO2 的 twist.twist.linear 为 SLAM 世界坐标系下的速度
    // // 与位置变换规则完全一致：SLAM(Vx, Vy, Vz) → ENU(Vy, -Vx, Vz)
    // geometry_msgs::msg::TwistStamped vel_msg;
    // vel_msg.header.stamp = msg->header.stamp;
    // vel_msg.header.frame_id = "map";  // ENU 世界系，MAVROS 自动转 NED

    // vel_msg.twist.linear.x =  msg->twist.twist.linear.y;  // SLAM Vy → ENU Vx
    // vel_msg.twist.linear.y = -msg->twist.twist.linear.x;  // SLAM Vx → ENU Vy（取反）
    // vel_msg.twist.linear.z =  msg->twist.twist.linear.z;  // Z 方向一致

    // vel_pub_->publish(vel_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pub_;
  // rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;  
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamToVisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}