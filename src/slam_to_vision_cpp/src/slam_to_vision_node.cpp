#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>   
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
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

    // 预计算对齐变换矩阵，避免在高频回调中重复构造
    tf2::Quaternion q_rot;
    q_rot.setRPY(0.0, 0.0, M_PI / 2.0);   // Rz(+90°): camera_init FLU → ENU
    align_transform_.setIdentity();
    align_transform_.setRotation(q_rot);

    RCLCPP_INFO(this->get_logger(), "slam_to_vision_cpp node started");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 获取 SLAM 的原始位姿 (位置 + 姿态)
    tf2::Transform slam_pose;
    tf2::fromMsg(msg->pose.pose, slam_pose);

    // 将纠正矩阵应用于 SLAM 位姿 (平移和姿态同时发生正确变换)
    tf2::Transform enu_pose = align_transform_ * slam_pose;

    // 打包回 PoseStamped 给 MAVROS
    geometry_msgs::msg::PoseStamped pose_msg;
    
    // 时间戳说明:
    //   使用 msg->header.stamp (传感器时间): EKF 时间对齐更准确，需保证
    //     PC 与飞控时钟同步 (建议使用 chrony 或 mavros timesync)。
    pose_msg.header.stamp    = msg->header.stamp;
    pose_msg.header.frame_id = "map";

    // 将转换后的XYZ和四元数安全、统一地写回pose_msg
    tf2::toMsg(enu_pose, pose_msg.pose);
    vision_pub_->publish(pose_msg);

    // ── [可选] 同时发布速度，提升 EKF 融合质量 ────────────────
    // FAST-LIO 的 twist.twist.linear 是 camera_init 世界系下的速度向量，
    // 需与位置使用相同的 Rz(+90°) 旋转:
    //   v_enu_x = -v_slam_y  (East  = -SLAM_Vy)  ← 原代码符号有误，已修正
    //   v_enu_y = +v_slam_x  (North = +SLAM_Vx)  ← 原代码符号有误，已修正
    //   v_enu_z = +v_slam_z  (Up    = +SLAM_Vz)
    //
    // geometry_msgs::msg::TwistStamped vel_msg;
    // vel_msg.header.stamp    = msg->header.stamp;
    // vel_msg.header.frame_id = "map";  // ENU 世界系，MAVROS 自动转 NED
    //
    // vel_msg.twist.linear.x = -msg->twist.twist.linear.y;  // -SLAM_Vy → ENU Vx (East)
    // vel_msg.twist.linear.y =  msg->twist.twist.linear.x;  // +SLAM_Vx → ENU Vy (North)
    // vel_msg.twist.linear.z =  msg->twist.twist.linear.z;  // +SLAM_Vz → ENU Vz (Up)
    //
    // vel_pub_->publish(vel_msg);
  }
  
  tf2::Transform align_transform_;  // 预计算对齐矩阵 Rz(+90°)

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