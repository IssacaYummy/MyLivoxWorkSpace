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

    RCLCPP_INFO(this->get_logger(), "slam_to_vision_cpp node started");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 1. 获取 SLAM 的原始位姿 (位置 + 姿态)
    tf2::Transform slam_pose;
    tf2::fromMsg(msg->pose.pose, slam_pose);

    // 2. 根据测试得出的结论，构造一个“纠正矩阵”
    tf2::Transform align_transform;
    align_transform.setIdentity(); // 初始化为无变换

    tf2::Quaternion q_rot;
    // 绕Z轴旋转90度 (M_PI/2) 以对齐 MAVROS 的 ENU (注意：此角度需实际手持测试确认方向)
    q_rot.setRPY(0.0, 0.0, M_PI / 2.0); 
    align_transform.setRotation(q_rot);

    // 3. 将纠正矩阵应用于 SLAM 位姿 (平移和姿态同时发生正确变换)
    tf2::Transform enu_pose = align_transform * slam_pose;

    // 4. 打包回 PoseStamped 给 MAVROS
    geometry_msgs::msg::PoseStamped pose_msg;
    
    // 强制对齐 PC 局部时间，避免时间戳飘移
    pose_msg.header.stamp = this->now(); 
    pose_msg.header.frame_id = "map";

    // 将转换后的XYZ和四元数安全、统一地写回pose_msg
    tf2::toMsg(enu_pose, pose_msg.pose);

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