#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class Mid360TransformNode : public rclcpp::Node
{
public:
  Mid360TransformNode() : Node("mid360_transform_node")
  {
    // 声明参数（支持运行时传入任意 yaw 角，默认 180°）
    this->declare_parameter<double>("rotation_yaw_deg", 180.0);
    this->declare_parameter<std::string>("input_imu_topic",    "/livox/imu");
    this->declare_parameter<std::string>("input_lidar_topic",  "/livox/lidar");
    this->declare_parameter<std::string>("output_imu_topic",   "/livox/imu_transformed");
    this->declare_parameter<std::string>("output_lidar_topic", "/livox/lidar_transformed");

    double yaw_deg = this->get_parameter("rotation_yaw_deg").as_double();
    double yaw_rad = yaw_deg * M_PI / 180.0;
    cos_yaw_ = std::cos(yaw_rad);  // 180° 时 = -1
    sin_yaw_ = std::sin(yaw_rad);  // 180° 时 =  0

    RCLCPP_INFO(this->get_logger(),
      "MID360 Transform: yaw=%.1f deg  cos=%.4f  sin=%.4f",
      yaw_deg, cos_yaw_, sin_yaw_);

    const auto in_imu    = this->get_parameter("input_imu_topic").as_string();
    const auto in_lidar  = this->get_parameter("input_lidar_topic").as_string();
    const auto out_imu   = this->get_parameter("output_imu_topic").as_string();
    const auto out_lidar = this->get_parameter("output_lidar_topic").as_string();

    imu_pub_   = this->create_publisher<sensor_msgs::msg::Imu>(out_imu, 10);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_lidar, 10);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      in_imu, 10,
      std::bind(&Mid360TransformNode::imuCallback, this, std::placeholders::_1));

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      in_lidar, 10,
      std::bind(&Mid360TransformNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Sub: %s | %s", in_imu.c_str(), in_lidar.c_str());
    RCLCPP_INFO(this->get_logger(), "Pub: %s | %s", out_imu.c_str(), out_lidar.c_str());
  }

private:
  // ── IMU 回调：对角速度和线加速度施加绕 Z 轴旋转 ──────────────
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto out = *msg;  // 复制，保留 header / orientation / 协方差

    // R_z * [wx, wy]^T，wz 不变
    const double wx = msg->angular_velocity.x;
    const double wy = msg->angular_velocity.y;
    out.angular_velocity.x = cos_yaw_ * wx - sin_yaw_ * wy;
    out.angular_velocity.y = sin_yaw_ * wx + cos_yaw_ * wy;

    // R_z * [ax, ay]^T，az 不变
    const double ax = msg->linear_acceleration.x;
    const double ay = msg->linear_acceleration.y;
    out.linear_acceleration.x = cos_yaw_ * ax - sin_yaw_ * ay;
    out.linear_acceleration.y = sin_yaw_ * ax + cos_yaw_ * ay;

    imu_pub_->publish(out);
  }

  // ── 点云回调：对每个点的 x/y 施加绕 Z 轴旋转（z 不变）────────
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto out = *msg;  // 复制，保留 header / fields / layout

    sensor_msgs::PointCloud2Iterator<float> ix(*msg, "x"), iy(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> ox(out,  "x"), oy(out,  "y");

    for (; ix != ix.end(); ++ix, ++iy, ++ox, ++oy) {
      const float x = *ix, y = *iy;
      *ox = static_cast<float>(cos_yaw_ * x - sin_yaw_ * y);
      *oy = static_cast<float>(sin_yaw_ * x + cos_yaw_ * y);
    }

    cloud_pub_->publish(out);
  }

  double cos_yaw_{-1.0};
  double sin_yaw_{ 0.0};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr            imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mid360TransformNode>());
  rclcpp::shutdown();
  return 0;
}