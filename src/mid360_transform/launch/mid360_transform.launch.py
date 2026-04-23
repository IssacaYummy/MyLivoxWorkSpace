from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mid360_transform',
            executable='mid360_transform_node',
            name='mid360_transform_node',
            output='screen',
            parameters=[{
                'rotation_yaw_deg':   180.0,          # 修改此值可支持其他安装方向
                'input_imu_topic':    '/livox/imu',
                'input_lidar_topic':  '/livox/lidar',
                'output_imu_topic':   '/livox/imu_transformed',
                'output_lidar_topic': '/livox/lidar_transformed',
            }]
        ),
    ])