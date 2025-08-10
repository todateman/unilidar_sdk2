import os
import subprocess

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable unitree lidar node (true/false)'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz2 (true/false)'
    )
    
    enable_cloud_arg = DeclareLaunchArgument(
        'enable_cloud',
        default_value='true',
        description='Enable point cloud publishing (true/false)'
    )
    
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='true', 
        description='Enable IMU data publishing (true/false)'
    )
    
    # Get launch configuration
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_cloud = LaunchConfiguration('enable_cloud')
    enable_imu = LaunchConfiguration('enable_imu')
    # Run unitree lidar
    node1 = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
                
                {'initialize_type': 1},
                {'work_mode': 8},
                {'use_system_timestamp': True},
                {'range_min': 0.0},
                {'range_max': 100.0},
                {'cloud_scan_num': 18},

                {'serial_port': '/dev/ttyACM0'},
                {'baudrate': 4000000},

                {'lidar_port': 6101},
                {'lidar_ip': '192.168.1.62'},
                {'local_port': 6203},
                {'local_ip': '192.168.1.2'},
                
                {'cloud_frame': "unilidar_lidar"},
                {'cloud_topic': "unilidar/cloud"},
                {'imu_frame': "unilidar_imu"},
                {'imu_topic': "unilidar/imu"},
                ]
    )

    # Run Rviz
    package_path = subprocess.check_output(['ros2', 'pkg', 'prefix', 'unitree_lidar_ros2']).decode('utf-8').rstrip()
    rviz_config_file = os.path.join(package_path, 'share', 'unitree_lidar_ros2', 'view.rviz')
    print("rviz_config_file = " + rviz_config_file)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log'
    )
    # Build launch description with conditional nodes
    from launch.conditions import IfCondition
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import AnyLaunchDescriptionSource
    
    # Create conditional nodes
    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[
            {'initialize_type': 1},
            {'work_mode': 8},
            {'use_system_timestamp': True},
            {'range_min': 0.0},
            {'range_max': 100.0},
            {'cloud_scan_num': 18},
            {'serial_port': '/dev/ttyACM0'},
            {'baudrate': 4000000},
            {'lidar_port': 6101},
            {'lidar_ip': '192.168.1.62'},
            {'local_port': 6203},
            {'local_ip': '192.168.1.2'},
            {'cloud_frame': "unilidar_lidar"},
            {'cloud_topic': "unilidar/cloud"},
            {'imu_frame': "unilidar_imu"},
            {'imu_topic': "unilidar/imu"},
            {'enable_cloud_publish': enable_cloud},
            {'enable_imu_publish': enable_imu},
        ],
        condition=IfCondition(enable_lidar)
    )
    
    rviz_node_conditional = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log',
        condition=IfCondition(enable_rviz)
    )
    
    # Add static TF publisher for LiDAR frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'unilidar_lidar'],
        output='log'
    )
    
    # Add IMU TF publisher
    static_tf_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        name='imu_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'unilidar_lidar', 'unilidar_imu'],
        output='log'
    )
    
    return LaunchDescription([
        enable_lidar_arg,
        enable_rviz_arg,
        enable_cloud_arg,
        enable_imu_arg,
        static_tf_node,
        static_tf_imu_node,
        lidar_node,
        rviz_node_conditional
    ])