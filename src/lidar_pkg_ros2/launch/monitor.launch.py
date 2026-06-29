import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'lidar_pkg'

    # Get path of lidar driver configuration yaml
    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'lidar_params.yaml'
    )
    
    # Load robot urdf model file
    urdf_file_name = 'lidar.urdf'
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        urdf_file_name)

    # Read urdf content and pass to robot state publisher
    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()

    # Load pre-saved rviz configuration file
    rviz_config_dir = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'lidar.rviz')

    return LaunchDescription([
        # Robot State Publisher: publish fixed tf frames defined in URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # LiDAR driver node: read serial data and publish /scan topic
        Node(
            package=pkg_name,
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[config_path]
        ),

        # Static TF publisher: transform from base_link to laser frame
        # Args: x y z roll pitch yaw parent_frame child_frame publish_rate(ms)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0.12", "0", "0.08", "0", "0", "0", "base_link", "laser", "100"],
            name="laser_static_tf",
            output="screen"
        ),

        # RViz2 visualization tool with custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir]
        ),
    ])
