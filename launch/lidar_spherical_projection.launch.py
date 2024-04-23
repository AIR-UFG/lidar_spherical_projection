from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fov_up', default_value='15.0', description='Field of view up'),
        DeclareLaunchArgument('fov_down', default_value='-15.0', description='Field of view down'),
        DeclareLaunchArgument('width', default_value='2048', description='Width of the projection'),
        DeclareLaunchArgument('height', default_value='16', description='Height of the projection'),
        Node(
            package='lidar_spherical_projection',
            executable='lidar_spherical_projection',
            name='lidar_spherical_projection_node',
            parameters=[
                {'fov_up': LaunchConfiguration('fov_up')},
                {'fov_down': LaunchConfiguration('fov_down')},
                {'width': LaunchConfiguration('width')},
                {'height': LaunchConfiguration('height')}
            ],
            remappings=[
                ('/velodyne_points', '/velodyne_points'),
                ('/lidar_spherical_projection', '/lidar_spherical_projection')
            ]
        )
    ])
