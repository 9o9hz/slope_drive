from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for starting all nodes in the slope_drive package.
    """
    return LaunchDescription([
        Node(
            package='slope_drive',
            executable='imu_orientation_node',
            name='imu_orientation_node',
            output='screen'
        ),
        Node(
            package='slope_drive',
            executable='bev_projection_node',
            name='bev_projection_node',
            output='screen'
        ),
        Node(
            package='slope_drive',
            executable='edge_lane_node',
            name='edge_lane_node',
            output='screen'
        ),
        Node(
            package='slope_drive',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen'
        ),
        Node(
            package='slope_drive',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen'
        ),
    ])
