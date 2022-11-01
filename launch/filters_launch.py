from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='traversability_layer',
            executable='pcl_filter',
            name='left_pcl_filter',
            remappings=[("traversible_points", "/velodynes/left/traversible_points"), ("non_traversible_points", "/velodynes/left/non_traversible_points")],
        ),
        Node(
            package='traversability_layer',
            executable='pcl_filter_right',
            name='right_pcl_filter',
            remappings=[("/velodynes/left/points", "/velodynes/right/points"), ("traversible_points", "/velodynes/right/traversible_points"), ("non_traversible_points", "/velodynes/right/non_traversible_points")],
        ),
         Node(
            package='traversability_layer',
            executable='pcl_rescale',
            name='center_pcl_filter',
        ),
    ])
