from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch turtlesim_node
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="sim"
        ),

        # Launch our custom turtle controller
        Node(
            package="assignment",  # <-- replace with the name of your package
            executable="traj_turtle",  # <-- must match entry point or script name in setup.py
            name="controller",
            output="screen"
        )
    ])

