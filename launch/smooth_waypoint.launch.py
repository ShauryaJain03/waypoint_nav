from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    rviz_config_file = "/home/shaurya/turtlebot3_ws/src/waypoint_nav/rviz/smooth_trajectory.rviz"

    smoother_node = Node(
        package="waypoint_nav",
        executable="smoother",
        name="smoother",
        output="screen"
    )

    controller_node = TimerAction(
        period=3.0,
        actions=[
            Node(
            package="waypoint_nav",
            executable="controller",
            name="controller",
            output="screen"
        )]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file]
    )

    return LaunchDescription([
        smoother_node,
        controller_node,
        rviz_node
    ])
