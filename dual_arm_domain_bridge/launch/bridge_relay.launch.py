# Plan B one-command launch of the whole bridge layer: endpoint transparent relay
# contents = uplink bridge (mandatory) + endpoint transparent relay (downlink)
#
# usage:
#   ros2 launch dual_arm_domain_bridge bridge_relay.launch.py
#   ros2 launch dual_arm_domain_bridge bridge_relay.launch.py host_domain:=10 arm_a_domain:=20 arm_b_domain:=30
#
# note:
#   - the two programs specify the domain explicitly via set_domain_id() internally (CLI positional arguments),
#     they "do not honor" this terminal's ROS_DOMAIN_ID environment variable — to change the number, change only the parameter here

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_domain = LaunchConfiguration("host_domain")
    arm_a_domain = LaunchConfiguration("arm_a_domain")
    arm_b_domain = LaunchConfiguration("arm_b_domain")

    return LaunchDescription([
        DeclareLaunchArgument(
            "host_domain", default_value="10",
            description="the domain of the host (brain/move_group/RViz)"),
        DeclareLaunchArgument(
            "arm_a_domain", default_value="20",
            description="the domain of Arm A (RA610-1476, big_) stock system"),
        DeclareLaunchArgument(
            "arm_b_domain", default_value="30",
            description="the domain of Arm B (RA605-710, small_) stock system"),

        # uplink bridge: the /joint_states of D_armA/D_armB → prefixed and merged → D_host /joint_states
        Node(
            package="dual_arm_domain_bridge",
            executable="joint_state_uplink_bridge",
            name="joint_state_uplink_bridge",
            output="screen",
            arguments=[host_domain, arm_a_domain, arm_b_domain],
        ),

        # downlink (option C): five-endpoint transparent relay — RViz's native Execute/Stop work, failure semantics = direct connection
        Node(
            package="dual_arm_domain_bridge",
            executable="trajectory_downlink_endpoint_relay",
            name="trajectory_downlink_endpoint_relay",
            output="screen",
            arguments=[host_domain, arm_a_domain, arm_b_domain],
        ),
    ])
