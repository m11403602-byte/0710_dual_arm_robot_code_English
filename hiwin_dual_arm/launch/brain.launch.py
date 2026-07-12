from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # load the dual-arm configuration package
    moveit_config = MoveItConfigsBuilder("dual_hiwin", package_name="hiwin_dual_arm").to_moveit_configs()
    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    
    # argument declaration
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))

    # 1. broadcast the Virtual Joints
    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # 2. start the Robot State Publisher (responsible for computing and broadcasting the 12-axis TF tree)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    # 3. start MoveGroup (the MoveIt core brain, responsible for computing collision-avoidance trajectories)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # 4. start RViz2 (the visualization / operator interface)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    return ld