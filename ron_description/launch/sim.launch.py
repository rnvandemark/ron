from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
#    NAMESPACE = "/"
#
#    NUM_ROBOT_NODES_PARAM_NAME = "num_robot_nodes"
#    CONTROLLER_MANAGER_NODE_NAME_PARAM_NAME = "controller_manager_node_name"
#
#    declared_num_robot_nodes = DeclareLaunchArgument(
#        name=NUM_ROBOT_NODES_PARAM_NAME,
#        default_value="0",
#        description="Number of robot nodes in RON",
#    )
#    num_robot_nodes = LaunchConfiguration(NUM_ROBOT_NODES_PARAM_NAME)
    num_robot_nodes = 1
#
#    declared_controller_manager_node_name = DeclareLaunchArgument(
#        name=CONTROLLER_MANAGER_NODE_NAME_PARAM_NAME,
#        default_value="controller_manager",
#        description="The relative name of the RON system controller manager node",
#    )
#    controller_manager_node_name = LaunchConfiguration(CONTROLLER_MANAGER_NODE_NAME_PARAM_NAME)
#
#    robot_controllers = PathJoinSubstitution([
#        FindPackageShare("ron_description"),
#        "config",
#        "diff_drive_controllers.yaml",
#    ])
#    control_node = Node(
#        package="controller_manager",
#        executable="ros2_control_node",
#        parameters=[robot_description, robot_controllers],
#        output="both",
#    )
#
#    joint_state_broadcaster_spawner = Node(
#        package="controller_manager",
#        executable="spawner",
#        arguments=["joint_state_broadcaster", "-c", CONTROLLER_MANAGER_NODE_NAME],
#        namespace=NAMESPACE,
#    )

    launch_desc = LaunchDescription()
    launch_desc.add_action(
        GroupAction([
            PushRosNamespace(namespace="/robot_node_0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("gazebo_ros2_control_demos"),
                        "launch",
                        "diff_drive.launch.py"
                    ])
                ),
            )
        ])
    )

#    use_rviz = LaunchConfiguration("use_rviz")
#    launch_desc.add_action(
#        DeclareLaunchArgument(
#            "use_rviz",
#            default_value="true",
#            description="Whether or not to launch RViz"
#        )
#    )
#
#    launch_desc.add_action(
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(
#                PathJoinSubstitution([
#                    FindPackageShare("gazebo_ros"),
#                    "launch",
#                    "gazebo.launch.py"
#                ])
#            ),
#        )
#    )
#
#    for i in range(num_robot_nodes):
#        robot_name = "robot_node_{0}".format(i)
#        launch_desc.add_action(
#            IncludeLaunchDescription(
#                PythonLaunchDescriptionSource(
#                    PathJoinSubstitution([
#                        FindPackageShare("ron_description"),
#                        "launch",
#                        "robot_node_bringup.launch.py"
#                    ])
#                ),
#                launch_arguments={
#                    "namespace": robot_name,
#                    "robot_name": robot_name,
#                    "use_rviz": use_rviz,
#                    "x": str(i),
#                    "z": "1.0",
#                }.items()
#            )
#        )

    return launch_desc
