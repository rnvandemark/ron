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
#
#    return launch_desc

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace"
    )

    absolute_controller_manager_name = LaunchConfiguration(
        "absolute_controller_manager_name",
        default=[namespace, "/controller_manager"]
    )

    xacro_file = os.path.join(
        get_package_share_directory('ron_description'),
        'urdf',
        'dummy.urdf.xacro'
    )

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-robot_namespace', namespace,
                                   '-entity', 'cartpole'],
                        namespace=namespace,
                        output='both')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[params],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '-c', absolute_controller_manager_name, '--set-state', 'start', 'joint_state_broadcaster'],
        output='both'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '-c', absolute_controller_manager_name, '--set-state', 'start', 'diff_drive_base_controller'],
        output='both'
    )

    group = GroupAction([
        PushRosNamespace(namespace=namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            ])
         ),
    ])

    return LaunchDescription([
        declare_namespace_cmd,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        group,
        node_robot_state_publisher,
        spawn_entity,
    ])
