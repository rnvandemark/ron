#from launch import LaunchDescription
#from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, LogInfo, RegisterEventHandler
#from launch.event_handlers import OnProcessExit
#from launch.events import Shutdown
#from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
#from launch_ros.actions import Node, PushRosNamespace
#from launch_ros.substitutions import FindPackageShare
#
#def generate_launch_description():
#    launch_desc = LaunchDescription()
#
#    #
#    # Declare the launch arguments
#    #
#
#    namespace = LaunchConfiguration("namespace")
#    declare_namespace_cmd = DeclareLaunchArgument(
#        "namespace",
#        default_value="",
#        description="Top-level namespace"
#    )
#    launch_desc.add_action(declare_namespace_cmd)
#
#    robot_name = LaunchConfiguration("robot_name")
#    launch_desc.add_action(DeclareLaunchArgument(
#        "robot_name",
#        default_value="",
#        description="Unique robot name"
#    ))
#
#    pose = {}
#    for cart_dimension in ["x", "y", "z", "R", "P", "Y"]:
#        pose[cart_dimension] = LaunchConfiguration(cart_dimension)
#        launch_desc.add_action(DeclareLaunchArgument(
#            cart_dimension,
#            default_value="0.0",
#            description="Initial {0} value".format(cart_dimension)
#        ))
#
#    controller_manager_name = LaunchConfiguration(
#        "controller_manager_name",
#        default=[robot_name, "_controller_manager"]
#    )
#
#    #
#    # Get URDF via xacro
#    #
#
#    robot_description={
#        "robot_description": Command([
#            PathJoinSubstitution([FindExecutable(name="xacro")]),
#            " ",
#            PathJoinSubstitution([FindPackageShare("diffbot_description"), "urdf", "diffbot.urdf.xacro"]),
#        ])
#    }
#
#    launch_desc.add_action(
#        Node(
#            package="controller_manager",
#            executable="ros2_control_node",
#            name=controller_manager_name,
#            parameters=[
#                robot_description,
#                PathJoinSubstitution([FindPackageShare("ron_description"), "config", "robot_node_controller_manager.yaml"]),
#            ],
#            output="both",
#        )
#    )
#
##    joint_state_broadcaster_spawner = Node(
##        package="controller_manager",
##        executable="spawner",
##        arguments=[
##            "joint_state_broadcaster",
##            "-c",
##            controller_manager_name,
##        ],
##    )
#
#    rviz_node = Node(
#        package="rviz2",
#        executable="rviz2",
#        name="rviz2",
#        arguments=["-d", PathJoinSubstitution([FindPackageShare("diffbot_description"), "config", "diffbot.rviz"])],
#        output="screen",
#    )
#
#    launch_desc.add_action(
#        GroupAction([
#            PushRosNamespace(
#                namespace=namespace,
#            ),
#            Node(
#                package="robot_state_publisher",
#                executable="robot_state_publisher",
#                parameters=[robot_description],
#                remappings=[
##                    ("diff_drive_controller/cmd_vel_unstamped", "cmd_vel"),
#                ],
#                output="both",
#            ),
#            Node(
#                package="gazebo_ros",
#                executable="spawn_entity.py",
#                arguments=[
#                    "-entity", robot_name,
#                    "-topic", "robot_description",
#                    "-robot_namespace", namespace,
#                    "-x", pose["x"],
#                    "-y", pose["y"],
#                    "-z", pose["z"],
#                    "-R", pose["R"],
#                    "-P", pose["P"],
#                    "-Y", pose["Y"],
#                ],
#                output="screen",
#            ),
##            joint_state_broadcaster_spawner,
##            # Delay rviz start after `joint_state_broadcaster`
##            RegisterEventHandler(
##                event_handler=OnProcessExit(
##                    target_action=joint_state_broadcaster_spawner,
##                    on_exit=[rviz_node],
##                )
##            ),
#            rviz_node,
#            RegisterEventHandler(
#                event_handler=OnProcessExit(
#                    target_action=rviz_node,
#                    on_exit=EmitEvent(event=Shutdown(reason="rviz exited"))
#                )
#            ),
##            # Delay load of diff drive controller after `joint_state_broadcaster`
##            RegisterEventHandler(
##                event_handler=OnProcessExit(
##                    target_action=joint_state_broadcaster_spawner,
##                    on_exit=[
##                        Node(
##                            package="controller_manager",
##                            executable="spawner",
##                            namespace=namespace,
##                            arguments=[
##                                "diff_drive_controller",
##                                "--load-only",
##                                "-c",
##                                controller_manager_name,
##                            ],
##                        )
##                    ],
##                )
##            )
#        ])
#    )
#
#    return launch_desc
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    #
    # Declare launch arguments
    #

    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace"
    )

    robot_name = LaunchConfiguration("robot_name")
    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description="Unique robot name"
    )

    controller_manager_name = LaunchConfiguration("controller_manager_name")
    declare_controller_manager_name_cmd = DeclareLaunchArgument(
        "controller_manager_name",
        default_value="controller_manager",
        description="Name of this robot's controller manager node"
    )

    use_rviz = LaunchConfiguration("use_rviz")
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether or not to launch RViz"
    )

    pose = {}
    declare_pose_cmds = {}
    for cart_dimension in ["x", "y", "z", "R", "P", "Y"]:
        pose[cart_dimension] = LaunchConfiguration(cart_dimension)
        declare_pose_cmds[cart_dimension] = DeclareLaunchArgument(
            cart_dimension,
            default_value="0.0",
            description="Initial {0} value".format(cart_dimension)
        )

    absolute_controller_manager_name = LaunchConfiguration(
        "absolute_controller_manager_name",
        default=["/", namespace, "/", controller_manager_name]
    )

    #
    # Get URDF via xacro
    #

    robot_description = {
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ron_description"), "urdf", "robot_node.urdf.xacro"]),
            " robot_node_name:=",
            robot_name
        ])
    }

#    control_node = Node(
#        package="controller_manager",
#        executable="ros2_control_node",
#        namespace=namespace,
#        parameters=[
#            robot_description,
#            PathJoinSubstitution([FindPackageShare("ron_description"), "config", "robot_node_controller_manager.yaml"])
#        ],
#        output="both",
#    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[
            robot_description,
            {"frame_prefix": robot_name}
        ],
#        remappings=[
#            ("/tf", "tf"),
#            ("/tf_static", "tf_static"),
#        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        output="log",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("ron_description"), "config", "example.rviz"])],
        condition=IfCondition(use_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", absolute_controller_manager_name],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", absolute_controller_manager_name],
    )

    log_entity_pose = LogInfo(msg=[
        "'", robot_name, "' entity spawn at [x=", pose["x"], ", y=", pose["y"], ", z=", pose["z"],
        ", R=", pose["R"], ", P=", pose["P"], ", Y=", pose["Y"], "]"
    ])

    entity_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace=namespace,
        arguments=[
            "-entity", robot_name,
            "-topic", "robot_description",
            "-robot_namespace", namespace,
            "-x", pose["x"],
            "-y", pose["y"],
            "-z", pose["z"],
            "-R", pose["R"],
            "-P", pose["P"],
            "-Y", pose["Y"],
        ],
        output="screen",
    )

    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=entity_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=entity_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        declare_namespace_cmd,
        declare_robot_name_cmd,
        declare_controller_manager_name_cmd,
        declare_use_rviz_cmd,
        declare_pose_cmds["x"],
        declare_pose_cmds["y"],
        declare_pose_cmds["z"],
        declare_pose_cmds["R"],
        declare_pose_cmds["P"],
        declare_pose_cmds["Y"],
        log_entity_pose,
        delay_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        delay_rviz,
#        control_node,
        robot_state_pub_node,
        entity_spawner,
    ]

    return LaunchDescription(nodes)
