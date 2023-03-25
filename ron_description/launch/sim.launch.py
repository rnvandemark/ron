from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    NAMESPACE = "/"

    NUM_ROBOT_NODES_PARAM_NAME = "num_robot_nodes"
    CONTROLLER_MANAGER_NODE_NAME_PARAM_NAME = "controller_manager_node_name"

    declared_num_robot_nodes = DeclareLaunchArgument(
        name=NUM_ROBOT_NODES_PARAM_NAME,
        default_value="0",
        description="Number of robot nodes in RON",
    )
    num_robot_nodes = LaunchConfiguration(NUM_ROBOT_NODES_PARAM_NAME)

    declared_controller_manager_node_name = DeclareLaunchArgument(
        name=CONTROLLER_MANAGER_NODE_NAME_PARAM_NAME,
        default_value="controller_manager",
        description="The relative name of the RON system controller manager node",
    )
    controller_manager_node_name = LaunchConfiguration(CONTROLLER_MANAGER_NODE_NAME_PARAM_NAME)

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("ron_description"),
        "config",
        "diff_drive_controllers.yaml",
    ])
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        namespace=NAMESPACE,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", CONTROLLER_MANAGER_NODE_NAME],
        namespace=NAMESPACE,
    )

    launch_desc_actions = [
        declared_num_robot_nodes,
        control_node,
        joint_state_broadcaster_spawner,
    ]
    for i in range(num_robot_nodes):
        robot_description = {"robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ron_description"), "urdf", "robot_node.urdf.xacro"]
            ),
            " ",
            NODE_ID_PARAM_NAME,
            ":=",
            i,
        ])}
        robot_state_pub_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
            remappings=[
                (NAMESPACE + "/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ],
            namespace=NAMESPACE,
        )
        launch_desc_actions.append(robot_state_pub_node)

        robot_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "-c", NAMESPACE + CONTROLLER_MANAGER_NODE_NAME],
            namespace=NAMESPACE,
        )

        # Delay start of robot_controller after `joint_state_broadcaster`
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
        launch_desc_actions.append(delay_robot_controller_spawner_after_joint_state_broadcaster_spawner)

    # Delay rviz start after `joint_state_broadcaster`
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ron_description"), "config", "example.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        namespace=NAMESPACE,
    )
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    launch_desc_actions.append(delay_rviz_after_joint_state_broadcaster_spawner)

    return LaunchDescription(launch_desc_actions)
