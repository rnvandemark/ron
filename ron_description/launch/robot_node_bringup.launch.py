from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ROBOT_NODE_PARAM_NAME = "robot_node"
    CONTROLLER_MANAGER_NODE_NAME_PARAM_NAME = "controller_manager_node_name"
    JOINT_STATE_BROADCASTER_NODE_NAME_PARAM_NAME = "joint_state_broadcaster_node_name"
    DIFF_DRIVE_CONTROLLER_NODE_NAME_PARAM_NAME = "diff_drive_controller_node_name"

    ROBOT_NODE_NAME_PARAM_NAME = "robot_node_name"

    declared_node_id = DeclareLaunchArgument(
        name=NODE_ID_PARAM_NAME,
        default_value="0",
        description="ID of this RON robot node",
    )
    node_id = LaunchConfiguration(NODE_ID_PARAM_NAME)

    declared_controller_manager_node_name = DeclareLaunchArgument(
        name=CONTROLLER_MANAGER_NODE_NAME_PARAM_NAME,
        default_value="controller_manager",
        description="The relative name of the RON system controller manager node",
    )
    controller_manager_node_name = LaunchConfiguration(CONTROLLER_MANAGER_NODE_NAME_PARAM_NAME)

    declared_joint_state_broadcaster_node_name = DeclareLaunchArgument(
        name=JOINT_STATE_BROADCASTER_NODE_NAME_PARAM_NAME,
        default_value="joint_state_broadcaster",
        description="The relative name of this robot's joint state broadcaster node",
    )
    joint_state_broadcaster_node_name = LaunchConfiguration(JOINT_STATE_BROADCASTER_NODE_NAME_PARAM_NAME)

    declared_diff_drive_controller_node_name = DeclareLaunchArgument(
        name=DIFF_DRIVE_CONTROLLER_NODE_NAME_PARAM_NAME,
        default_value="diff_drive_controller",
        description="The relative name of this robot's diff drive controller node",
    )
    diff_drive_controller_node_name = LaunchConfiguration(DIFF_DRIVE_CONTROLLER_NODE_NAME_PARAM_NAME)

    robot_node_name = "robot_node_" + node_id

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot_node_name + "/" + joint_state_broadcaster_node_name,
            "-c",
            controller_manager_node_name
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ron_description"), "urdf", "robot_node.urdf.xacro"]
            ),
            " ",
            ROBOT_NODE_NAME_PARAM_NAME,
            ":=",
            robot_node_name,
        ])}],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot_node_name + "/" + diff_drive_controller_node_name,
            "-c",
            controller_manager_node_name
        ],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

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
    )
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription([
        declared_node_id,
        declared_controller_manager_node_name,
        declared_joint_state_broadcaster_node_name,
        declared_diff_drive_controller_node_name,
        joint_state_broadcaster_spawner,
        robot_state_pub_node,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ])
