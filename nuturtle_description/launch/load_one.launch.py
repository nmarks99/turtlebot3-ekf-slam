from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

PACKAGE_NAME = "nuturtle_description"


def generate_launch_description():

    return LaunchDescription([

        # Argument for whether or not to use the joint state publisher
        DeclareLaunchArgument(
            "use_jsp",
            default_value="true",
            description="Choose whether to use the joint state publisher"
        ),

        # Argument for whether or not to use rviz
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Choose whether to use rviz"
        ),

        # Argument to set the color of the robot which is passed to urdf
        DeclareLaunchArgument(
            "color",
            default_value="purple",
            choices=["purple", "red", "blue", "green"],
            description="Set the color of the robot"
        ),

        # Defines the TF prefix to be "color/"
        SetLaunchConfiguration(
            "tf_prefix",
            [LaunchConfiguration("color"), TextSubstitution(text="/")]
        ),

        # Constructs the name of the rviz config file based on color
        SetLaunchConfiguration(
            "rviz_config",
            [
                TextSubstitution(text="config/basic_"),
                LaunchConfiguration("color"),
                TextSubstitution(text=".rviz")
            ]
        ),

        # Constructs the name of the fixed frame based on color
        SetLaunchConfiguration(
            "fixed_frame",
            [
                LaunchConfiguration("color"),
                TextSubstitution(text="/base_footprint")
            ]
        ),

        # Loads turtlebot3_burger.urdf.xacro into a robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": Command([
                    TextSubstitution(text="xacro "),
                    PathJoinSubstitution([
                        FindPackageShare(PACKAGE_NAME),
                        "urdf/turtlebot3_burger.urdf.xacro "
                    ]),
                    TextSubstitution(text="color:="),
                    LaunchConfiguration("color")
                ])
                },
                {"frame_prefix": LaunchConfiguration("tf_prefix")}
            ],
            namespace=LaunchConfiguration("color")
        ),

        # Run joint_state_publisher if use_jsp="true"
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=LaunchConfigurationEquals("use_jsp", "true"),
            namespace=LaunchConfiguration("color")
        ),

        # Start rviz with appropriate config file and fixed frame
        Node(
            package="rviz2",
            executable="rviz2",
            condition=LaunchConfigurationEquals("use_rviz", "true"),
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare(PACKAGE_NAME),
                    LaunchConfiguration("rviz_config")
                ]),
                '--fixed-frame', LaunchConfiguration("fixed_frame")
            ],
            namespace=LaunchConfiguration("color")
        )

    ])
