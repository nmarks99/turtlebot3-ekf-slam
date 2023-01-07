import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals

PACKAGE_NAME = "nuturtle_description"

def generate_launch_description():
    
    # Get parameters from turtle.yaml file 
    #  params = PathJoinSubstitution([
        #  FindPackageShare('nuturtle_description'),
        #  'config/turtle.yaml'
    #  ])

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
    
        # loads turtlebot3_burger.urdf.xacro into a robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description" : Command([
                    TextSubstitution(text="xacro "),
                    PathJoinSubstitution([
                        FindPackageShare(PACKAGE_NAME),
                        "urdf/turtlebot3_burger.urdf.xacro"
                    ])
                ])
                }
            ]
        ),

        # Run joint_state_publisher if use_jsp="true"
        Node (
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition = LaunchConfigurationEquals("use_jsp","true")
        ),

        # start rviz
        Node (
            package="rviz2",
            executable="rviz2",
            condition = LaunchConfigurationEquals("use_rviz", "true")
            #  arguments=[
                #  '-d',
                #  PathJoinSubstitution([
                    #  FindPackageShare(package_name),
                    #  "config/ddrive_urdf.rviz"
                #  ])
            #  ]
        )

    ])


