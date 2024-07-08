from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Define the path to the RViz config file
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('kobuki_description'), 'rviz', 'model.rviz']
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='true',  # Changed to true to launch RViz by default
            description='Run RViz'
        ),

        # Node for odom_tf_broadcaster
        Node(
            package='kobuki_tf',
            executable='kobuki_tf',
            name='odom_tf_broadcaster',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Node for RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
    ])
