from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([FindPackageShare('maze_simulation'), 'worlds', 'maze_0.world'])
        }.items()
    )

    robot_controller = Node(
        package='maze_simulation',
        executable='maze_controller',
        name='maze_controller',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_controller
    ])
