from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_rt_indy7',
            executable='rt_node',
            name='rt_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rqt'],
            output='screen'
        ),
    ])

