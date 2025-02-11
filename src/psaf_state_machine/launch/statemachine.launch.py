import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

name = 'psaf_state_machine'


def generate_launch_description():
    ld = LaunchDescription()
    state_machine_node = Node(
        package=name,
        executable='basic_cup_state_machine',
        name='state_machine',
        parameters=[os.path.join(get_package_share_directory(name), 'config', name + '.yaml')],
        output='screen'
    )
    ld.add_action(state_machine_node)
    return ld
