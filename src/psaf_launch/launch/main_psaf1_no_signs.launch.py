"""Start the whole pipeline with all Nodes for the old car."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node


__ucbridge = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_launch'), 'launch', 'ucbridge_old.launch.py')))

__realsense2_camera = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_launch'), 'launch',
                 'realsense2_camera_455.launch.py')
))

__psaf_controller = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_controller'),
                 'launch', 'controller.launch.py')
))

__psaf_lane_detection = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_lane_detection'),
                 'launch', 'lanedetection.launch.py')
))


__psaf_startbox = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_startbox'),
                 'launch', 'startbox.launch.py')
))

__psaf_state_machine = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_state_machine'),
                 'launch', 'statemachine.launch.py')
))

__psaf_trajectory = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_trajectory'),
                 'launch', 'trajectory.launch.py')
))

__psaf_watchdog = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(get_package_share_directory('psaf_watchdog'),
                 'launch', 'watchdog.launch.py')
))


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['Start model car for the Carolo-Cup']),
        __ucbridge,
        __realsense2_camera,
        __psaf_controller,
        __psaf_lane_detection,
        __psaf_startbox,
        __psaf_state_machine,
        __psaf_trajectory,
        __psaf_watchdog
    ])
