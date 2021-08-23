#!/usr/bin/env python

import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import sys


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "fog_bumper"
    pkg_share_path = get_package_share_directory(package_name=pkg_name)

    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

    fcu_horizontal_frame = DRONE_DEVICE_ID + "/fcu"

    ld.add_action(launch.actions.DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))
    dbg_sub = None
    if sys.stdout.isatty():
        dbg_sub = launch.substitutions.PythonExpression(['"" if "false" == "', launch.substitutions.LaunchConfiguration("debug"), '" else "debug_ros2launch ' + os.ttyname(sys.stdout.fileno()) + '"'])

    namespace = DRONE_DEVICE_ID

    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_fog_bumper',
        package='rclcpp_components',
        executable='component_container_mt',
        # executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                namespace=namespace,
                name='bumper',
                package=pkg_name,
                plugin='fog_bumper::Bumper',
                remappings=[
                    # subscribers
                    ("lidar1d_down_in", "/" + DRONE_DEVICE_ID + "/garmin/range"),
                    ("lidar2d_in", "/" + DRONE_DEVICE_ID + "/rplidar/scan"),
                    # publishers
                    ("obstacle_sectors_out", "/" + DRONE_DEVICE_ID + "/obstacle_sectors"),
                ],
                parameters=[
                    pkg_share_path + '/config/param.yaml',
                    {"frame_id": fcu_horizontal_frame},
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))
    return ld
