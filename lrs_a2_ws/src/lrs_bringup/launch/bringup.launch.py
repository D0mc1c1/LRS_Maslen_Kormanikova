# Bring-up dvoch dronov (§1): 2x MAVROS a bridge kamier.
#
# Predpoklad: bezi gz sim so svetom fei_lrs_precision_landing.world a oba SITL
# (ros2 run lrs_bringup sitl_2x.sh).
#
#   ros2 launch lrs_bringup bringup.launch.py
#   ros2 launch lrs_bringup bringup.launch.py bridge:=false   # len MAVROS
#
# MAVROS bezi v namespace /drone_N a sam si prida uzol "mavros", takze temy su
# /drone_N/mavros/state atd. (tak ich cita follow_mission). S namespace drone_N/mavros
# by boli /drone_N/mavros/mavros/state (overene v Gazebe, MAVROS 2026).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# drone_N: SITL --instance N-1 posiela MAVLink na UDP 14550 + 10*(N-1), sysid = N
DRONES = [
    {'n': 1, 'port': 14550},
    {'n': 2, 'port': 14560},
]


def generate_launch_description():
    bridge_cfg = os.path.join(
        get_package_share_directory('lrs_bringup'), 'config', 'bridge.yaml')

    bridge_arg = DeclareLaunchArgument(
        'bridge', default_value='true', description='spustit ros_gz_bridge kamier')

    actions = [bridge_arg]
    for d in DRONES:
        n = d['n']
        actions.append(Node(
            package='mavros',
            executable='mavros_node',
            namespace=f'drone_{n}',
            output='screen',
            parameters=[{
                'fcu_url': f"udp://0.0.0.0:{d['port']}@",
                'tgt_system': n,
                'tgt_component': 1,
            }],
        ))

    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{'config_file': bridge_cfg}],
        condition=IfCondition(LaunchConfiguration('bridge')),
    ))
    return LaunchDescription(actions)
