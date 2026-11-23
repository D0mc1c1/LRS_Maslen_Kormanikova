# Bring-up dvoch dronov (§1): 2x MAVROS a bridge kamier.
#
# Predpoklad: bezi gz sim so svetom fei_lrs_precision_landing.world a oba SITL
# (ros2 run lrs_bringup sitl_2x.sh).
#
#   ros2 launch lrs_bringup bringup.launch.py
#   ros2 launch lrs_bringup bringup.launch.py bridge:=false   # len MAVROS
#   ros2 launch lrs_bringup bringup.launch.py truth:=true     # + skutocne polohy roverov (len overenie §3)
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
    share = get_package_share_directory('lrs_bringup')
    bridge_cfg = os.path.join(share, 'config', 'bridge.yaml')
    truth_cfg = os.path.join(share, 'config', 'bridge_truth.yaml')

    bridge_arg = DeclareLaunchArgument(
        'bridge', default_value='true', description='spustit ros_gz_bridge kamier')

    truth_arg = DeclareLaunchArgument(
        'truth', default_value='false',
        description='bridge skutocnej polohy roverov z Gazeba, LEN na overenie (verify_pose.py)')

    actions = [bridge_arg, truth_arg]
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
    # samostatny bridge, aby bolo jasne, ze misia skutocnu polohu nepouziva
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='truth_bridge',
        output='screen',
        parameters=[{'config_file': truth_cfg}],
        condition=IfCondition(LaunchConfiguration('truth')),
    ))
    return LaunchDescription(actions)
