# Misia find-and-follow: pre kazdy dron N v namespace drone_N detektor a follow_mission.
#
# Predpoklad: bezi Gazebo, sitl_2x.sh a bringup.launch.py (check_bringup.sh = PASS).
# Spustat, ked drony stoja na zemi.
#
#   ros2 launch lrs_bringup follow.launch.py                       # oba drony, skutocny detektor
#   ros2 launch lrs_bringup follow.launch.py drones:=1             # len drone_1
#   ros2 launch lrs_bringup follow.launch.py drones:=1 fake:=true  # fake_target namiesto detektora
#   ros2 launch lrs_bringup follow.launch.py slow:=true            # pomala simulacia (RTF < 1)
#
# sysid drona = marker_id jeho roveru = N. drone_2 hlada vo vacsej vyske (12 m), aby sa
# vyhladavacie vzory oboch dronov neprekryvali vo vyske (plan 2.6).
#
# Abort: ros2 service call /drone_N/follow_mission/abort std_srvs/srv/Trigger
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

SEARCH_ALT = {1: 10.0, 2: 12.0}


def _truthy(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _nodes(context):
    follow_cfg = os.path.join(
        get_package_share_directory('lrs_control'), 'config', 'follow.yaml')
    vision_cfg = os.path.join(
        get_package_share_directory('lrs_vision'), 'config', 'vision.yaml')

    drones_arg = context.launch_configurations['drones']
    fake = _truthy(context.launch_configurations['fake'])
    slow = _truthy(context.launch_configurations['slow'])

    drones = []
    for tok in drones_arg.replace(' ', '').split(','):
        if tok not in ('1', '2'):
            raise RuntimeError(f"drones: povolene su 1 a 2, nie '{tok}'")
        drones.append(int(tok))

    follow_over = {}
    vision_over = {}
    if slow:
        # plan 7.5 a 7.7: pomala simulacia sposobuje falosny abort a stare polohy
        follow_over = {'pose_timeout': 2.0, 'state_timeout': 8.0}
        vision_over = {'pose_max_age': 1.0}

    nodes = []
    for n in drones:
        ns = f'drone_{n}'
        nodes.append(Node(
            package='lrs_control', executable='follow_mission', name='follow_mission',
            namespace=ns, output='screen',
            parameters=[follow_cfg, {'sysid': n, 'search_alt': SEARCH_ALT[n]}, follow_over],
        ))
        if fake:
            # fake_target si pri starte zapamata zem a polohu drona -> spustat na zemi
            nodes.append(Node(
                package='lrs_control', executable='fake_target', name='fake_target',
                namespace=ns, output='screen',
            ))
        else:
            nodes.append(Node(
                package='lrs_vision', executable='aruco_detector', name='aruco_detector',
                namespace=ns, output='screen',
                parameters=[vision_cfg, {'marker_id': n}, vision_over],
            ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drones', default_value='1,2',
                              description='ktore drony: 1, 2 alebo 1,2'),
        DeclareLaunchArgument('fake', default_value='false',
                              description='fake_target namiesto aruco_detector'),
        DeclareLaunchArgument('slow', default_value='false',
                              description='dlhsie timeouty pre pomalu simulaciu'),
        OpaqueFunction(function=_nodes),
    ])
