"""Nav2 for RoamView in the warehouse club: map + AMCL + planner/controller + keepout zones (+ RViz).

    # terminal 1: the simulation
    ros2 launch roamview_sim sim.launch.py world:=warehouse_club rviz:=false
    # terminal 2: navigation (starts localised at the same spawn preset)
    ros2 launch roamview_sim nav.launch.py
    # terminal 3: patrol the store
    ros2 run roamview_sim patrol.py --ros-args -p use_sim_time:=true

Keepout zones (maps/warehouse_club_keepout.pgm) are no-go areas Nav2 will not plan or drive into: the 1.22 m
dock edges in receiving, the two doorways, and everything outside the building. Turn them off with keepout:=false
to see why they are there.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SPAWNS = {'dock': (1.6, 92.0, 0.0), 'entrance': (115.0, 6.0, 1.5708), 'receiving': (119.0, 90.0, 1.5708)}


def _merge(base, over):
    """Recursive dict merge: values in `over` win; lists are replaced, not appended."""
    out = dict(base)
    for k, v in over.items():
        out[k] = _merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def _setup(context):
    pkg = get_package_share_directory('roamview_sim')
    val = lambda n: LaunchConfiguration(n).perform(context)
    with open(val('params_file')) as f:
        ours = yaml.safe_load(f)
    stamped = ours.pop('stamped_cmd_vel_nodes', [])
    if val('merge_defaults').lower() in ('1', 'true', 'on', 'yes'):
        with open(os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')) as f:
            params = _merge(yaml.safe_load(f), ours)
    else:
        params = ours
    for node in stamped:                       # Jazzy needs this for TwistStamped; later releases ignore it
        if node in params and 'ros__parameters' in params[node]:
            params[node]['ros__parameters']['enable_stamped_cmd_vel'] = True
    x, y, yaw = SPAWNS[val('spawn')]
    x, y, yaw = (float(val(k)) if val(k) else d for k, d in zip(('x', 'y', 'yaw'), (x, y, yaw)))
    amcl = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}
    use_keepout = val('keepout').lower() in ('1', 'true', 'on', 'yes')
    for cm in ('global_costmap', 'local_costmap'):
        p = params.get(cm, {}).get(cm, {}).get('ros__parameters', {})
        if not use_keepout:
            p.pop('filters', None)
    tmp = tempfile.NamedTemporaryFile('w', prefix='nav2_warehouse_club_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, tmp, sort_keys=False)
    tmp.close()

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                                                   'bringup_launch.py')),
        launch_arguments={'map': val('map'), 'params_file': tmp.name, 'use_sim_time': 'true',
                          'autostart': 'true'}.items())]
    if use_keepout:
        actions += [
            Node(package='nav2_map_server', executable='map_server', name='filter_mask_server', output='screen',
                 parameters=[{'use_sim_time': True, 'frame_id': 'map', 'topic_name': '/keepout_filter_mask',
                              'yaml_filename': val('keepout_mask')}]),
            Node(package='nav2_map_server', executable='costmap_filter_info_server', name='costmap_filter_info_server',
                 output='screen',
                 parameters=[{'use_sim_time': True, 'type': 0, 'filter_info_topic': '/costmap_filter_info',
                              'mask_topic': '/keepout_filter_mask', 'base': 0.0, 'multiplier': 1.0}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_filters',
                 output='screen',
                 parameters=[{'use_sim_time': True, 'autostart': True,
                              'node_names': ['filter_mask_server', 'costmap_filter_info_server']}]),
        ]
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('roamview_sim')
    args = [
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'warehouse_club.yaml'),
                              description='Occupancy map YAML'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_warehouse_club.yaml'),
                              description='Nav2 parameters (our changes, merged over your Nav2 defaults)'),
        DeclareLaunchArgument('merge_defaults', default_value='true',
                              description='Merge params_file over nav2_bringup/params/nav2_params.yaml (false = use it as is)'),
        DeclareLaunchArgument('spawn', default_value='dock',
                              description='Where the robot starts (sets the AMCL initial pose): dock, entrance, receiving'),
        DeclareLaunchArgument('x', default_value='', description='Initial pose x (m); empty = the spawn preset'),
        DeclareLaunchArgument('y', default_value='', description='Initial pose y (m)'),
        DeclareLaunchArgument('yaw', default_value='', description='Initial heading (rad)'),
        DeclareLaunchArgument('keepout', default_value='true', description='Use the keepout zones'),
        DeclareLaunchArgument('keepout_mask', default_value=os.path.join(pkg, 'maps', 'warehouse_club_keepout.yaml'),
                              description='Keepout mask YAML'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz with the Nav2 tools'),
    ]
    try:                                         # WSLg runs Wayland; RViz's renderer needs X11
        wsl = 'microsoft' in open('/proc/sys/kernel/osrelease').read().lower()
    except OSError:
        wsl = False
    rviz = Node(
        package='rviz2', executable='rviz2', output='log', condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        additional_env={'QT_QPA_PLATFORM': 'xcb'} if wsl else None)
    return LaunchDescription(args + [OpaqueFunction(function=_setup), rviz])
