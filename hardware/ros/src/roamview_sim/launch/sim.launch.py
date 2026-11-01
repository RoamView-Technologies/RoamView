"""RoamView in Gazebo: world + robot + ROS 2 bridge + robot_state_publisher (+ RViz, + cliff guard).

    ros2 launch roamview_sim sim.launch.py                                   # two-room test world
    ros2 launch roamview_sim sim.launch.py world:=warehouse_club             # 130 x 100 m warehouse club
    ros2 launch roamview_sim sim.launch.py world:=warehouse_club lighting:=night people:=20 clutter:=high seed:=4
    ros2 launch roamview_sim sim.launch.py world:=warehouse_club spawn:=receiving guard:=true
    ros2 launch roamview_sim sim.launch.py x:=2.9 y:=-1.0 yaw:=-1.5708 guard:=true   # test world: facing the stairwell

Arguments are listed with:  ros2 launch roamview_sim sim.launch.py --show-args
The warehouse club world is written at launch time by club_world.py (into ~/.ros/roamview_sim/) from your
lighting / people / clutter / hazards / seed settings; the heavy meshes are shared and never rebuilt.
On WSL the Gazebo window draws with ogre on the GPU and the robot's sensors render with ogre2 in software (see
is_wsl() for the measurements behind that); render_engine and sensor_rendering override it.
"""
import importlib.util
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CLUB_SPAWNS = {'dock': (1.6, 92.0, 0.0), 'entrance': (115.0, 6.0, 1.5708), 'receiving': (119.0, 90.0, 1.5708)}
TEST_SPAWN = (-2.0, 0.0, 0.0)


def is_wsl():
    """True under WSL2, where the GPU is reached through Mesa's D3D12 driver. Measured there (Gazebo Jetty, RTX 5060):
      ogre2 on the GPU leaks memory: about 150 MB/s for the Gazebo window in the warehouse club, 20 MB/s for the
            robot's sensors, until WSL runs out of memory and freezes;
      ogre  on the GPU is stable and draws the window and the camera correctly, but its LiDAR and cliff sensors
            return their minimum range on every beam;
      both engines in software (Mesa llvmpipe) are correct; ogre2 sensors run at real time on about 3 CPU cores.
    So on WSL the window is drawn by ogre on the GPU and the sensors by ogre2 in software."""
    try:
        with open('/proc/sys/kernel/osrelease') as f:
            return 'microsoft' in f.read().lower()
    except OSError:
        return False


def _club_world_module():
    path = os.path.join(get_package_prefix('roamview_sim'), 'lib', 'roamview_sim', 'club_world.py')
    spec = importlib.util.spec_from_file_location('club_world', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _on(v):
    return str(v).strip().lower() in ('1', 'true', 'on', 'yes')


def _setup(context):
    pkg = get_package_share_directory('roamview_sim')
    val = lambda name: LaunchConfiguration(name).perform(context)
    world = val('world')
    actions = []
    wsl, gui = is_wsl(), _on(val('gui'))
    engine = val('render_engine')                     # draws the Gazebo window
    if engine == 'auto':
        engine = 'ogre' if wsl else 'ogre2'
    if engine not in ('ogre', 'ogre2'):
        raise RuntimeError('render_engine must be auto, ogre or ogre2')
    sensors = val('sensor_rendering')                 # the robot's camera, LiDAR and cliff sensors (always ogre2)
    if sensors == 'auto':
        sensors = 'software' if wsl else 'gpu'
    if sensors not in ('gpu', 'software'):
        raise RuntimeError('sensor_rendering must be auto, gpu or software')
    if wsl and (sensors == 'gpu' or (gui and engine == 'ogre2')):
        actions.append(LogInfo(msg='WARNING: ogre2 on the GPU leaks memory under WSL (D3D12 driver) until WSL freezes; '
                                   'watch the memory, or drop render_engine / sensor_rendering to use the defaults'))
    elif wsl:
        actions.append(LogInfo(msg='WSL: the Gazebo window draws with ogre on your GPU; the robot sensors render with '
                                   'ogre2 in software (about 3 CPU cores) because ogre2 on the D3D12 driver leaks memory'))
    cache = os.path.join(os.environ.get('ROS_HOME', os.path.expanduser('~/.ros')), 'roamview_sim')
    os.makedirs(cache, exist_ok=True)
    if world in ('warehouse_club', 'club'):
        import json
        club = _club_world_module()
        with open(os.path.join(pkg, 'models', 'warehouse_club', 'manifest.json')) as f:
            manifest = json.load(f)
        text, summary = club.world_sdf(manifest, lighting=val('lighting'), shadows=_on(val('shadows')),
                                       people=val('people'), clutter=val('clutter'), seed=int(val('seed')),
                                       hazards=_on(val('hazards')), rtf=float(val('rtf')))
        tag = '_'.join(f'{k}-{v}' for k, v in summary.items())
        world_file = os.path.join(cache, f'warehouse_club_{tag}.sdf')
        with open(world_file, 'w') as f:
            f.write(text)
        actions.append(LogInfo(msg=f'warehouse club world: {summary} -> {world_file}'))
        preset = CLUB_SPAWNS.get(val('spawn'))
        if preset is None:
            raise RuntimeError(f"spawn must be one of {', '.join(CLUB_SPAWNS)}")
    elif world in ('roamview_test', 'test', ''):
        world_file, preset = os.path.join(pkg, 'worlds', 'roamview_test.sdf'), TEST_SPAWN
    else:
        world_file, preset = os.path.expanduser(world), TEST_SPAWN
    x, y, yaw = (val(k) or str(p) for k, p in zip(('x', 'y', 'yaw'), preset))

    # odometry: the EKF (wheel speed + IMU yaw rate) publishes odom -> base_footprint, so the DiffDrive's own
    # wheel-only transform is left out of the bridge
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')
    use_ekf = _on(val('ekf'))
    if use_ekf:
        try:
            get_package_share_directory('robot_localization')
        except Exception:
            use_ekf = False
            actions.append(LogInfo(msg='robot_localization is not installed: using wheel odometry only '
                                       '(sudo apt install ros-$ROS_DISTRO-robot-localization)'))
    if use_ekf:
        import tempfile
        import yaml
        with open(bridge_cfg) as f:
            entries = [e for e in yaml.safe_load(f) if e.get('ros_topic_name') != '/tf']
        tmp = tempfile.NamedTemporaryFile('w', prefix='roamview_bridge_', suffix='.yaml', delete=False)
        yaml.safe_dump(entries, tmp, sort_keys=False)
        tmp.close()
        bridge_cfg = tmp.name
        actions.append(Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
            parameters=[os.path.join(pkg, 'config', 'ekf.yaml'), {'use_sim_time': True}]))
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}]))

    # Gazebo: one process (server + window) when both render on the GPU; with software sensors the server runs on
    # its own with Mesa's llvmpipe and the window joins it as a second process on the GPU
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    def gz(gz_args, env=None):
        include = IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                           launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())
        sets = [SetEnvironmentVariable(k, v) for k, v in (env or {}).items()]
        return GroupAction(sets + [include], scoped=True, forwarding=True)

    server = f'{world_file} -s -r -v 2 --headless-rendering'
    if sensors == 'software':
        actions.append(gz(server, {'LIBGL_ALWAYS_SOFTWARE': '1', 'GALLIUM_DRIVER': 'llvmpipe'}))
        if gui:
            actions.append(gz(f'-g -v 2 --render-engine-gui {engine}'))
    else:
        actions.append(gz(f'{world_file} -r -v 2 --render-engine-gui {engine}' if gui else server))
    actions.append(Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'roamview', '-x', x, '-y', y, '-z', '0.002', '-Y', yaw]))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('roamview_sim')
    arg = LaunchConfiguration

    args = [
        DeclareLaunchArgument('world', default_value='roamview_test',
                              description='roamview_test, warehouse_club, or a path to an SDF world file'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument('guard', default_value='false',
                              description='Run cliff_guard: send commands to /cmd_vel_in instead of /cmd_vel'),
        DeclareLaunchArgument('spawn', default_value='dock',
                              description='warehouse_club start: dock (charging dock), entrance, receiving'),
        DeclareLaunchArgument('x', default_value='', description='Spawn x (m); empty = the world default / spawn preset'),
        DeclareLaunchArgument('y', default_value='', description='Spawn y (m)'),
        DeclareLaunchArgument('yaw', default_value='', description='Spawn heading (rad)'),
        DeclareLaunchArgument('lighting', default_value='day', description='warehouse_club: day, bright, dim, night'),
        DeclareLaunchArgument('shadows', default_value='true', description='warehouse_club: shadows (false = faster)'),
        DeclareLaunchArgument('people', default_value='0',
                              description='warehouse_club: number of walking shoppers, 0-40 (true = 8)'),
        DeclareLaunchArgument('clutter', default_value='none',
                              description='warehouse_club: loose items by the shelves: none, low, medium, high or a count'),
        DeclareLaunchArgument('seed', default_value='1', description='warehouse_club: same seed = same people and clutter'),
        DeclareLaunchArgument('hazards', default_value='true',
                              description='warehouse_club: pallet jack, empty pallet, wet-floor sign, abandoned cart'),
        DeclareLaunchArgument('ekf', default_value='true',
                              description='Fuse wheel odometry with the IMU (robot_localization); false = wheels only'),
        DeclareLaunchArgument('rtf', default_value='1.0',
                              description='warehouse_club: real-time factor to aim for (lower it if your PC cannot keep up)'),
        DeclareLaunchArgument('render_engine', default_value='auto',
                              description='Engine of the Gazebo window: auto (ogre on WSL, ogre2 elsewhere), ogre or ogre2'),
        DeclareLaunchArgument('sensor_rendering', default_value='auto',
                              description='Robot camera / LiDAR / cliff sensors (ogre2): auto (software on WSL, GPU '
                                          'elsewhere), gpu or software'),
        DeclareLaunchArgument('camera', default_value='true', description='Simulate the OAK-D Pro W'),
        DeclareLaunchArgument('suspension', default_value='true', description='Sprung drive modules'),
        DeclareLaunchArgument('caster_mode', default_value='swivel', description='swivel or ball'),
        DeclareLaunchArgument('cog_x', default_value='-0.004', description='Robot CoG ahead of the axle (m)'),
        DeclareLaunchArgument('cog_z', default_value='0.126', description='Robot CoG height (m)'),
        DeclareLaunchArgument('max_speed', default_value='0.5', description='Speed limit (m/s)'),
        DeclareLaunchArgument('accel', default_value='0.5', description='Acceleration limit (m/s^2)'),
        DeclareLaunchArgument('decel', default_value='0.5', description='Deceleration limit (m/s^2)'),
    ]

    xacro_file = os.path.join(pkg, 'urdf', 'roamview.urdf.xacro')
    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' camera:=', arg('camera'), ' suspension:=', arg('suspension'), ' caster_mode:=', arg('caster_mode'),
        ' cog_x:=', arg('cog_x'), ' cog_z:=', arg('cog_z'), ' max_speed:=', arg('max_speed'),
        ' accel:=', arg('accel'), ' decel:=', arg('decel'),
    ]), value_type=str)

    # meshes and textures of the warehouse club are found through model://warehouse_club/...
    resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models'))

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    # WSLg runs Wayland; RViz's renderer needs X11 (else "Unable to create the rendering window")
    rviz = Node(
        package='rviz2', executable='rviz2', output='log', condition=IfCondition(arg('rviz')),
        arguments=['-d', os.path.join(pkg, 'rviz', 'roamview.rviz')],
        parameters=[{'use_sim_time': True}],
        additional_env={'QT_QPA_PLATFORM': 'xcb'} if is_wsl() else None)

    guard = Node(
        package='roamview_sim', executable='cliff_guard.py', output='screen', condition=IfCondition(arg('guard')),
        parameters=[{'use_sim_time': True}])

    return LaunchDescription(args + [resources, OpaqueFunction(function=_setup), robot_state_publisher, rviz, guard])
