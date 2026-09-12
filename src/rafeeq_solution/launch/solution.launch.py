"""Rafeeq Library Robot - competition entry point.

Evaluation command::

    ros2 launch rafeeq_solution solution.launch.py \\
        shelf_column_number:=2 book_colour:=red

Those two arguments are the only ones required.  Everything else has a working
default, the mission starts itself, and no browser, dashboard or second
"go" command is involved.

The simulation itself must already be running::

    ros2 launch erc_bringup simulation.launch.py

This launch file deliberately does not start it, so the two can be launched
separately on camera as the rules require.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _repo_root():
    """Where the evidence directory lives.

    Phase 1 asks for the annotated images in a folder named ``erc_images`` in
    the team's repository, so that is what this looks for - not merely for
    somewhere writable.  It matters because the judges clone this repository
    into a workspace and build it: the launch file then runs from ``install/``,
    several directories away from the checkout that has to receive the images.

    Searched in order:

    1. up from this file, for a directory holding ``erc_images`` - true of a
       plain clone and of a symlink-installed or mounted workspace;
    2. a sibling checkout under ``src/``, which is where a cloned repository
       sits once it has been built and the launch file has moved to
       ``install/``;
    3. the working directory, so a run still writes its evidence somewhere
       rather than not at all.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    roots = []
    for _ in range(8):
        here = os.path.dirname(here)
        if not here or here == os.sep:
            break
        if os.path.isdir(os.path.join(here, 'erc_images')):
            return here
        roots.append(here)

    # The checkout, from a workspace root.  Prefer the repository that actually
    # holds this package over any other clone that happens to sit beside it.
    for base in roots + [os.getcwd()]:
        src = os.path.join(base, 'src')
        if not os.path.isdir(src):
            continue
        try:
            entries = sorted(os.listdir(src))
        except OSError:
            continue
        for name in entries:
            checkout = os.path.join(src, name)
            if (os.path.isdir(os.path.join(checkout, 'erc_images'))
                    and os.path.isdir(
                        os.path.join(checkout, 'src', 'rafeeq_solution'))):
                return checkout
        for name in entries:
            checkout = os.path.join(src, name)
            if os.path.isdir(os.path.join(checkout, 'erc_images')):
                return checkout
    return os.getcwd()


def generate_launch_description():
    column = LaunchConfiguration('shelf_column_number')
    colour = LaunchConfiguration('book_colour')
    arm_side = LaunchConfiguration('arm_side')
    image_dir = LaunchConfiguration('image_dir')
    results_dir = LaunchConfiguration('results_dir')
    row_one_is_top = LaunchConfiguration('row_one_is_top')
    auto_start = LaunchConfiguration('auto_start')
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_id = LaunchConfiguration('run_id')

    arm_poses = os.path.join(
        get_package_share_directory('rafeeq_solution'),
        'config', 'arm_poses.yaml')

    root = _repo_root()
    common = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    # The two competition inputs are typed explicitly.  Launch substitutions
    # arrive as strings, and a node that declared an int parameter would get a
    # type error on the judges' machine rather than on ours.
    column_param = ParameterValue(column, value_type=int)
    colour_param = ParameterValue(colour, value_type=str)

    perception = Node(
        package='rafeeq_solution',
        executable='perception_node',
        name='rafeeq_perception',
        output='screen',
        emulate_tty=True,
        parameters=[common, {
            'shelf_column_number': column_param,
            'book_colour': colour_param,
            'image_dir': ParameterValue(image_dir, value_type=str),
            'row_one_is_top': ParameterValue(row_one_is_top, value_type=bool),
        }],
    )

    navigation = Node(
        package='rafeeq_solution',
        executable='navigation_node',
        name='rafeeq_navigation',
        output='screen',
        emulate_tty=True,
        parameters=[common],
    )

    manipulation = Node(
        package='rafeeq_solution',
        executable='manipulation_node',
        name='rafeeq_manipulation',
        output='screen',
        emulate_tty=True,
        parameters=[common, {
            'arm_side': ParameterValue(arm_side, value_type=str),
            'config_file': arm_poses,
        }],
    )

    mission = Node(
        package='rafeeq_solution',
        executable='mission_node',
        name='rafeeq_mission',
        output='screen',
        emulate_tty=True,
        parameters=[common, {
            'shelf_column_number': column_param,
            'book_colour': colour_param,
            'auto_start': ParameterValue(auto_start, value_type=bool),
            'results_dir': ParameterValue(results_dir, value_type=str),
            'run_id': ParameterValue(run_id, value_type=str),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'shelf_column_number',
            description='Shelf column number to retrieve from: 1-5. '
                        'Required.'),
        DeclareLaunchArgument(
            'book_colour',
            description='Target book colour: red, green, blue or yellow. '
                        'Required.'),
        DeclareLaunchArgument(
            'arm_side', default_value='right',
            description='Which single arm grasps with: left or right. The '
                        'other arm is tucked out of the way.'),
        DeclareLaunchArgument(
            'image_dir', default_value=os.path.join(root, 'erc_images'),
            description='Where annotated evidence images are written. '
                        'Defaults to the erc_images directory of this '
                        'workspace.'),
        DeclareLaunchArgument(
            'results_dir', default_value=os.path.join(root, 'results'),
            description='Where the machine-readable trial record is written.'),
        DeclareLaunchArgument(
            'row_one_is_top', default_value='true',
            description='True if shelf row 1 is the topmost book row.'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='Start the mission as soon as the robot is ready. '
                        'Set false to arm it and trigger with '
                        '/rafeeq/mission/start.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Follow the Gazebo clock. Leave true.'),
        DeclareLaunchArgument(
            'run_id', default_value='',
            description='Identifier for the trial record. Generated if empty.'),

        LogInfo(msg=['[rafeeq] Team Kanban | target column ', column,
                     ', ', colour, ' book | grasping with the ', arm_side,
                     ' arm | evidence -> ', image_dir]),

        perception,
        navigation,
        manipulation,
        mission,
    ])
