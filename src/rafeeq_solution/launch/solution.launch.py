"""Rafeeq Library Robot - competition entry point.

Evaluation command::

    ros2 launch rafeeq_solution solution.launch.py \
        shelf_column_number:=2 book_colour:=red

Brings up the four solution nodes. The simulation itself must already be
running (``ros2 launch erc_bringup simulation.launch.py``); this launch file
deliberately does not start it, so the two can be launched separately on
camera as the rules require.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    column = LaunchConfiguration('shelf_column_number')
    colour = LaunchConfiguration('book_colour')
    arm_side = LaunchConfiguration('arm_side')
    image_dir = LaunchConfiguration('image_dir')
    row_one_is_top = LaunchConfiguration('row_one_is_top')
    auto_start = LaunchConfiguration('auto_start')
    use_sim_time = LaunchConfiguration('use_sim_time')

    arm_poses = os.path.join(
        get_package_share_directory('rafeeq_solution'),
        'config', 'arm_poses.yaml')

    common = {'use_sim_time': use_sim_time}

    perception = Node(
        package='rafeeq_solution',
        executable='perception_node',
        name='rafeeq_perception',
        output='screen',
        emulate_tty=True,
        parameters=[common, {
            'shelf_column_number': column,
            'book_colour': colour,
            'image_dir': image_dir,
            'row_one_is_top': row_one_is_top,
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
            'arm_side': arm_side,
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
            'shelf_column_number': column,
            'book_colour': colour,
            'auto_start': auto_start,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'shelf_column_number',
            description='Shelf column number to retrieve from, 1-5.'),
        DeclareLaunchArgument(
            'book_colour',
            description='Target book colour: red, green, blue or yellow.'),
        DeclareLaunchArgument(
            'arm_side', default_value='right',
            description='Which arm to grasp with: left or right.'),
        DeclareLaunchArgument(
            'image_dir', default_value='/opt/erc_ws/src/erc_images',
            description='Where annotated images are written. This path is '
                        'inside the mounted workspace so the files survive '
                        'the container; /erc_images is symlinked to it.'),
        DeclareLaunchArgument(
            'row_one_is_top', default_value='true',
            description='True if shelf row 1 is the topmost book row.'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='Start the mission immediately. Set false to arm it '
                        'and trigger with /rafeeq/mission/start.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Follow the Gazebo clock. Leave true.'),

        LogInfo(msg=['[rafeeq] target column ', column,
                     ', book colour ', colour,
                     ', grasping with the ', arm_side, ' arm']),

        perception,
        navigation,
        manipulation,
        mission,
    ])
