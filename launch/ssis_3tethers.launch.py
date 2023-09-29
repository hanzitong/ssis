

from launch import LaunchDescription
from launch_ros.actions import Node

""" winch position """
L = 50.
winch1_pos = [0., 0., 0.]       # x,y,z
winch2_pos = [L, 0., 0.]      # x,y,z   atteru???
winch3_pos = [L/2., L, 0.]      # x,y,z


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pub_winch1_frame',
            arguments = [
                '--x', str(winch1_pos[0]), '--y', str(winch1_pos[1]), '--z', str(winch1_pos[2]),
                '--yaw', '0','--pitch', '0', '--roll', '0',
                '--frame-id', 'world', '--child-frame-id', 'winch1'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pub_winch2_frame'
            arguments = [
                '--x', str(winch2_pos[0]), '--y', str(winch2_pos[1]), '--z', str(winch2_pos[2]),
                '--yaw', '0','--pitch', '0', '--roll', '0',
                '--frame-id', 'world', '--child-frame-id', 'winch2'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pub_winch3_frame',
            arguments = [
                '--x', str(winch3_pos[0]), '--y', str(winch3_pos[1]), '--z', str(winch3_pos[2]),
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'world', '--child-frame-id', 'winch3'
            ]
        ),

        Node(
            package='ssis',
            executable='uav_position_publisher',
            name='pub_uav_position',
        ),
        Node(
            package='ssis',
            executable='uav_frame_publisher_3tethers',
            name='pub_uav_frame_3tethers',
        ),

        Node(
            package='ssis',
            executable='min_tuav_tethers',
            name='min_tuav_tethers',
        ),
        Node(
            package='ssis',
            executable='visualization_node',
            name='visualization_node',
        ),
        # Node(
        #     package='ssis',
        #     executable='get_result_node',
        #     name='get_result_node',
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
    ])

