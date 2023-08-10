from launch import LaunchDescription
from launch_ros.actions import Node

""" ver2 position """
L = 50.
winch1_pos = [0., 0., 0.]       # x,y,z
winch2_pos = [L, 0., 0.]      # x,y,z
winch3_pos = [L/2., L, 0.]      # x,y,z
# winch4_pos = [0., L, 0.]      # x,y,z
# winch5_pos = [L, L, 0.]      # x,y,z


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
            name='pub_winch2_frame',
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
            executable='tension_visualization_node',
            name='tension_visualization_node',
        ),
        Node(
            package='ssis',
            executable='tether_visualization_node',
            name='tether_visualization_node',
        ),
        Node(
            package='ssis',
            executable='get_result_node',
            name='get_result_node',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
    ])





"""
        Node(
            package='parallel_wire',
            executable='uav1_frame_publisher',
            name='pub_uav1_frame'
        ),
        Node(
            package='parallel_wire',
            executable='uav2_frame_publisher',
            name='pub_uav2_frame'
        ),
        Node(
            package='parallel_wire',
            executable='uav3_frame_publisher',
            name='pub_uav3_frame'
        ),
        Node(
            package='parallel_wire',
            executable='make_figure_node',
            name='make_figure_node',
        ),





        Node(
            package='parallel_wire',
            executable='min_tuav_1tether',
            name='min_tuav_1tether',
            #arguments = [50., [3., 3.], [50., 50.], [0.045, 0.045]]   # [L:float, s_margin:list, v_margin:list, rho:list]
        ),
        Node(
            package='parallel_wire',
            executable='min_tuav_2tethers',
            name='min_tuav_2tethers',
        ),
        Node(
            package='parallel_wire',
            executable='min_tuav_3tethers',
            name='min_tuav_3tethers',
        ),
        Node(
            package='parallel_wire',
            executable='pub_pose_test',
            name='pub_pose_for_test'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pub_winch4_frame',
            arguments = [
                '--x', str(winch3_pos[0]), '--y', str(winch3_pos[1]), '--z', str(winch3_pos[2]),
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'winch_world', '--child-frame-id', 'winch3'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pub_winch5_frame',
            arguments = [
                '--x', str(winch3_pos[0]), '--y', str(winch3_pos[1]), '--z', str(winch3_pos[2]),
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'winch_world', '--child-frame-id', 'winch3'
            ]
        ),



"""

