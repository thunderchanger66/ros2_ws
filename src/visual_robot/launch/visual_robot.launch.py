from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('visual_robot'),
        'urdf',
        'ahpu_robot.urdf'
    )

    rviz2_config = os.path.join(get_package_share_directory('visual_robot'),'rviz','lslidar.rviz')

    return LaunchDescription([
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': open(urdf_file).read()},
        #                 {'publish_frequency': 22.0}  # 设置为10Hz，可根据需要调整
        #                 ]
        # ),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': open(urdf_file).read()},
        #                  {'rate': 22.0}  # 🔺设置 joint_states 发布频率
        #                 ]
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz2_config],
            output='screen'
        )
    ])