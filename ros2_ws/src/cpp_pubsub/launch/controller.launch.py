from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # my own launch arguments
    participant_parameter_name = 'part_id'
    trajectory_parameter_name = 'traj_id'
    autonomy_parameter_name = 'auto_id'

    participant = LaunchConfiguration(participant_parameter_name)
    trajectory = LaunchConfiguration(trajectory_parameter_name)
    autonomy = LaunchConfiguration(autonomy_parameter_name)


    return LaunchDescription([

        # my experimental config (using launch arguments)
        DeclareLaunchArgument(
            participant_parameter_name,
            default_value='0',  
            description='Participant ID parameter'),
        DeclareLaunchArgument(
            trajectory_parameter_name,
            default_value='0',  
            description='Trajectory ID parameter'),
        DeclareLaunchArgument(
            autonomy_parameter_name,
            default_value='5',  
            description='Autonomy ID parameter'),


        # real robot controller node [need position_talker to be running]
        Node(
            package='cpp_pubsub',
            executable='real_controller',
            parameters=[
                {participant_parameter_name: participant},
                {trajectory_parameter_name: trajectory},
                {autonomy_parameter_name: autonomy}
            ],
            output='screen',
            emulate_tty=True,
            name='real_controller'
        ),

    ])
