from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # my own launch arguments
    free_drive_parameter_name = 'free_drive'
    mapping_ratio_parameter_name = 'mapping_ratio'
    participant_parameter_name = 'part_id'
    trajectory_parameter_name = 'traj_id'
    autonomy_parameter_name = 'auto_id'

    free_drive = LaunchConfiguration(free_drive_parameter_name)
    mapping_ratio = LaunchConfiguration(mapping_ratio_parameter_name)
    participant = LaunchConfiguration(participant_parameter_name)
    trajectory = LaunchConfiguration(trajectory_parameter_name)
    autonomy = LaunchConfiguration(autonomy_parameter_name)


    return LaunchDescription([

        # my experimental config (using launch arguments)
        DeclareLaunchArgument(
            free_drive_parameter_name,
            default_value='0',  
            description='Free drive parameter'),
        DeclareLaunchArgument(
            mapping_ratio_parameter_name,
            default_value='3.0',  
            description='Mapping ratio parameter'),
        DeclareLaunchArgument(
            participant_parameter_name,
            default_value='0',  
            description='Participant ID parameter'),
        DeclareLaunchArgument(
            autonomy_parameter_name,
            default_value='0',  
            description='Autonomy ID parameter'),
        DeclareLaunchArgument(
            trajectory_parameter_name,
            default_value='0',  
            description='Trajectory ID parameter'),


        # real robot controller node [need position_talker to be running]
        Node(
            package='cpp_pubsub',
            executable='real_controller',
            parameters=[
                {free_drive_parameter_name: free_drive},
                {mapping_ratio_parameter_name: mapping_ratio},
                {participant_parameter_name: participant},
                {trajectory_parameter_name: trajectory},
                {autonomy_parameter_name: autonomy}
            ],
            output='screen',
            emulate_tty=True,
            name='real_controller'
        ),

    ])
