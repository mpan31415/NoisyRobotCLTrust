from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    # my own launch arguments
    participant_parameter_name = 'part_id'
    trajectory_parameter_name = 'traj_id'
    autonomy_parameter_name = 'auto_id'

    participant = LaunchConfiguration(participant_parameter_name)
    trajectory = LaunchConfiguration(trajectory_parameter_name)
    autonomy = LaunchConfiguration(autonomy_parameter_name)


    return LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            default_value='172.16.0.2',                 ### originally this line was not here
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='true',                       ### this was originally false
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),

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
            default_value='0',  
            description='Autonomy ID parameter'),


        ### franka_bringup launch ###
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'])]),
            launch_arguments={robot_ip_parameter_name: robot_ip,
                              load_gripper_parameter_name: load_gripper,
                              use_fake_hardware_parameter_name: use_fake_hardware,
                              fake_sensor_commands_parameter_name: fake_sensor_commands,
                              use_rviz_parameter_name: use_rviz
                              }.items(),
        ),


        ############################## THE FOLLOWING ARE MY OWN STUFF ##############################

        # joint trajectory controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            output='screen',
        ),

        # test controller
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['test_controller'],
        #     output='screen',
        # ),

        ### kinect_camera launch ### 
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution(
        #         [FindPackageShare('azure_kinect_ros_driver'), 'launch', 'driver.launch.py'])])
        # ),

        # activate Falcon node [need Falcon to be connected]
        Node(
            package='cpp_pubsub',
            executable='position_talker',
            parameters=[
                {participant_parameter_name: participant},
                {trajectory_parameter_name: trajectory},
                {autonomy_parameter_name: autonomy}
            ],
            output='screen',
            emulate_tty=True,
            name='position_talker'
        ),

        # marker publisher node
        Node(
            package='cpp_pubsub',
            executable='marker_publisher',
            parameters=[
                {participant_parameter_name: participant},
                {trajectory_parameter_name: trajectory},
                {autonomy_parameter_name: autonomy}
            ],
            output='screen',
            emulate_tty=True,
            name='marker_publisher'
        ),

        # trajectory recorder node
        # Node(
        #     package='cpp_pubsub',
        #     executable='traj_recorder.py',
        #     parameters=[
        #         {participant_parameter_name: participant},
        #         {trajectory_parameter_name: trajectory},
        #         {autonomy_parameter_name: autonomy}
        #     ],
        #     output='screen',
        #     emulate_tty=True
        # )

        # publish camera frame [need camera to be connected]
        # Node(
        #     package='cpp_pubsub',
        #     executable='const_br',
        #     name='const_br'
        # ),

    ])
