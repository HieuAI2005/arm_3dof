from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('simulation')
    urdf_default = PathJoinSubstitution([pkg, 'urdf', 'arm_robot.urdf'])

    urdf_arg   = DeclareLaunchArgument('urdf', default_value=urdf_default, description='URDF file path')
    use_gui    = DeclareLaunchArgument('use_gui', default_value='true',  description='Use joint_state_publisher_gui')
    use_rviz   = DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2')

    urdf_path = LaunchConfiguration('urdf')
    gui       = LaunchConfiguration('use_gui')
    rviz      = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(
        PythonExpression(["open('", urdf_path, "', 'r').read()"]),
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    jsp = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        condition=UnlessCondition(gui), output='screen'
    )

    jsp_gui = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        condition=IfCondition(gui), output='screen'
    )

    rviz2 = Node(
        package='rviz2', executable='rviz2',
        condition=IfCondition(rviz), output='screen'
    )

    return LaunchDescription([urdf_arg, use_gui, use_rviz, rsp, jsp, jsp_gui, rviz2])