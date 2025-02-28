from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_ymbot_d_moveit = '/home/ymzz/ymbot_d_VD_ws/src/ymbot_d_moveit_config'
    pkg_ymbot_d_description = '/home/ymzz/ymbot_d_VD_ws/src/ymbot_d_description'
    
    # 加载参数文件
    ros_controllers_path = os.path.join(pkg_ymbot_d_moveit, 'config', 'moveit_controllers.yaml')
    kinematics_path = os.path.join(pkg_ymbot_d_moveit, 'config', 'kinematics.yaml')
    
    # 加载URDF和SRDF文件
    robot_description_path = os.path.join(pkg_ymbot_d_description, 'urdf', 'ymbot_d_description.urdf')
    robot_description_semantic_path = os.path.join(pkg_ymbot_d_moveit, 'config', 'ymbot_d_description.srdf')

    # 读取文件内容
    with open(robot_description_path, 'r') as f:
        robot_description = f.read()
    with open(robot_description_semantic_path, 'r') as f:
        robot_description_semantic = f.read()

    # 修改参数加载命令，添加节点名称
    load_controllers = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/controller_manager', ros_controllers_path],
        output='screen'
    )

    load_kinematics = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/move_group', kinematics_path],
        output='screen'
    )
    # 获取demo.launch.py文件的路径
    sub_launch_path = os.path.join(
        get_package_share_directory('ymbot_d_moveit_config'),  # 替换为你的包名
        'launch',
        'demo.launch.py'  # 子 launch 文件名
    )
    # 包含子 launch 文件
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sub_launch_path)
    )
    # UDP节点
    udp_node = Node(
        package='udp2joint',
        executable='moveit_test',
        name='moveit_test',
        output='screen'
    )

    # 设置机器人描述参数
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic
        }]
    )

    # 以下是注释掉的节点，如果需要可以取消注释
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('udp2joint'), 'rviz', 'rviz2.rviz')],
        parameters=[{'required': True}]
    )
    manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{
                'robot_description': robot_description,
                # 'arm_id': 'ymbot_d'
            }],
            output='screen'
        )
    # 在launch文件中添加
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "body_controller",
            "neck_controller",
            "left_arm_controller",
            "right_arm_controller",
            "left_arm_position_controller",   # 添加位置控制器
            "right_arm_position_controller",  # 添加位置控制器
        ],
        output='screen'
    )
    # 返回启动描述
    return LaunchDescription([
        # load_controllers,
        # load_kinematics,
        # spawner,
        moveit_launch,
        # robot_state_publisher,
        udp_node
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        # manager,
        # spawner
        # rviz
    ]) 