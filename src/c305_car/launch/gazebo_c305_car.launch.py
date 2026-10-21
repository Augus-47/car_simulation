import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_path = get_package_share_directory('c305_car')
    xacro_path = os.path.join(pkg_path, 'urdf', 'c305_car.urdf.xacro')
    rviz_path = os.path.join(pkg_path, 'config', 'display_c305_car.rviz')
    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    entity = LaunchConfiguration('entity')
    spawn_z = LaunchConfiguration('z')
    steering_bridge_enabled = LaunchConfiguration('steering_bridge')

    # xacro 展开为 URDF 文本
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)

    # 发布机器人 TF（Gazebo 中关节状态由 joint_state_broadcaster 提供）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # 把本包的 share 目录加入 GAZEBO_RESOURCE_PATH，Gazebo 才能找到
    # media/materials/scripts/c305_car.material（自定义半透明材质）。
    # 必须保留系统原有路径，否则 Gazebo 找不到内置材质。
    gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        os.pathsep.join([
            pkg_path,
            os.environ.get('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),
        ]),
    )

    # 启动 Gazebo（gzserver.launch.py 默认会加载 libgazebo_ros_init.so
    # 与 libgazebo_ros_factory.so，spawn_entity.py 依赖后者）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': 'true',
        }.items(),
    )

    # 把机器人模型生成到 Gazebo 中。z 默认 0.02m，让轮子略高于地面，避免初始穿透。
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', entity,
            '-z', spawn_z,
        ],
        output='screen',
    )

    # 控制器：关节状态广播 + 后轮差速驱动 + 前轮转向位置控制
    # 注意 spawner 必须等模型生成、controller_manager 起来之后再调用，
    # 所以挂在 spawn_entity 的退出事件上（spawn_entity 生成完模型后自动退出）。
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )
    steering_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'steering_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster,
                diff_drive_controller,
                steering_controller,
            ],
        )
    )

    # /cmd_vel -> 前轮转向 + 后轮差速。任何键盘遥控（teleop_twist_keyboard）
    # 发布 /cmd_vel 时，两个前轮都会跟着转向。
    steering_bridge = Node(
        package='c305_car',
        executable='cmd_vel_to_ackermann.py',
        name='cmd_vel_to_ackermann',
        parameters=[{
            # 与 urdf/c305_car.urdf.xacro 的 L / W 推导值保持一致
            'wheelbase': 1.152,           # 2 * wheel_x
            'track_width': 0.902,         # 2 * wheel_y
            'max_steering_angle': 0.5,    # 转向关节限位
            'steer_gain': 0.5,            # angular.z=1.0 -> 0.5 rad
            'ackermann': True,
        }],
        condition=IfCondition(steering_bridge_enabled),
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用 Gazebo /clock 作为仿真时间'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='是否启动 Gazebo 图形界面（false 为无界面模式）'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='是否同时启动 RViz2'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world'),
            description='Gazebo world 文件路径'),
        DeclareLaunchArgument(
            'entity', default_value='c305_car',
            description='生成到 Gazebo 中的模型名'),
        DeclareLaunchArgument(
            'z', default_value='0.02',
            description='模型初始高度（m）'),
        DeclareLaunchArgument(
            'steering_bridge', default_value='true',
            description='是否启动 /cmd_vel -> 前轮转向桥接节点'),
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        spawn_entity,
        load_controllers,
        steering_bridge,
        rviz2,
    ])
