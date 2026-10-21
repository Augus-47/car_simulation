import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('c305_car')
    xacro_path = os.path.join(pkg_path, 'urdf', 'c305_car.urdf.xacro')
    rviz_path = os.path.join(pkg_path, 'config', 'display_c305_car.rviz')

    # 声明模型路径参数，方便运行时用 -s model:=xxx 覆盖
    model_arg = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(xacro_path),
        description='要加载的 xacro 模型文件路径'
    )

    # 用 xacro 命令把 xacro 文件展开成 URDF 文本
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]
        ),
        value_type=str
    )

    # 发布机器人静态 TF（把 URDF 里的关节关系广播出去）
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 发布关节状态（continuous 关节默认停在 0 位置）
    joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # RViz2 可视化
    rviz2 = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path]
    )

    return launch.LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz2,
    ])
