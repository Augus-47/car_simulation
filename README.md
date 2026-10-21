# c305_car 四轮阿克曼小车仿真 — 操作文档

本文档面向本项目的使用者，覆盖环境准备、编译、启动、控制、传感器、参数修改和常见问题。

---

## 1. 项目简介

`c305_car` 是一个基于 **ROS 2 Humble + Gazebo Classic 11** 的四轮小车仿真包，包含：

| 项目 | 内容 |
|---|---|
| 驱动方式 | 后轮差速驱动 + 前轮阿克曼转向（双层关节） |
| 传感器 | 1 个 360° 2D 激光雷达 + 4 个 150° 广角相机 |
| 控制接口 | `ros2_control`（`gazebo_ros2_control` 硬件插件） |
| 可视化 | rviz2 / Gazebo GUI |
| 模型格式 | URDF 由 `xacro` 生成 |

主要文件：

```
src/c305_car/
├── urdf/c305_car.urdf.xacro      # 主模型（几何、关节、传感器、ros2_control）
├── config/controllers.yaml       # 控制器参数
├── config/display_c305_car.rviz  # rviz2 显示配置
├── launch/display_c305_car.launch.py   # 仅 rviz2 显示
├── launch/gazebo_c305_car.launch.py    # Gazebo 仿真
├── scripts/cmd_vel_to_ackermann.py     # /cmd_vel → 前轮转向 + 后轮差速
├── meshes/gnss_dome.stl          # 天线穹顶网格
├── media/materials/scripts/c305_car.material  # Gazebo 自定义半透明材质
└── images/                       # 实物参考照片
```

> 目前模型所有部件都设置为 **半透明**（alpha = 0.5）：
> Gazebo 由 `media/materials/scripts/c305_car.material` 提供，rviz2 由
> `config/display_c305_car.rviz` 里 RobotModel 的 `Alpha` 控制。修改方法见 [Q11](#q11-如何调整半透明程度)。

---

## 2. 环境要求

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo Classic 11（`gazebo_ros`）
- 关键依赖包：

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-diff-drive-controller \
  ros-humble-position-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard     # 可选，键盘遥控
```

验证 Gazebo 版本：

```bash
gazebo --version        # 应为 11.x
```

---

## 3. 编译

```bash
cd ~/Simulation_study/car_simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select c305_car
source install/setup.bash
```

> **重要**：`CMakeLists.txt` 使用 `install(DIRECTORY ...)` 复制文件，安装目录不是符号链接。
> **每次修改 urdf / launch / config 后都必须重新 `colcon build`**，否则运行的是旧版本。

建议把下面两行写进 `~/.bashrc`：

```bash
source /opt/ros/humble/setup.bash
source ~/Simulation_study/car_simulation/install/setup.bash
```

---

## 4. 启动

### 4.1 仅用 rviz2 查看模型

```bash
ros2 launch c305_car display_c305_car.launch.py
```

可选参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `model` | 包内 xacro 路径 | 指定其它 xacro 文件 |

### 4.2 Gazebo 仿真（主要用法）

```bash
ros2 launch c305_car gazebo_c305_car.launch.py
```

启动参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `gui` | `true` | 是否打开 Gazebo 图形界面 |
| `rviz` | `false` | 是否同时启动 rviz2 |
| `use_sim_time` | `true` | 使用 Gazebo 仿真时间 |
| `world` | `gazebo_ros` 的 `empty.world` | 世界文件 |
| `entity` | `c305_car` | 模型名称 |
| `z` | `0.02` | 初始生成高度（m） |
| `steering_bridge` | `true` | 是否启动 `/cmd_vel` → 前轮转向桥接节点 |

常用组合：

```bash
# 有界面
ros2 launch c305_car gazebo_c305_car.launch.py gui:=true

# 有界面 + rviz2
ros2 launch c305_car gazebo_c305_car.launch.py gui:=true rviz:=true

# 无界面（服务器/远程）
ros2 launch c305_car gazebo_c305_car.launch.py gui:=false
```

启动流程（自动完成）：

1. `xacro` 展开模型 → `robot_description`
2. `robot_state_publisher` 发布 TF
3. 把本包 share 目录加入 `GAZEBO_RESOURCE_PATH`，使 Gazebo 能找到
   `media/materials/scripts/c305_car.material`（半透明材质）
4. 启动 Gazebo（加载 `libgazebo_ros_init.so` 与 `libgazebo_ros_factory.so`）
5. `spawn_entity.py` 把模型生成到世界
6. 模型生成完成后自动加载三个控制器：
   - `joint_state_broadcaster`
   - `diff_drive_controller`
   - `steering_controller`

---

## 5. 控制小车

### 5.1 重要规则

1. `/cmd_vel` 的 **命令超时是 0.5 秒**（`cmd_vel_timeout: 0.5`）。
   必须用 `-r` **持续发布**；只发一次会在 0.5 秒后自动停车。
2. YAML 冒号后**必须有空格**，否则会报
   `'Vector3' object has no attribute 'x:0.5'`。
3. 转向范围是 **±0.5 rad（约 ±28.6°）**；`/cmd_vel.angular.z = ±1.0` 对应满舵。

### 5.2 前进 / 后退

```bash
# 前进 0.5 m/s
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 后退 0.3 m/s：把 x 改成 -0.3
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

按 `Ctrl+C` 停止发布，车会在 0.5 秒内停下。

### 5.3 转向（用 `/cmd_vel` 的 `angular.z`）

启动时 `cmd_vel_to_ackermann` 节点会把 `angular.z` 映射成前轮转角：

```
前轮中心转角 δ = clamp(0.5 × angular.z, -0.5, +0.5) rad
```

再按阿克曼几何算出左右前轮的差角（转弯时内轮比外轮转得更多）。例如：

| `angular.z` | 左前轮 | 右前轮 | 含义 |
|---|---|---|---|
| `+1.0` | `+0.500 rad` | `+0.423 rad` | 左转打满 |
| `+0.5` | `+0.250 rad` | `+0.207 rad` | 左转半圈 |
| `0.0` | `0` | `0` | 回正 |
| `-1.0` | `-0.423 rad` | `-0.500 rad` | 右转打满 |

```bash
# 原地左打方向（车不动，只有前轮转）
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 1.0}}"

# 回正
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

> 真实汽车在静止时不能原地旋转，所以 `linear.x = 0` 时只有前轮转向、车身不动。
> 想让车转弯，需要同时给 `linear.x`（见 5.4）。

### 5.4 走曲线（边开边转）

```bash
# 前进 0.5 m/s，同时左转打满
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

- `linear.x` 控制速度，`angular.z` 控制方向；
- 后轮差速由桥接节点按前轮转角反算，因此里程计和实际转向一致；
- 后退时 `linear.x` 取负值，转向方向不变（车沿圆弧倒车）；
- 满舵时后轴中心的转弯半径约 `1.152 / tan(0.5) ≈ 2.11 m`。

**直接控制转向控制器**（调试用）仍然可用，但 `/cmd_vel` 持续发布时会被桥接节点覆盖：

```bash
# 直接给两个前轮下角度，顺序 [左前, 右前]，单位 rad
ros2 topic pub --once /steering_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.3, 0.3]}"
```

需要完全手动控制转向时，启动仿真关闭桥接：

```bash
ros2 launch c305_car gazebo_c305_car.launch.py steering_bridge:=false
```

### 5.5 键盘遥控

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- `i` / `,`：前进 / 后退
- `j` / `l`：左转 / 右转（**前轮会跟着转**）
- `u` / `o` / `m` / `.`：斜向行驶（前进/后退 + 转向）
- `k`：停车回正
- `q` / `z`：整体加减速；`w` / `x`：只调线速度；`e` / `c`：只调转向量

行为说明：

- 按 `j` / `l` 时，桥接节点会立即让两个前轮转到对应角度；
- 静止时按 `j` / `l`：前轮打方向，车身不转（和真车一样）；
- 再按 `i` 起步，车就会沿前轮方向行驶（`angular.z` 会保持，直到按 `k` 或反向键）；
- 转向灵敏度由 `steer_gain` 控制：`angular.z = 1.0` 对应 `steer_gain` 弧度，
  默认 `0.5`（即满舵）。觉得转向太猛可以在 launch 文件里调小。

### 5.6 推荐速度

| 场景 | `linear.x` | `angular.z` | 对应前轮中心转角 |
|---|---|---|---|
| 直线测试 | 0.3–0.5 m/s | 0 | 0 |
| 小半径转弯 | 0.3 m/s | 1.0 | 0.50 rad（满舵） |
| 大半径转弯 | 0.5 m/s | 0.4 | 0.20 rad |

---

## 6. 话题一览

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 订阅 | 遥控指令：`linear.x` 速度、`angular.z` 转向 |
| `/ackermann_cmd_vel` | `geometry_msgs/msg/Twist` | 内部 | 桥接节点修正后交给后轮差速控制器的指令 |
| `/odom` | `nav_msgs/msg/Odometry` | 发布 | 里程计 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 发布 | 6 个关节的位置/速度 |
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | 发布 | 控制器内部状态 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 发布 | 激光雷达 |
| `/camera_front/image_raw` | `sensor_msgs/msg/Image` | 发布 | 前相机 |
| `/camera_rear/image_raw` | `sensor_msgs/msg/Image` | 发布 | 后相机 |
| `/camera_left/image_raw` | `sensor_msgs/msg/Image` | 发布 | 左相机 |
| `/camera_right/image_raw` | `sensor_msgs/msg/Image` | 发布 | 右相机 |
| `/camera_*/camera_info` | `sensor_msgs/msg/CameraInfo` | 发布 | 相机内参 |
| `/steering_controller/commands` | `std_msgs/msg/Float64MultiArray` | 订阅 | 前轮转角（由桥接节点发布） |
| `/clock` | `rosgraph_msgs/msg/Clock` | 发布 | Gazebo 仿真时间 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 发布 | 坐标变换 |

相机还提供 `image_raw/compressed`、`compressedDepth`、`theora` 等压缩传输话题。

常用检查命令：

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /camera_front/image_raw
ros2 topic echo /odom --field pose.pose.position
ros2 topic echo /joint_states
```

---

## 7. 传感器说明

### 7.1 激光雷达

位于车顶，参数如下：

| 参数 | 值 |
|---|---|
| 话题 | `/scan` |
| 类型 | `sensor_msgs/LaserScan` |
| `frame_id` | `lidar_link` |
| 扫描角 | −π ~ +π（360°） |
| 采样数 | 720 |
| 量程 | 0.10 ~ 30.0 m |
| 更新率 | 10 Hz |
| 噪声 | 高斯，σ = 0.01 |

rvtool 查看：

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix c305_car)/share/c305_car/config/display_c305_car.rviz
# 手动添加 LaserScan，话题选 /scan
```

### 7.2 四个相机

| 参数 | 值 |
|---|---|
| 话题 | `/camera_front|rear|left|right/image_raw` |
| 分辨率 | 640 × 480 |
| 格式 | R8G8B8 |
| 更新率 | 30 Hz |
| 水平视场角 | 2.61799 rad（约 150°） |
| `frame_id` | `camera_*_link` |

> **鱼眼说明**：Gazebo 原生相机插件不支持鱼眼畸变模型，这里用 150° 大视场角近似。
> 若算法需要真实鱼眼内参/畸变，请在 ROS 层用 `image_proc` 或自写节点处理。

查看图像：

```bash
ros2 run rqt_image_view rqt_image_view
# 或
ros2 run rviz2 rviz2    # 添加 Image 显示，话题选 /camera_front/image_raw
```

### 7.3 GNSS 天线

模型包含定位天线外观（`gnss_antenna_link`），但**没有 GNSS 传感器插件**。
如需输出定位数据，可在该 link 下添加 `libgazebo_ros_gps_sensor.so` 插件。

---

## 8. 控制器与关节接口

### 8.1 控制器总表

| 控制器 | 类型 | 命令话题 | 控制对象 |
|---|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | — | 发布全部关节状态 |
| `diff_drive_controller` | `diff_drive_controller/DiffDriveController` | `/ackermann_cmd_vel` | 后轮速度 |
| `steering_controller` | `position_controllers/JointGroupPositionController` | `/steering_controller/commands` | 前轮转角 |

另外有一个普通节点负责键盘/遥控指令转换：

| 节点 | 订阅 | 发布 | 作用 |
|---|---|---|---|
| `cmd_vel_to_ackermann` | `/cmd_vel` | `/steering_controller/commands`、`/ackermann_cmd_vel` | 把 `angular.z` 变成前轮转角，并按转向几何修正后轮速度 |

`controller_manager` 更新频率：**100 Hz**。

### 8.2 关节接口

| 关节 | 类型 | 命令接口 | 状态接口 | 说明 |
|---|---|---|---|---|
| `wheel_rear_left_joint` | continuous | velocity | position, velocity | 后轮驱动 |
| `wheel_rear_right_joint` | continuous | velocity | position, velocity | 后轮驱动 |
| `steer_front_left_joint` | revolute（±0.5） | position | position, velocity | 左前转向 |
| `steer_front_right_joint` | revolute（±0.5） | position | position, velocity | 右前转向 |
| `wheel_front_left_joint` | continuous | effort（未接管） | position, velocity | 被动随动 |
| `wheel_front_right_joint` | continuous | effort（未接管） | position, velocity | 被动随动 |

前轮自转关节说明：

- 只声明 `effort` 命令接口，**没有任何控制器接管**，因此扭矩恒为 0；
- 前轮由地面摩擦带动自由滚动，控制器无法驱动它；
- 仍然提供 `position/velocity` 状态接口，所以 `/joint_states` 能读到前轮转角，rviz 的 TF 才能连通。

### 8.3 后轮差速参数

| 参数 | 值 |
|---|---|
| `wheel_separation` | 0.902 m |
| `wheel_radius` | 0.180 m |
| `publish_rate` | 50 Hz |
| `odom_frame_id` | `odom` |
| `base_frame_id` | `base_link` |
| `use_stamped_vel` | `false`（用 `Twist`） |
| `enable_odom_tf` | `true` |
| `cmd_vel_timeout` | 0.5 s |

> 话题重映射写在 `c305_car.urdf.xacro` 的 `gazebo_ros2_control` 插件 `<ros>` 块里：
> `diff_drive_controller/cmd_vel_unstamped` → `ackermann_cmd_vel`，
> `diff_drive_controller/odom` → `odom`。
> 差速控制器不再直接订阅 `/cmd_vel`，而是由 `cmd_vel_to_ackermann` 按前轮转角
> 反算角速度后再转发，保证后轮差速与转向几何一致。
> **不要**把重映射写到 spawner 或 YAML 里（无效）。

### 8.4 手动管理控制器

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces

# 卸载/重载（调试用）
ros2 control unload_controller steering_controller
ros2 control load_controller --set-state active steering_controller
```

---

## 9. TF 树

```
odom
└── base_link
    ├── chassis_link
    │   ├── bumper_rod_front_left_link / front_right / rear_left / rear_right
    │   └── wheel_axle_front_left_link / front_right / rear_left / rear_right
    ├── white_body_platform_link
    ├── white_body_front_link / rear_link / left_link / right_link
    ├── front_bumper_link / rear_bumper_link
    ├── wheel_mount_front_left_link / front_right / rear_left / rear_right
    ├── upper_frame_link
    ├── camera_front_link / rear_link / left_link / right_link
    ├── lidar_base_link
    │   └── lidar_stand_link
    │       └── lidar_link
    │           ├── lidar_cap_bottom_link
    │           ├── lidar_body_link
    │           └── lidar_cap_top_link
    ├── gnss_base_link
    │   └── gnss_stand_link
    │       └── gnss_antenna_link
    ├── steer_front_left_link
    │   └── wheel_front_left_link
    ├── steer_front_right_link
    │   └── wheel_front_right_link
    ├── wheel_rear_left_link
    └── wheel_rear_right_link
```

> 上面的父子关系按 URDF 逻辑关系书写；Gazebo 生成模型时会把固定关节的子 link 合并到父 link（`*_fixed_joint_lump__*`），但 `robot_state_publisher` 仍会按 URDF 发布完整 TF。

检查 TF：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link wheel_front_left_link
```

---

## 10. 模型尺寸参数

所有尺寸集中在 `c305_car.urdf.xacro` 开头，主要靠 `L` / `W` 整体缩放：

```xml
<xacro:property name="L" value="1.60"/>   <!-- 前后方向 -->
<xacro:property name="W" value="1.10"/>   <!-- 左右方向 -->
```

由它们推导出的关键尺寸：

| 参数 | 计算式 | 当前值 |
|---|---|---|
| 底盘离地间隙 | 常量 | 0.10 m |
| 车轮直径 | 0.225 × L | 0.36 m |
| 车轮半径 | 0.1125 × L | 0.18 m |
| 轮距（左右轮中心） | 2 × 0.41 × W | 0.902 m |
| 前后轮中心距的一半 | 0.36 × L | 0.576 m |
| 上层框架宽 | 常量 | 0.68 m |
| 框架总高 | 常量 | 0.50 m |
| 转向关节限位 | 常量 | ±0.5 rad |

**修改 `L` / `W` 后必须同步 `config/controllers.yaml`：**

```yaml
wheel_separation: 2 * 0.41 * W
wheel_radius:     0.5 * 0.225 * L
```

否则里程计和差速控制会出现比例误差。

---

## 11. 常见问题（FAQ）

### Q1. Gazebo 里模型是纯白色 / 没有颜色

**原因**：sdformat 在 URDF→SDF 转换时会**丢弃** `<visual>` 里的
`<material><color rgba="..."/></material>`，Gazebo 只认 `<gazebo reference>` 里的材质名。

**已修复**：模型里为所有可见 link 添加了自定义半透明材质引用：

```xml
<xacro:gazebo_material link="chassis_link" mat="C305Car/BlackTransparent"/>
```

这些材质定义在 `media/materials/scripts/c305_car.material` 中。Gazebo 通过
`GAZEBO_RESOURCE_PATH` 查找该脚本，`gazebo_c305_car.launch.py` 已自动把本包的
share 目录加进去，无需手动设置。

**新增 link 时**记得也加一行 `gazebo_material`，否则该 link 在 Gazebo 里显示为默认灰白色。

> 注意：rviz2 读取 URDF 的 `<material><color>`，Gazebo 读取 `<gazebo reference>`，
> 两套颜色/透明度互不影响。

### Q2. `Failed to populate field: 'Vector3' object has no attribute 'x:0.5'`

YAML 冒号后少了空格。要写 `x: 0.5`，不能写 `x:0.5`。

### Q3. 没有 `/cmd_vel` 话题

检查 `gazebo_ros2_control` 插件的 `<ros>` 块是否包含：

```xml
<remapping>diff_drive_controller/cmd_vel_unstamped:=cmd_vel</remapping>
```

重映射必须写在插件里，写在 spawner 或 YAML 中无效。

### Q4. 相机只有 `camera_info`，没有 `image_raw`

常见原因：模型里有 Gazebo 无法解析的 `package://` 网格路径。
本项目已把 GNSS 穹顶改为 `file://$(find c305_car)/meshes/gnss_dome.stl`。
**新增 mesh 时请使用 `file://` 绝对路径**，否则相机渲染管线会整体失效。

### Q5. rviz 报 `No transform from [wheel_front_left_link] to [base_link]`

前轮自转关节必须出现在 `ros2_control` 中并带状态接口，`joint_state_broadcaster` 才能读到角度。
本项目已用 `effort` 命令接口 + `position/velocity` 状态接口解决，不要删除。

### Q6. 启动报 `Address already in use`（端口 11345）

有残留的 gzserver：

```bash
pkill -9 -f gzserver
pkill -9 -f gzclient
```

### Q7. `/joint_states` 被全 0 覆盖 / rviz 里关节乱跳

可能同时开着旧的 `joint_state_publisher`（例如之前用 `display_c305_car.launch.py` 启动的）。
它和 Gazebo 的 `joint_state_broadcaster` 会同时发布同名话题：

```bash
pkill -9 -f joint_state_publisher
```

**不要同时运行** `display_c305_car.launch.py` 和 `gazebo_c305_car.launch.py`。

### Q8. 修改了 xacro 但仿真没变化

安装目录是复制而非符号链接，改完必须：

```bash
colcon build --packages-select c305_car
source install/setup.bash
```

### Q9. 前轮不滚动 / 前轮打滑

这里说的是前轮的**自转**（`wheel_front_*_joint`）。它是**被动关节**（不归控制器管），
只靠地面摩擦带动，这是设计行为。转向关节 `steer_front_*_joint` 是主动控制的，
会随 `/cmd_vel.angular.z` 正常转动。若需要主动驱动前轮自转，要修改
`ros2_control` 接口并新增控制器。

### Q10. Gazebo 里车不动 / 抖动

- 检查三个控制器是否都处于 `active`：

  ```bash
  ros2 control list_controllers
  ```

- 检查轮胎摩擦系数 `<mu1>/<mu2>` 是否为 1.0；
- 确认发布 `/cmd_vel` 时使用了 `-r` 持续发布。

### Q11. 如何调整半透明程度

模型所有部件默认 alpha = 0.5（半透明）。两套渲染各自独立：

**Gazebo**：修改 `media/materials/scripts/c305_car.material` 中每个材质的
`alpha_op_ex source1 src_manual src_current 0.5`，最后一列即 alpha：

- `0.0` = 完全透明，`0.5` = 半透明，`1.0` = 不透明。
- 5 个材质（Black / White / DarkGrey / Grey / Blue）都要改。
- 改完执行 `colcon build --packages-select c305_car` 并重启 Gazebo。

```bash
# 例如全部改成 0.3
sed -i 's/src_current 0\.5$/src_current 0.3/' \
  src/c305_car/media/materials/scripts/c305_car.material
```

**rviz2**：修改 `config/display_c305_car.rviz` 中 RobotModel 的 `Alpha`：

```yaml
- Alpha: 0.5          # 0.0 全透明，1.0 不透明
  Class: rviz_default_plugins/RobotModel
```

也可以直接在 rviz2 的 Displays 面板里拖动 RobotModel → Alpha 滑块实时调整。
URDF 里的颜色 alpha 保持 1.0，透明度完全由 rviz 的 Alpha 与 Gazebo 材质控制。

### Q12. 键盘遥控时前轮不跟着转

按顺序检查：

```bash
# 1. 桥接节点是否在运行
ros2 node list | grep cmd_vel_to_ackermann

# 2. /cmd_vel 是否在持续发布（cmd_vel_timeout = 0.5 s）
ros2 topic hz /cmd_vel

# 3. 桥接节点是否发出了转向角
ros2 topic echo /steering_controller/commands

# 4. 前轮关节是否收到位置
ros2 topic echo /joint_states --field position
```

常见原因：

- 启动时加了 `steering_bridge:=false`；
- 只按了一次方向键，`teleop_twist_keyboard` 停止发布后 `cmd_vel_timeout`
  会让车停下（但转向角会保持，不会丢失）；
- 直接往 `/steering_controller/commands` 发布角度，同时 `/cmd_vel` 又在持续
  发布，桥接节点会把角度覆盖掉。

桥接节点未运行时，可以手动重启：

```bash
ros2 run c305_car cmd_vel_to_ackermann.py
```

---

## 12. 启动自检清单

按顺序执行，可快速定位问题：

1. **模型能否解析**

   ```bash
   xacro src/c305_car/urdf/c305_car.urdf.xacro > /tmp/check.urdf && echo OK
   ```

2. **rviz 能否显示**：`ros2 launch c305_car display_c305_car.launch.py`
3. **Gazebo 能否生成模型**：`ros2 launch c305_car gazebo_c305_car.launch.py gui:=true`
4. **控制器是否激活**

   ```bash
   ros2 control list_controllers
   ```

5. **驱动是否生效**

   ```bash
   # 终端 A：持续发布
   ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

   # 终端 B：桥接节点应有输出，车应开始移动
   ros2 topic echo /ackermann_cmd_vel
   ros2 topic echo /odom --field pose.pose.position
   ```

6. **转向是否生效（前轮跟着转）**

   ```bash
   # 终端 A：原地左打方向
   ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

   # 终端 B：应看到 steer_front_left ≈ 0.5、steer_front_right ≈ 0.423
   ros2 topic echo /joint_states --field position
   ```

   若桥接节点已关闭（`steering_bridge:=false`），可直接发布转向角：

   ```bash
   ros2 topic pub --once /steering_controller/commands \
     std_msgs/msg/Float64MultiArray "{data: [0.3, 0.3]}"
   ```

7. **传感器是否出数**

   ```bash
   ros2 topic hz /scan
   ros2 topic hz /camera_front/image_raw
   ```

8. **TF 是否连通**：`ros2 run tf2_tools view_frames`

---

## 13. 停止与清理

正常结束：在 launch 终端按 `Ctrl+C`。

强制清理残留进程：

```bash
pkill -9 -f "ros2 launch c305_car"
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f cmd_vel_to_ackermann
pkill -9 -f robot_state_publisher
pkill -9 -f joint_state_publisher
pkill -9 -f rviz2
```

---

## 14. 已知限制

| 限制 | 说明 |
|---|---|
| 鱼眼畸变 | Gazebo 原生相机不支持，用 150° FOV 近似 |
| 前轮转向 | 已按阿克曼几何计算内外轮差角；满舵时内轮超过 ±0.5 rad 会被关节限位截断 |
| 前轮驱动 | 被动随动，不参与驱动 |
| GNSS | 只有外观，没有传感器插件 |
| 材质 | 使用自定义 `.material` 脚本实现半透明；关闭了光照（`lighting off`），颜色为纯色，无明暗渐变 |
| 半透明遮挡 | 关闭了深度写入（`depth_write off`），多个半透明面重叠时前后排序可能不完美 |
| 雷达 | 2D 单线，非 3D 点云；需要 3D 时修改 `<ray>` 配置并改输出类型 |


---

*文档中的命令均基于 `~/Simulation_study/car_simulation` 路径，若项目位置不同请自行替换。*
