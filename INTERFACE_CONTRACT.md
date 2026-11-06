# c305_car 与 auto_driving 接口约定

本文档是仿真小车与 `Augus-47/auto_driving` 对接时的唯一接口约定。除非明确说明，所有长度使用米、时间使用秒、角度速度使用弧度每秒，所有时间戳使用 ROS 时间；仿真中即 Gazebo `/clock`。

## 1. 传感器与里程计

| 话题 | 消息类型 | `header.frame_id` / `child_frame_id` | 单位与语义 | 当前状态 |
|---|---|---|---|---|
| `/camera_front/image_raw` | `sensor_msgs/msg/Image` | `camera_front_link` | `640×480`，`rgb8` | 已实现 |
| `/camera_front/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_front_link` | 与前相机图像匹配的内参 | 已实现 |
| `/camera_rear/image_raw` | `sensor_msgs/msg/Image` | `camera_rear_link` | `640×480`，`rgb8` | 已实现 |
| `/camera_rear/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_rear_link` | 与后相机图像匹配的内参 | 已实现 |
| `/camera_left/image_raw` | `sensor_msgs/msg/Image` | `camera_left_link` | `640×480`，`rgb8` | 已实现 |
| `/camera_left/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_left_link` | 与左相机图像匹配的内参 | 已实现 |
| `/camera_right/image_raw` | `sensor_msgs/msg/Image` | `camera_right_link` | `640×480`，`rgb8` | 已实现 |
| `/camera_right/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_right_link` | 与右相机图像匹配的内参 | 已实现 |
| `/scan` | `sensor_msgs/msg/LaserScan` | `rslidar` | 360°，720 点，0.15–60 m，3D 雷达中心水平面的兼容扫描 | 已实现 |
| `/rslidar_points` | `sensor_msgs/msg/PointCloud2` | `rslidar` | 16 线、360°、XYZ 单位 m，包含 `x/y/z/intensity` | 已实现 |
| `/imu` | `sensor_msgs/msg/Imu` | `imu_link` | 角速度 rad/s、线加速度 m/s²、四元数姿态；未知协方差应按 ROS 规范填写 | 待增加仿真插件 |
| `/odom` | `nav_msgs/msg/Odometry` | `header.frame_id=odom`；`child_frame_id=base_link` | 位姿在 `odom` 中；twist 在 `base_link` 中，线速度 m/s、角速度 rad/s | 已实现 |

统一要求：算法节点不得把 `/scan` 改名冒充点云；三维定位使用 `/rslidar_points`，二维避障和 Nav2 使用 `/scan`。相机算法必须同时使用同一方向的 `image_raw` 和 `camera_info`。

`auto_driving` 真实 IMU 驱动当前发布相对话题 `imu_msg`（通常解析为 `/imu_msg`），而 NDT 配置订阅 `/imu`。统一后所有算法只订阅 `/imu`；真实驱动启动时应重映射 `imu_msg:=/imu`，仿真插件直接发布 `/imu`。

## 2. 坐标系

必须保持以下 TF 链连通：

```text
map -> odom -> base_link -> rslidar
                         -> imu_link
                         -> camera_front_link
                         -> camera_rear_link
                         -> camera_left_link
                         -> camera_right_link
```

- `base_link`：车辆主体坐标系，x 向前、y 向左、z 向上。
- `rslidar`：三维雷达测量原点，x 向前、y 向左、z 向上。
- `imu_link`：IMU 测量坐标系，遵循 ROS REP-103。
- 当前相机消息使用 `camera_*_link`。需要严格投影或三维融合时，应再增加对应的 `camera_*_optical_frame`（z 向前、x 向右、y 向下），并让图像消息使用 optical frame。

禁止额外启动另一套 `odom -> base_link` 广播器，否则会产生 TF 冲突。Gazebo 中由 `diff_drive_controller` 发布这条 TF。

## 3. `car_msg/msg/VehicleState`

仓库中的实际消息定义为：

```text
std_msgs/Header header
float32 velocity
float32 steering
float32 brake
float32 gears
string  led
float32 battary
```

字段名 `battary` 是历史拼写错误，为保持消息 ABI 暂不改名。

### 3.1 统一字段语义

| 字段 | 统一单位/取值 | 方向或含义 | 命令 `/control_cmd` | 反馈 `/vehicle_info` |
|---|---|---|---|---|
| `header.stamp` | ROS 时间 | 消息产生时间 | 必须填写 | 必须填写 |
| `header.frame_id` | `base_link` | 车辆状态参考系 | 固定为 `base_link` | 固定为 `base_link` |
| `velocity` | m/s | 有符号纵向车速；正数前进，负数倒车 | 目标速度 | 实测速度 |
| `steering` | degree | 等效前轮中心转角；正数左转，负数右转 | 目标转角 | 实测转角 |
| `brake` | `[0.0, 1.0]` | 0 松开，1 完全/紧急制动 | 制动请求 | 制动状态（当前硬件协议未提供时填 0） |
| `gears` | `0.0` | 当前版本保留字段；方向由 `velocity` 正负决定 | 必须填 0 | 当前固定为 0 |
| `led` | 字符串 | 辅助状态/灯光请求 | 可选，如 `auto`、`stop` | 当前固定为 `no` |
| `battary` | `[0,100]` | 电池百分比；未知时填 0 | 不使用，填 0 | 底盘反馈字节 |

注意：`gears` 当前不是有效挡位枚举。不要把 0 解释为 N 挡，也不要同时通过 `gears` 和速度符号表达方向；现有 CAN 协议使用有符号速度表示前进/倒车。

### 3.2 当前发布端和订阅端

| 节点 | 话题方向 | 实际使用 |
|---|---|---|
| `car_simple_follow` | 发布 `/control_cmd` | `velocity` 为 m/s；`steering` 由 rad 转成 degree；检测到障碍时设置 `velocity=0, brake=1, gears=0` |
| `e2e_inference_node.py` | 发布 `/control_cmd` 或 `/e2e_debug_cmd` | `velocity` 为 m/s；`steering` 为 degree；停止时设置 `brake`，`gears=0` |
| `car_cmd_interface` | 订阅 `/control_cmd` | 只读取 `velocity` 和 `steering`；速度编码为 `m/s × 1000` 的有符号 int16，转角编码为 `degree × 100`；当前忽略 `brake/gears/led/battary/header` |
| `car_cmd_interface` | 发布 `/vehicle_info` | CAN 速度 `/1000` 得 m/s，转角 `/100` 得 degree；`gears=0, brake=0, led=no`，电池来自反馈字节 |
| `car_simple_follow` | 订阅 `/vehicle_info` | 只读取 `velocity` |
| `wheel_odometry_node.py` | 订阅 `/vehicle_info` | 正确按 m/s 和 degree 计算 Ackermann 里程计 |
| `odom.py` | 订阅 `/vehicle_info` | **错误地执行 `velocity / 100`，不得使用，必须删除该缩放或停用此脚本** |
| EM Planner | 订阅 `/vehicle_state` | 只读取 `velocity`；仓库没有稳定的 `/vehicle_state` 发布端，应统一改订阅 `/vehicle_info` 或在启动文件中重映射 |

### 3.3 安全限制

当前 `car_cmd_interface` 对 `/control_cmd` 的实现忽略 `brake`。因此在修复订阅端以前：

- `brake=1` 本身不会产生任何 CAN 制动动作。
- 所有停车命令必须同时设置 `velocity=0.0`。
- 仿真适配器也必须优先处理 `brake>0`：强制输出零速度；如果仿真没有独立制动执行器，则至少将 `/cmd_vel.linear.x` 和 `/cmd_vel.angular.z` 清零。

建议后续把命令和反馈拆成两个不同消息，或把 `gears` 改成整数枚举、把 `brake` 明确为归一化比例；这会破坏现有消息 ABI，应作为单独迁移完成。

## 4. 仿真适配器约定

`c305_sim_adapter` 已实现并默认随 `gazebo_c305_car.launch.py` 启动：

1. 订阅 `/odom` 和 `/joint_states`，发布 `/vehicle_info`；默认同时复制发布
   `/vehicle_state`，兼容仍在订阅旧话题的规划器。
2. 订阅 `/control_cmd`，将有符号速度和 degree 制中心转角转换为 `/cmd_vel`。
3. 反馈转角从 `steer_front_left_joint` 和 `steer_front_right_joint` 的实际位置按
   阿克曼几何反算，不回显命令值。
4. 收到 `brake>0.01` 时，强制把 `/cmd_vel.linear.x` 和
   `/cmd_vel.angular.z` 清零。
5. 前进速度限幅为 `0.70 m/s`，倒车速度限幅为 `0.12 m/s`；倒车由负
   `velocity` 表达，`gears` 保持 0。
6. `/control_cmd` 超过 `0.5 s` 未更新时持续发布停车命令，反馈设置
   `brake=1, led=stop`。
7. 所有反馈使用 Gazebo 仿真时间并设置 `header.frame_id=base_link`。

现有 `cmd_vel_to_ackermann.py` 的实现是：

```text
中心转角 delta = clamp(steer_gain * /cmd_vel.angular.z, -max_steer, +max_steer)
```

默认 `steer_gain=0.5`，因此适配器输出
`/cmd_vel.angular.z = radians(control_cmd.steering) / 0.5`。这里的
`angular.z` 是本项目桥接节点约定的**转向控制输入**，不是标准 Twist 偏航角速度；
不得把这个换算直接用于其它底盘。桥接节点会再根据车速、轴距和中心转角计算实际
偏航角速度并发布到 `/ackermann_cmd_vel`。倒车时保持负 `linear.x`，转角符号不反转。

主要参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `wheelbase` | `1.152` | 轴距（m） |
| `track_width` | `0.902` | 轮距（m） |
| `steer_gain` | `0.5` | 桥接输入到中心转角的比例 |
| `max_steering_angle` | `0.5` | 中心转角绝对值上限（rad） |
| `max_forward_speed` | `0.70` | 前进速度上限（m/s） |
| `max_reverse_speed` | `0.12` | 倒车速度绝对值上限（m/s） |
| `command_timeout` | `0.5` | 命令超时停车时间（s） |
| `feedback_rate` | `20.0` | 状态反馈频率（Hz） |
| `publish_vehicle_state` | `true` | 是否同时发布兼容话题 `/vehicle_state` |

## 5. 验证命令

```bash
ros2 topic type /camera_front/image_raw
ros2 topic echo --once /camera_front/image_raw --field header
ros2 topic type /scan
ros2 topic echo --once /scan --field header
ros2 topic type /rslidar_points
ros2 topic echo --once /rslidar_points --field header
ros2 topic type /odom
ros2 topic echo --once /odom --field header
ros2 topic echo --once /odom --field child_frame_id
ros2 run tf2_ros tf2_echo base_link rslidar
ros2 topic type /control_cmd
ros2 topic type /vehicle_info
ros2 topic echo --once /vehicle_info
ros2 topic echo --once /vehicle_state
```

增加 IMU 后还应执行：

```bash
ros2 topic type /imu
ros2 topic echo --once /imu --field header
ros2 run tf2_ros tf2_echo base_link imu_link
```
