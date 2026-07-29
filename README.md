# robot_contest · 智能服务机器人竞赛

百度智能云智能服务机器人赛参赛项目（西北大学 · 小智快跑）。基于 **ROS 1（catkin）** 的服务机器人完整工程，覆盖「仿真 → 建图定位 → 导航 → 视觉识别 → 机械臂取放 → 语音播报」全链路，比赛场景为**智能工厂送件**：机器人按订单导航到取件台，用 AI 视觉搜索零件，机械臂抓取后再送达指定桌面并完成结算。

## 仓库结构

仓库由三个相互独立的 catkin 工作空间组成，各自含 `src/`，需分别 `catkin_make`：

```
robot_contest/
├── sebot_factory/     # 应用层：比赛业务逻辑（状态机 / 取件 / 下单 / 付款 / YOLO）
├── sebot_ros_kits/    # 机器人套件：驱动 / 导航 / 建图 / 底盘 / 语音 / 视觉
└── sebot_ros_stdr/    # 仿真层：STDR 二维机器人仿真器及配套导航/建图
```

### sebot_factory — 应用层

| 包 | 说明 |
| --- | --- |
| `sebot_factory` | 比赛主业务节点。`factory.cpp` 为送件场景状态机（基于 MultiNavi），`confirm.cpp` 需求确认、`picking.cpp` 智能取件、`summary.cpp` 结算汇总。配置（导航超时、PID、取件距离、补扫参数等）集中在 `launch/sebot_factory.launch`。 |
| `sebot_marking` | 基于 qrviz 的建图/标注上位机 GUI（Qt），用于采图与点位标注。 |

业务子流程对应独立 launch：`sebot_ordering`（下单）、`sebot_paying`（付款）、`sebot_picking`（取件）、`sebot_arm`（机械臂）、`sebot_yolo`（NPU 加速 Yolov3 目标检测）、`sebot_model_check`（模型自检）。

### sebot_ros_kits — 机器人套件

| 包组 | 包含 | 说明 |
| --- | --- | --- |
| `sebot_driver` | `rplidar_ros`、`sebot_talon`、`sebot_talon_moveit`、`sebot_urdf`、`robot_pose_ekf` | 思岚激光雷达驱动、Talon 机械臂及其 MoveIt! 配置、机器人 URDF 模型、EKF 里程计融合。 |
| `sebot_navigation` | `amcl`、`move_base`、`costmap_2d`、`dwa_local_planner`、`global_planner`、`navfn`、`map_server` 等 | 完整 ROS navigation 导航栈，外加 `sebot_navigation` 提供地图、参数与 RViz 配置。 |
| `sebot_slam` | `slam_gmapping`、`slam_hector`、`sebot_slam` | Gmapping 与 Hector 两套建图方案及建图/存图 launch。 |
| `sebot_robot` | `controller.cpp`、`joystick.cpp`、`keyboards.cpp`、`transform.cpp` | 底盘运动控制、手柄/键盘遥控、静态 TF 发布。 |
| `sebot_speech` | `sebot_audio.py`、`uart.py` | 语音播报（经串口驱动）。 |
| `sebot_visions` | `sebotCollection.py`、`joyStick.py` | 视觉采集与遥控脚本。 |

### sebot_ros_stdr — 仿真层

STDR（Simple Two-Dimensional Robot）轻量二维仿真器及其适配包：`stdr_server`、`stdr_robot`、`stdr_gui`、`stdr_navigation`、`stdr_amcl`、`stdr_move_base`、`stdr_hector_mapping`、`stdr_launchers` 等。无需实体硬件即可在仿真中跑通建图、定位与导航，便于调试比赛流程。

## 技术栈

- **框架**：ROS 1 + catkin，C++（roscpp）为主，Python 脚本为辅
- **导航**：move_base + AMCL 蒙特卡洛定位 + DWA 局部规划
- **建图**：Gmapping / Hector SLAM
- **机械臂**：MoveIt!（Talon）+ PID 位姿/夹爪控制
- **视觉**：Yolov3 目标检测（NPU 加速推理）
- **传感器**：RPLidar 激光雷达 + IMU + 里程计（EKF 融合）
- **仿真**：STDR 二维仿真器
- **可视化/上位机**：RViz、Qt（sebot_marking）

## 编译与运行

每个工作空间独立编译（以 `sebot_factory` 为例）：

```bash
cd sebot_factory
catkin_make
source devel/setup.bash
```

真机/主流程启动（会依次拉起语音、底盘控制、EKF、激光雷达、地图服务、AMCL、move_base、RViz 及业务主节点）：

```bash
roslaunch sebot_factory sebot_factory.launch
```

仿真调试（STDR）可从 `sebot_ros_stdr/src/sebot_stdr/stdr_launchers/launch/` 下的 `server_with_map_and_gui_plus_robot.launch` 入手。

> 说明：`sebot_factory.launch` 中 `simulation` 参数用于切换 STDR 仿真模式；`debug` 参数控制是否打开图像识别界面，正式比赛计时时建议置为 `false`。

## 主要业务流程

1. 上电自检，发布静态 TF，融合里程计（EKF）就绪；
2. 加载先验地图，AMCL 初始化机器人位姿；
3. 语音播报提示，按订单导航至取件台（move_base / MultiNavi）；
4. 需求确认（`confirm`）后，YOLO 视觉搜索目标零件，多帧检测确认；
5. 机械臂 + 机械爪 PID 闭环抓取零件（`picking`）；
6. 导航送达目标桌面，完成放置与结算（`summary` / `paying`）。

## 维护

- 作者：hpw（sasu@saishukeji.com）
- 许可：见各包 `package.xml` / `LICENSE`
