---
kind: configuration_system
name: ROS参数与配置文件体系
category: configuration_system
scope:
    - '**'
source_files:
    - sebot_factory/src/sebot_factory/launch/sebot_factory.launch
    - sebot_ros_kits/src/sebot_navigation/sebot_navigation/launch/sebot_navigation.launch
    - sebot_ros_kits/src/sebot_slam/sebot_slam/launch/sebot_gmapping.launch
    - sebot_ros_kits/src/sebot_navigation/sebot_navigation/param/move_base_params.yaml
    - sebot_ros_kits/src/sebot_navigation/sebot_navigation/param/amcl.xml
    - sebot_ros_kits/src/sebot_navigation/sebot_navigation/param/move_base.xml
    - sebot_ros_kits/src/sebot_slam/sebot_slam/param/gmapping.xml
    - sebot_ros_kits/src/sebot_slam/sebot_slam/param/hector.xml
    - sebot_ros_kits/src/sebot_driver/sebot_talon_moveit/config/joint_limits.yaml
    - sebot_factory/src/sebot_marking/resources/location.xml
    - sebot_factory/src/sebot_marking/resources/location.yaml
    - sebot_ros_kits/src/sebot_driver/sebot_urdf/config/sebot_urdf.yaml
---

本仓库采用 ROS 1 catkin 工作空间架构，配置系统围绕 launch 文件、XML/ YAML 参数文件以及资源文件（.xml/.yaml）三层组织，通过 rosparam 和 include 机制实现配置的加载与组合。

**一、核心框架与工具**
- 使用 ROS launch 作为统一的启动入口，所有节点启动、参数注入、依赖包含均通过 .launch 文件声明。
- 参数存储主要采用 YAML 和 XML 两种格式：YAML 用于算法参数（move_base、AMCL、gmapping、hector、MoveIt 等），XML 用于位置数据与 RViz 显示配置。
- 通过 `<include file="$(find <pkg>)/..."/>` 引用外部包中的 launch/XML 配置，实现跨包的配置复用。

**二、关键配置目录与文件**
- `sebot_navigation/sebot_navigation/param/`：导航栈核心参数，包括 `amcl.xml`、`move_base_params.yaml`、`costmap_common_params.yaml`、`costmap_global_params.yaml`、`planner_global_params.yaml`、`move_base.xml`。
- `sebot_slam/sebot_slam/param/`：SLAM 参数，包括 `gmapping.xml`、`hector.xml`、`auto_slam.xml`。
- `sebot_driver/sebot_talon_moveit/config/`：机械臂 MoveIt 配置，含 `joint_limits.yaml`、`cartesian_limits.yaml`、`kinematics.yaml`、`ompl_planning.yaml`、`ros_controllers.yaml` 等。
- `sebot_factory/src/sebot_marking/resources/location.yaml` 与 `location.xml`：标记工具生成的多目标导航点与 RViz 显示配置。
- `sebot_driver/sebot_urdf/config/sebot_urdf.yaml`：URDF 控制器关节名占位配置。

**三、配置加载与分层模式**
1. **Launch 层**：顶层 launch 文件（如 `sebot_factory.launch`、`sebot_navigation.launch`、`sebot_gmapping.launch`）负责组装各子模块，通过 `<node>` 的 `<param>` 直接注入运行时参数，或通过 `<include>` 引入独立 XML/YAML 配置。
2. **XML 参数层**：AMCL、move_base、gmapping、hector 等算法以独立 `.xml` 文件提供完整参数树，便于在不同场景下切换。
3. **YAML 参数层**：move_base、costmap、规划器、MoveIt 等复杂参数以 YAML 文件组织，结构清晰、可读性强。
4. **资源数据层**：`location.xml` 存储工作台、取件台、起始点等物理位置坐标；`location.yaml` 存储 RViz 显示状态与多目标路径配置。

**四、设计约定与开发规范**
- 所有可配置参数应集中在对应包的 `param/` 或 `config/` 目录下，避免在代码中硬编码数值。
- 不同运行模式（仿真/真机、建图/导航/工厂流程）通过不同的 launch 文件组合，而非修改同一份参数。
- 机器人类参数（如 PID、超时、阈值）按功能分组命名（如 `pidPoseKp`、`disClaw`、`camera_warmup_ms`），便于调试时快速定位。
- 地图、模型、标签等静态资源统一放在 `res/`、`model/`、`inference/` 等子目录，通过 launch 中的 `$(find)` 宏引用。
- 不推荐使用环境变量或 `.env` 文件管理配置，所有运行时参数均通过 ROS 参数服务器传递。

**五、典型配置示例**
- `sebot_factory.launch` 中为 `sebot_factory` 节点注入 60+ 个参数，涵盖导航超时、PID 系数、相机预热、机械臂动作等待等。
- `move_base_params.yaml` 详细定义了全局/局部规划器选择、恢复行为链、振荡检测等导航策略。
- `location.xml` 以结构化 XML 描述每个工作台的名称、局部坐标与朝向，供标记工具持久化保存。

该配置体系完全基于 ROS 原生机制，无额外配置框架，依赖 launch 文件的组合能力实现模块化与可复用性。