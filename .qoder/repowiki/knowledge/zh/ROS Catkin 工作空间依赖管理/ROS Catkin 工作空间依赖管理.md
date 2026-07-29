---
kind: dependency_management
name: ROS Catkin 工作空间依赖管理
category: dependency_management
scope:
    - '**'
source_files:
    - sebot_factory/src/CMakeLists.txt
    - sebot_factory/src/sebot_factory/package.xml
    - sebot_ros_kits/src/CMakeLists.txt
    - sebot_ros_kits/src/sebot_driver/rplidar_ros/package.xml
    - sebot_ros_kits/src/sebot_navigation/navigation/package.xml
    - sebot_ros_stdr/src/CMakeLists.txt
    - sebot_ros_stdr/src/sebot_stdr/stdr_simulator/package.xml
---

本项目采用 ROS 1 catkin 构建系统作为统一的依赖管理机制，通过三个独立的工作空间（workspace）组织不同层次的机器人功能模块。

## 核心架构与工具链

**构建系统**: 基于 catkin + CMake 的 ROS 标准构建流程，每个工作空间根目录包含标准的 `CMakeLists.txt` 和 `.catkin_workspace` 标记文件。顶层 CMakeLists.txt 使用 `catkin_workspace()` 宏自动发现并编译所有子包。

**依赖声明**: 每个 ROS 包通过 `package.xml` 文件声明依赖关系，支持三种依赖类型：
- `build_depend`: 编译时依赖（如 roscpp、std_msgs、geometry_msgs）
- `exec_depend`: 运行时依赖（如 launch、rviz 等启动工具）
- `build_export_depend`: 导出依赖（供其他包在编译时使用）

**工作空间结构**: 
- `sebot_factory/`: 应用层工厂自动化套件
- `sebot_ros_kits/`: 驱动、导航、SLAM、语音、视觉等底层能力包
- `sebot_ros_stdr/`: STDR 2D 仿真器集成

## 依赖管理策略

**版本控制**: 项目直接内嵌第三方 ROS 包的源代码（如 rplidar_ros、slam_gmapping、hector_slam、robot_pose_ekf），而非通过 apt 或源码管理器获取。这些包的版本号在各自 `package.xml` 中显式声明（如 rplidar_ros 1.5.7、navigation 1.16.7）。

**元包模式**: 使用 `<metapackage/>` 标签聚合相关功能包，如 `navigation` 元包聚合了 amcl、costmap_2d、move_base 等 14 个导航相关包，`stdr_simulator` 元包聚合了 8 个仿真相关包。

**Python 依赖**: 部分节点使用 Python 编写（如 sebot_speech 中的 `sebot_audio.py`、`uart.py`），但未发现 `requirements.txt` 或 `setup.py` 文件，Python 依赖可能通过系统包管理器安装。

## 开发约束与约定

1. **工作空间隔离**: 三个工作空间物理分离，通过 `CMAKE_PREFIX_PATH` 环境变量关联
2. **依赖最小化**: 仅在 `package.xml` 中声明必要的依赖，避免过度耦合
3. **版本锁定**: 关键第三方包以源码形式固定版本，确保构建可重现性
4. **构建顺序**: catkin 自动解析依赖图并按正确顺序编译各包

## 注意事项

- 未使用 pip、conda 等 Python 包管理器进行依赖管理
- 无 `go.mod`、`package.json` 等其他语言依赖文件
- 第三方库以源码形式直接纳入版本控制，便于比赛环境部署