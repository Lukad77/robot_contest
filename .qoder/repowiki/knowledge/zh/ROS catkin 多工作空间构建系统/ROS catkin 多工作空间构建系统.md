---
kind: build_system
name: ROS catkin 多工作空间构建系统
category: build_system
scope:
    - '**'
source_files:
    - sebot_factory/src/CMakeLists.txt
    - sebot_ros_kits/src/CMakeLists.txt
    - sebot_ros_stdr/src/CMakeLists.txt
    - sebot_factory/src/sebot_factory/CMakeLists.txt
    - sebot_factory/src/sebot_marking/CMakeLists.txt
    - sebot_factory/src/sebot_factory/package.xml
    - sebot_ros_kits/src/sebot_driver/sebot_urdf/package.xml
    - sebot_ros_stdr/src/sebot_stdr/stdr_msgs/package.xml
    - .travis.yml
---

## 构建系统与工具链

本项目采用 **ROS 1 + catkin** 作为统一构建系统，组织为三个相互独立的 catkin 工作空间（workspace），每个工作空间包含 `src/` 目录和顶层 `CMakeLists.txt`，通过 `.catkin_workspace` 标记。构建流程遵循标准 ROS 开发约定：

- **构建工具**：`catkin_make`（每个工作空间独立执行）
- **包管理**：`package.xml` 声明依赖关系，`CMakeLists.txt` 定义编译规则
- **依赖解析**：通过 `find_package(catkin REQUIRED COMPONENTS ...)` 和 `pkg_search_module()` 查找外部库

## 工作空间架构

仓库包含三个并列的 catkin 工作空间，按职责分层组织：

1. **sebot_factory**（应用层）：比赛业务逻辑，包含 sebot_factory（主状态机）和 sebot_marking（Qt GUI 标注工具）两个包
2. **sebot_ros_kits**（驱动与功能层）：聚合机器人驱动、导航栈、SLAM、语音、视觉等基础能力
3. **sebot_ros_stdr**（仿真层）：STDR 二维仿真器及其配套包，支持无硬件调试

每个工作空间的顶层 `CMakeLists.txt` 均为标准的 catkin 工作空间模板，通过 `catkin_workspace()` 宏自动发现并构建子包。

## 包级构建配置

各包的 CMakeLists.txt 遵循统一模式：
- **C++ 标准**：默认 C++11/14，通过 `set(CMAKE_CXX_STANDARD 14)` 或 `CMAKE_CXX_FLAGS` 设置
- **依赖声明**：使用 `find_package(catkin REQUIRED COMPONENTS ...)` 声明 ROS 依赖
- **外部库**：通过 `pkg_search_module()` 查找 OpenCV、libserial、PPNC、ONNX 等第三方库
- **目标定义**：`add_executable()` 定义可执行文件，`target_link_libraries()` 链接依赖
- **安装规则**：部分包显式声明 `install(TARGETS ... RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})`

## Qt 集成

sebot_marking 包展示了 Qt5 与 catkin 的集成方式：
- 通过 `find_package(Qt5 REQUIRED Core Widgets)` 查找 Qt 组件
- 使用 `QT5_ADD_RESOURCES()`、`QT5_WRAP_UI()`、`QT5_WRAP_CPP()` 处理 .ui 和 .qrc 文件
- 将生成的 moc 文件和 UI 头文件加入源列表

## 依赖管理与版本控制

- **ROS 包版本**：package.xml 中 `<version>` 标签管理包版本（如 0.0.0、1.0.0、0.3.2）
- **依赖类型**：区分 `build_depend`、`exec_depend`、`build_export_depend` 等依赖类型
- **消息生成**：通过 `message_generation` 和 `message_runtime` 支持自定义 msg/srv/action
- **架构无关包**：URDF 等纯配置文件包声明 `<architecture_independent />` 以优化构建

## CI/CD 支持

项目包含 Travis CI 配置文件（`.travis.yml`），用于跨 ROS 发行版（indigo/kinetic/melodic/noetic）的自动化构建测试，基于 ros_buildfarm 框架实现预发布检查。

## 开发者规范

1. **工作空间隔离**：每个功能域独立工作空间，避免依赖冲突
2. **包粒度合理**：按功能模块拆分为独立包，便于复用和维护
3. **依赖最小化**：仅在 package.xml 中声明必要依赖，避免过度耦合
4. **构建一致性**：保持 CMakeLists.txt 结构一致，便于团队协作
5. **仿真优先**：通过 STDR 仿真环境进行算法验证，减少真机调试成本