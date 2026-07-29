---
kind: logging_system
name: 基于 ROS rosconsole 的日志系统
category: logging_system
scope:
    - '**'
source_files:
    - sebot_factory/src/sebot_factory/src/factory.cpp
    - sebot_factory/src/sebot_factory/src/picking.cpp
    - sebot_factory/src/sebot_factory/src/confirm.cpp
    - sebot_ros_kits/src/sebot_slam/sebot_slam/src/auto_slam.cpp
    - sebot_ros_kits/src/sebot_driver/rplidar_ros/package.xml
    - sebot_ros_kits/src/sebot_navigation/costmap_2d/package.xml
---

本仓库采用 ROS 1 内置的 rosconsole 作为统一的日志框架，所有 C++ 节点通过 ROS_INFO_STREAM、ROS_WARN_STREAM、ROS_ERROR_STREAM 等宏输出结构化日志，Python 脚本则使用 print() 进行简单调试输出。

使用的框架与工具
- C++ 层：依赖 rosconsole（在多个 package.xml 中声明为 build_depend/exec_depend），通过 #include <ros/console.h> 使用 ROS_*_STREAM 宏族，支持 INFO/WARN/ERROR/FATAL 等级别。
- Python 层：未引入 logging 模块，统一使用 print() 输出调试信息，无结构化日志格式。
- 日志级别动态调整：在 sebot_slam/sebot_slam/src/auto_slam.cpp 中通过 ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) 配合 notifyLoggerLevelsChanged() 在运行时将默认 logger 设为 Debug 级别，用于调试模式。

核心文件与分布
- 工厂自动化节点 sebot_factory/src/sebot_factory/src/factory.cpp：大量使用 ROS_INFO_STREAM/ROS_WARN_STREAM/ROS_ERROR_STREAM 记录导航状态、参数加载、相机打开失败、机械臂超时等关键流程。
- 取件节点 sebot_factory/src/sebot_factory/src/picking.cpp：相机错误、步骤跟踪、机械臂动作超时的错误日志。
- 确认节点 sebot_factory/src/sebot_factory/src/confirm.cpp：AI 检测结果大小打印。
- SLAM 自动建图 sebot_ros_kits/src/sebot_slam/sebot_slam/src/auto_slam.cpp：演示了运行时动态设置 rosconsole 级别的用法。
- 第三方包如 rplidar_ros、costmap_2d、navfn、base_local_planner 等均依赖 rosconsole。

架构与约定
- 日志输出直接写入标准输出/错误流，由 ROS 控制台（rosout）或终端捕获，无独立日志文件或集中式日志后端。
- 日志级别按语义划分：INFO 用于正常流程推进（启动、到达目标、重定位成功等），WARN 用于可恢复异常（导航重试、超时等待），ERROR 用于致命错误（相机打开失败、XML 解析失败、MoveBase 连接失败）。
- 日志内容包含中文描述与关键数值拼接，便于比赛调试，但缺乏统一字段结构（如时间戳、节点名、线程ID等由 rosconsole 自动添加）。
- Python 脚本（如 uart.py、joyStick.py）仅用 print("[Info] ...") 风格输出，未与 C++ 日志体系打通。

开发者应遵循的规则
1. 优先使用 ROS_INFO_STREAM/ROS_WARN_STREAM/ROS_ERROR_STREAM 而非 std::cout/printf，确保日志可通过 rosrun rqt_console 或 roslaunch 统一管理。
2. 错误日志必须包含具体上下文（如文件名、行号、失败原因），避免空泛的 ERROR 消息。
3. 调试时可参考 auto_slam.cpp 的做法，通过 ros::console::set_logger_level 动态提升日志级别，无需修改代码重新编译。
4. Python 脚本如需与 C++ 日志对齐，建议改用 rospy.loginfo/logwarn/logerr 替代 print()。
5. 避免在生产环境中输出过多 DEBUG 级别日志，以免阻塞实时控制回路。