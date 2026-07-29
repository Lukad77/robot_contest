---
kind: error_handling
name: 错误处理机制（ROS日志与C++异常）
category: error_handling
scope:
    - '**'
source_files:
    - sebot_factory/src/sebot_factory/include/arm.hpp
    - sebot_factory/src/sebot_factory/src/factory.cpp
    - sebot_factory/src/sebot_marking/include/tinyxml.h
    - sebot_factory/src/sebot_factory/src/picking.cpp
---

该仓库是一个基于 ROS 1 catkin 的机器人竞赛工程，错误处理采用**混合模式**：C++ 层使用标准异常（try/catch）配合 libserial 库的特定异常类型进行硬件通信错误处理；业务逻辑层统一通过 ROS 日志宏（ROS_ERROR_STREAM/ROS_WARN_STREAM/ROS_INFO_STREAM）输出错误信息，并辅以语音播报作为用户反馈。没有统一的错误码或自定义异常体系。

**核心模式：**
- **硬件驱动层**（arm.hpp）：对串口操作使用 try/catch 捕获 ReadTimeout、NotOpen、OpenFailed、AlreadyOpen 等 libserial 异常，返回负值错误码而非抛出异常，保证上层调用安全
- **配置解析层**（factory.cpp）：字符串转 double 时使用 std::stod 并捕获 invalid_argument/out_of_range 异常，通过 ROS_ERROR_STREAM 记录具体失败原因
- **XML 解析层**（tinyxml.h）：定义 TIXML_NO_ERROR/TIXML_ERROR_* 枚举常量表示解析状态，调用方检查返回值判断是否出错
- **导航控制层**：move_base 连接失败时通过 ROS_ERROR_STREAM 报错并触发语音播报 "errMovebase"；导航超时通过 retryNavigation 有限次重试（maxRetryCount=1），避免无限循环
- **信号处理**：main 函数注册 SIGINT 信号处理器 exitSignal，确保退出时机械臂复位并失能，防止设备处于危险状态

**关键文件：**
- sebot_factory/src/sebot_factory/include/arm.hpp：串口异常捕获与错误码返回
- sebot_factory/src/sebot_factory/src/factory.cpp：ROS 日志输出、参数解析异常处理、导航重试逻辑
- sebot_factory/src/sebot_marking/include/tinyxml.h：XML 错误码枚举定义
- sebot_factory/src/sebot_factory/src/picking.cpp：相机打开失败的错误处理

**开发者约定：**
- 硬件 I/O 操作必须包裹 try/catch，捕获具体异常类型后返回负值错误码
- 业务错误统一使用 ROS_ERROR_STREAM/ROS_WARN_STREAM 记录，关键错误需同步触发语音播报
- 资源加载失败（如 XML 文件）应设置 enable=false 标志位阻止后续流程
- 导航等异步操作必须实现超时和有限重试机制，避免死锁