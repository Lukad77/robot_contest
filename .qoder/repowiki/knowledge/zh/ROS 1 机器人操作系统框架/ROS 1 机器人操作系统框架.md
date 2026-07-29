---
kind: external_dependency
name: ROS 1 机器人操作系统框架
slug: ros-1
category: external_dependency
category_hints:
    - vendor_identity
scope:
    - '**'
---

### ROS 1 框架
- **角色**：项目基于 ROS 1（catkin）构建，作为三个工作空间（sebot_factory/sebot_ros_kits/sebot_ros_stdr）的通信中间件
- **集成点**：所有包通过 `roscpp`、`std_msgs`、`geometry_msgs`、`actionlib` 等标准依赖进行节点间通信
- **使用模式**：采用话题/服务/Action 架构，主节点 sebot_factory 通过 `/cmd_vel`、`/initialpose`、`/amcl_pose` 等标准话题与导航栈交互
- **技术栈事实**：C++（roscpp）为主，Python 脚本为辅，支持 move_base + AMCL + DWA 导航栈