---
kind: external_dependency
name: 思岚 RPLidar 激光雷达传感器
slug: rplidar-laser
category: external_dependency
category_hints:
    - vendor_identity
scope:
    - '**'
---

### 激光雷达传感器
- **角色**：提供环境扫描数据，用于 SLAM 建图、障碍物检测和近场 PID 视觉伺服
- **集成点**：通过 rplidar_ros 包发布 `/scan` 话题，被 Confirm/Picking 模块复用
- **硬件约束**：存在 0.2m 盲区，所有距离参数需加上 20cm 补偿；与深度相机配合实现远近场互补感知
- **配置要点**：launch 文件中通过 include 方式启动，与 EKF 里程计融合提升定位精度