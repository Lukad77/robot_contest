---
kind: external_dependency
name: STDR 二维机器人仿真器
slug: stdr-simulator
category: external_dependency
category_hints:
    - vendor_identity
    - client_constraint
scope:
    - '**'
---

### STDR 仿真环境
- **角色**：轻量级二维机器人仿真器，用于无需实体硬件的建图、定位与导航调试
- **集成点**：通过 `simulation` 参数切换真机/仿真模式，提供 stdr_server、stdr_robot、stdr_navigation 等配套包
- **约束特性**：仅支持二维环境，适合比赛流程验证和算法调试，不支持三维视觉和机械臂物理仿真
- **启动方式**：从 `server_with_map_and_gui_plus_robot.launch` 入手，配合 RViz 可视化