---
kind: external_dependency
name: 百度 EdgeBoard NPU 推理加速平台
slug: baidu-edgeboard-npu
category: external_dependency
category_hints:
    - vendor_identity
    - sdk_real_api
scope:
    - '**'
---

### NPU 推理加速
- **角色**：为 YOLOv3 目标检测提供硬件加速，替代纯 CPU 推理
- **部署特点**：需要现场解 tar 包并编译目标文件，属于嵌入式部署场景的典型模式