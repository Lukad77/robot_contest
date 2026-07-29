---
kind: frontend_style
name: Qt + ROS 桌面工具界面风格
category: frontend_style
scope:
    - '**'
source_files:
    - sebot_factory/src/sebot_marking/CMakeLists.txt
    - sebot_factory/src/sebot_marking/ui/addtopics.ui
    - sebot_factory/src/sebot_marking/ui/slam.ui
    - sebot_factory/src/sebot_marking/resources/images.qrc
    - sebot_ros_stdr/src/sebot_stdr/stdr_gui/CMakeLists.txt
---

本仓库的前端 UI 全部基于 Qt（Qt4/Qt5）+ ROS 的 C++ 桌面应用，没有使用 CSS/SCSS/Tailwind 等 Web 样式体系。界面通过 Qt Designer 的 `.ui` XML 文件定义布局与控件，CMake 中通过 `QT5_WRAP_UI` / `QT4_WRAP_UI` 自动编译为 C++ 头文件，再由 C++ 代码加载并绑定逻辑。

- 样式组织方式：
  - 布局与控件结构由 `.ui` 文件声明（如 `sebot_factory/src/sebot_marking/ui/addtopics.ui`、`slam.ui`；`stdr_gui/ui/*.ui`），采用 Qt 内置布局管理器（QVBoxLayout/QHBoxLayout/QGridLayout）和标准控件（QPushButton/QLineEdit/QTreeWidget/QToolBox 等）。
  - 图标与资源通过 Qt Resource System（`.qrc` 文件，如 `resources/images.qrc`、`media.qrc`）集中管理，在 `.ui` 中以 `:/images/...` 路径引用。
  - 样式定制主要依赖 Qt 原生 API（QPalette、QStyle、setStyleSheet 等），未发现统一的样式主题文件或全局样式表。

- 关键位置：
  - sebot_marking：基于 Qt5 的 RViz 辅助标记工具，提供 SLAM 建图可视化、多目标导航点配置与 XML/YAML 持久化。
  - stdr_gui：基于 Qt4 的 STDR 仿真器 GUI，包含大量 `.ui` 表单用于机器人创建、传感器属性编辑与地图元信息编辑。

- 开发者约定：
  - 新增界面应使用 Qt Designer 生成 `.ui`，并在对应包的 `CMakeLists.txt` 中注册到 `QT*_WRAP_UI`。
  - 图标等资源统一放入 `resources/` 目录并通过 `.qrc` 注册，避免硬编码路径。
  - 不引入 CSS/前端框架，所有视觉表现通过 Qt 原生样式机制实现。