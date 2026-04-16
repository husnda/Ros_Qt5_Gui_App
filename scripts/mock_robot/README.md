# ROS2 模拟机器人与 GUI App 测试指南

本文档介绍如何使用 `scripts/mock_robot` 目录下的模拟环境对 `ros_qt5_gui_app` 进行功能验证。

## 1. 模拟机器人概览

模拟机器人（Mock Robot）是一个基于 ROS2 Humble 的 Python 节点，它模拟了真实机器人的核心通信接口，允许在没有实体硬件的情况下测试 GUI 的显示和控制功能。

### 支持的 Topic 列表

| Topic 名称 | 消息类型 | 功能描述 |
| :--- | :--- | :--- |
| `/map` | `nav_msgs/OccupancyGrid` | 模拟静态地图（200x200） |
| `/odom` | `nav_msgs/Odometry` | 实时里程计数据 |
| `/tf` | `tf2_msgs/TFMessage` | 坐标变换树 `map -> odom -> base_link` |
| `/scan` | `sensor_msgs/LaserScan` | 模拟 360 度激光雷达数据 |
| `/battery` | `sensor_msgs/BatteryState` | 模拟电池电量 |
| `/camera/front/image_raw` | `sensor_msgs/Image` | 动态 RGB 测试图像 |
| `/plan` / `/local_plan` | `nav_msgs/Path` | 模拟导航路径规划 |
| `/cmd_vel` | `geometry_msgs/Twist` | **订阅**：接收运动指令 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | **订阅**：接收导航目标点 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | **订阅**：接收重定位请求 |

---

## 2. 快速开始步骤

### 第一步：环境准备
在项目根目录下，执行环境设置脚本。该脚本会自动配置 ROS2 环境变量及库加载路径：
```bash
source setup_env.sh
```

### 第二步：启动模拟机器人
进入模拟器目录并执行启动脚本：
```bash
cd scripts/mock_robot
./run_mock.sh
```
*看到日志输出 `Mock Robot Started.` 说明启动成功。*

### 第三步：启动 GUI 应用程序
打开另一个终端，在项目根目录下执行：
```bash
source setup_env.sh
cd build
./ros_qt5_gui_app
```

---

## 3. 功能验证说明

### 3.1 运动控制验证
1. 在 GUI 界面中找到 **Joystick** 控件。
2. 尝试移动摇杆或使用方向键。
3. **预期结果**：模拟机器人终端会收到指令，同时 GUI 中的机器人图标会平滑移动。

### 3.2 地图与雷达验证
1. GUI 启动后应能立即看到白底黑框的地图。
2. 地图上应显示一圈模拟的雷达散点。
3. **预期结果**：地图中心会出现机器人图标，雷达散点随机器人移动同步。

### 3.3 导航路径测试
1. 在 GUI 顶部菜单栏点击 **"view"** -> **"Task"** 开启任务窗口。
2. 在 Task 窗口中点击 **"Add Point"**。
3. 在地图任意位置点击。
4. **预期结果**：地图上会出现模拟的导航规划路径（直线段）。

### 3.4 视频流测试
1. 在 GUI 顶部菜单栏点击 **"view"** -> **"image/front"**。
2. **预期结果**：图像窗口弹出，显示实时变化的彩色图案。此功能已进行线程安全加固，支持频繁开关切换。

---

## 4. 常见问题排查

*   **无法加载 ROS2 驱动？**
    *   确保已执行 `source setup_env.sh` 且 `LD_LIBRARY_PATH` 已包含 ROS2 库路径。
*   **图像窗口切换崩溃？**
    *   该问题已在最新代码中修复。如果仍有发生，请检查是否使用了最新编译的二进制文件。
