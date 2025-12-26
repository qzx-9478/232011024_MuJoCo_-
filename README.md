# MuJoCo MPC 汽车仪表盘项目

## 项目信息
- **学号**: 232011024
- **姓名**: 邱智轩
- **班级**: 计科2301班
- **完成日期**: 2025年12月26日

## 项目概述
本项目基于Google DeepMind开源的MuJoCo MPC物理仿真与模型预测控制框架，实现了一个完整的汽车仪表盘可视化系统。系统在MuJoCo的3D物理仿真环境中实时渲染汽车运动，同时将车辆的速度、转速、油量、温度等关键信息通过精美的2D仪表盘界面直观展示。仪表盘包含速度表、转速表、油量表、温度表等组件，支持动态数据更新和视觉反馈（如高温警告、低油量提示等）。

## 环境要求
- **操作系统**：Ubuntu 22.04 LTS / macOS 12+ / Windows 10+ (推荐Ubuntu或WSL2)
- **编译器**：gcc/g++ 11+ 或 clang 14+
- **构建工具**：CMake 3.22+
- **依赖库**：
  - MuJoCo 2.3.5+
  - GLFW3
  - GLEW
  - Eigen3
  - absl (Abseil库)
  
## 编译和运行

### 编译步骤
\`\`\`bash
# 克隆项目（假设你已下载MuJoCo MPC源码）
cd mujoco_mpc
# 创建构建目录
mkdir -p build && cd build
# 配置项目
cmake .. -DCMAKE_BUILD_TYPE=Release
# 编译（使用4个线程加速）
cmake --build . -j4
\`\`\`

### 运行
\`\`\`bash
# 运行SimpleCar任务（已集成仪表盘）
./bin/mjpc --task SimpleCar
\`\`\`

## 功能说明

### 已实现功能
✅ 物理仿真环境：基于MuJoCo的车辆物理模型，支持转向和前进控制
✅ 实时数据采集：从仿真中提取车辆速度、位置、姿态等数据
✅ 2D仪表盘渲染：在3D场景上叠加绘制2D仪表盘界面
✅ 速度表：0-50 km/h圆形仪表，红色指针动态指示当前速度
✅ 转速表：0-8000 RPM圆形仪表，绿色指针动态指示发动机转速
✅ 油量表：百分比油量显示，根据油量变化动态改变颜色（绿→黄→红）
✅ 温度表：60-120°C温度显示，根据温度变化动态改变颜色（蓝→绿→黄→红）
✅ 警告系统：高转速警告（RPM > 6000），低油量警告（油量 < 20%），高温警告（温度 > 100°C）
✅ 目标追踪：车辆自动导航至随机目标点，到达后自动切换新目标
✅ 实时数据显示：屏幕显示车辆当前位置和目标位置

### 进阶功能
✅ UI动画效果：仪表盘指针平滑移动，警告区域闪烁效果
✅ 视觉美化：仪表盘采用半透明效果，融入3D场景不遮挡视线
✅ 颜色编码：根据数值范围自动调整显示颜色（如油量、温度）
✅ 调试信息：每秒输出仪表盘数据到控制台

## 文件说明
mujoco_mpc/mjpc/tasks/simple_car/
├── simple_car.cc          # 主实现文件（仪表盘逻辑）
├── simple_car.h           # 头文件声明
├── car_model.xml          # 车辆3D模型（外观美化）
└── task.xml              # 任务配置文件

关键代码文件
simple_car.cc：包含仪表盘数据更新、2D绘图函数、仪表盘渲染逻辑
simple_car.h：定义DashboardData结构和所有绘图函数声明
car_model.xml：美化后的车辆3D模型，使用彩色材质和灯光效果
task.xml：配置MPC控制参数和传感器设置

## 参考资料
MuJoCo官方文档：https://mujoco.readthedocs.io/
MuJoCo MPC GitHub仓库：https://github.com/google-deepmind/mujoco_mpc
OpenGL编程指南
C++现代编程实践
