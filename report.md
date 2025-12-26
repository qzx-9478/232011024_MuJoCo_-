# MuJoCo MPC 汽车仪表盘 - 作业报告

## 一、项目概述

### 1.1 作业背景
本次大作业要求基于Google DeepMind开源的MuJoCo MPC框架，开发一个汽车仪表盘可视化系统。MuJoCo是一款高性能的物理仿真引擎，广泛应用于机器人控制、自动驾驶仿真等领域。MPC（模型预测控制）是现代控制理论中的重要方法，能够处理多约束优化问题。通过本项目，我们实现了物理仿真与可视化界面的结合，展示了车辆运动的实时数据。

### 1.2 实现目标
本项目的主要目标包括：
1. **环境搭建**：成功编译和运行MuJoCo MPC框架
2. **物理仿真**：创建和加载车辆物理模型，实现基本的运动控制
3. **数据提取**：从仿真环境中实时获取车辆状态数据
4. **可视化界面**：设计并实现美观的2D汽车仪表盘
5. **系统集成**：将仪表盘无缝集成到MuJoCo的3D渲染环境中

### 1.3 开发环境
- **操作系统**：Ubuntu 22.04 LTS
- **编译器**：gcc 11.4.0
- **构建系统**：CMake 3.22.1
- **图形API**：OpenGL 3.3+
- **开发工具**：VSCode + C++扩展
- **版本控制**：Git

## 二、技术方案

### 2.1 系统架构
┌─────────────────────────────────────────────────────┐
│ MuJoCo MPC 框架                                      │
├─────────────────────────────────────────────────────┤
│ 物理仿真层 │ 控制算法层 │ 渲染引擎层 │ 任务管理层 │
├─────────────────────────────────────────────────────┤
│ 汽车仪表盘模块（本作业） │
│ ├─ 数据提取模块 │ 数据处理模块 │ 2D渲染模块 ─┤ │
└─────────────────────────────────────────────────────┘
系统分为四个主要模块：
1. **物理仿真模块**：MuJoCo引擎，负责车辆动力学计算
2. **控制算法模块**：MPC控制器，计算最优控制指令
3. **渲染引擎模块**：OpenGL渲染器，绘制3D场景
4. **仪表盘模块**：本项目核心，负责数据提取和2D界面渲染

### 2.2 数据流程
车辆物理仿真 (mjData)
↓
数据提取 (DashboardDataExtractor)
↓
数据处理 (速度单位转换、模拟数据生成)
↓
仪表盘渲染 (2D OpenGL绘图)
↓
屏幕显示 (叠加在3D场景上)

**数据结构设计**：
struct DashboardData {
    double speed_kmh;      // 速度 (km/h)
    double rpm;            // 转速 (RPM)
    double fuel;           // 油量 (%)
    double temperature;    // 温度 (°C)
    // 模拟数据
    mutable double simulated_fuel;
};

### 2.3 渲染方案
采用2D覆盖层渲染方案，即在3D场景渲染完成后，切换到2D正交投影，绘制仪表盘界面。这种方案的优点是：
实现简单，不干扰3D渲染管线
性能开销小
可以灵活调整位置和大小
支持半透明效果，与3D场景融合良好

## 三、实现细节
### 3.1 场景创建
MJCF文件设计：
创建了两个XML文件：
1.car_model.xml：车辆3D模型定义
 - 使用彩色材质（红色车身、黄色车轮）
 - 添加灯光效果（前灯）
 - 定义关节和传动系统
2.task.xml：任务配置
 - MPC控制参数设置
 - 传感器定义
 - 目标点（mocap body）配置
场景特点：
蓝色棋盘格地面，增强视觉效果
车辆采用自由关节，支持全向运动
添加装饰性几何体，丰富场景内容

### 3.2 数据获取
关键代码实现：
void SimpleCar::UpdateDashboardData(const mjModel* model, const mjData* data) const {
    // 获取车身速度
    double vx = data->qvel[0];  // X方向速度
    double vy = data->qvel[1];  // Y方向速度
    double speed = std::sqrt(vx * vx + vy * vy);
    
    // 转换为km/h
    dashboard_.speed_kmh = speed * 3.6;
    
    // 模拟转速（与速度成正比）
    dashboard_.rpm = dashboard_.speed_kmh * 40.0 + 800.0;
    dashboard_.rpm = std::min(std::max(dashboard_.rpm, 800.0), 8000.0);
    
    // 模拟油量消耗
    dashboard_.simulated_fuel -= 0.001;
    if (dashboard_.simulated_fuel < 0.0) dashboard_.simulated_fuel = 100.0;
    dashboard_.fuel = dashboard_.simulated_fuel;
    
    // 模拟温度（随转速变化）
    dashboard_.temperature = 60.0 + (dashboard_.rpm / 8000.0) * 60.0;
    dashboard_.temperature = std::min(dashboard_.temperature, 120.0);
}

数据验证：
通过控制台输出实时验证数据正确性：
printf("Dashboard - Speed: %.1f km/h, RPM: %.0f, Fuel: %.1f%%, Temp: %.1f°C\n",
       dashboard_.speed_kmh, dashboard_.rpm, dashboard_.fuel, dashboard_.temperature);

### 3.3 仪表盘渲染
#### 3.3.1 速度表
- 实现思路
    绘制背景圆形（浅灰色半透明）
    添加外圈边框（亮蓝色）
    绘制刻度线和数字标签（0-50 km/h）
    计算指针角度并绘制红色指针
    在中心显示当前速度数值
- 代码片段
void SimpleCar::DrawSpeedometer2D(mjvScene* scene, float x, float y, float size) const {
    // 表盘背景
    Draw2DCircle(scene, x, y, size, 0.7f, 0.7f, 0.75f, 0.7f);
    
    // 刻度线（12个刻度）
    for (int i = 0; i < 12; i++) {
       float angle = i * (2.0f * M_PI / 12.0f);
       // ... 绘制刻度线
    }
    
    // 指针计算
    float speed_ratio = dashboard_.speed_kmh / 50.0f;
    float angle = speed_ratio * 2.0f * M_PI - M_PI/2.0f;
    
    // 绘制指针（红色）
    float end_x = x + pointer_length * std::cos(angle);
    float end_y = y + pointer_length * std::sin(angle);
    Draw2DLine(scene, x, y, end_x, end_y, 0.025f, 1.0f, 0.0f, 0.0f, 1.0f);
}

#### 3.3.2 转速表
实现特点：
    采用米色背景，橙色边框
    6000-8000 RPM区域为红色警告区
    指针为绿色，与速度表区分
    高转速时显示"HIGH RPM!"警告文字

#### 3.3.3 油量表和温度表
创新设计：
 - 动画效果：
    低油量时闪烁警告
    高温时脉冲效果
 - 视觉反馈：
    进度条动态填充
    刻度线辅助读数
    当前值标记（三角形指示器）

### 3.4 2D绘图函数库
为实现仪表盘渲染，实现了一组2D绘图辅助函数：
    Draw2DRectangle()：绘制矩形
    Draw2DLine()：绘制直线（支持旋转）
    Draw2DCircle()：绘制圆形
    AddLabel()：添加文字标签
这些函数封装了MuJoCo的几何体创建逻辑，简化了2D界面开发。

## 四、遇到的问题和解决方案
### 问题1: 仪表盘遮挡3D场景
- **现象**: 最初仪表盘完全不透明，遮挡了背后的车辆和场景
- **原因**: 默认几何体使用不透明材质
- **解决**: 
// 设置半透明颜色
geom->rgba[3] = 0.7f;  // Alpha值设为0.7（70%不透明）

### 问题2: 坐标系转换困难
- **现象**: 2D绘图位置不正确，或者随摄像机移动
- **原因**: 未正确切换投影矩阵
- **解决**: 
float screen_center_x = 0.0f;  // 屏幕中心
float screen_top = 3.0f;       // 屏幕顶部位置
DrawSpeedometer2D(scene, screen_center_x - 2.5f, screen_top - 2.0f, 0.8f);

### 问题3: 指针跳动不连续
- **现象**: 速度变化时指针跳动明显，不流畅
- **原因**: 数据更新频率与渲染频率不一致
- **解决**:     
1.确保UpdateDashboardData()在TransitionLocked()中每帧调用
2.使用浮点数计算，避免整数截断
3.保持足够的数值精度

## 五、测试与结果
### 5.1 功能测试
测试项目	测试方法	预期结果	实际结果
车辆运动	启动MPC控制	车辆向目标点移动	✅ 通过
速度表更新	观察速度表指针	指针随车速变化	✅ 通过
转速表更新	观察转速表指针	指针随转速变化	✅ 通过
油量模拟	长时间运行	油量逐渐减少，低油量警告	✅ 通过
温度模拟	改变车速	温度随转速变化	✅ 通过
目标切换	车辆到达目标	目标跳转到新位置	✅ 通过

### 5.2 性能测试
    基准帧率（无仪表盘）：约120 FPS
    当前帧率（有仪表盘）：约110 FPS
    性能损失：约8.3%
    内存占用：无明显增加
性能表现良好，仪表盘渲染对整体性能影响较小。

### 5.3 效果展示
截图说明：
    图1-环境配置：成功编译和运行MuJoCo MPC
    图2-场景加载：车辆模型和蓝色地面
    图3-速度表示例：显示当前速度25.3 km/h
    图4-转速表示例：显示当前转速2685 RPM
    图5-完整仪表盘：四个仪表组件完整显示
视频演示：
录制了1分30秒的演示视频，展示：
    程序启动和任务选择
    车辆自动导航过程
    仪表盘实时更新效果
    目标切换和持续追踪
    警告提示显示（高转速、低油量）

## 六、总结与展望
### 6.1 学习收获
通过完成本项目，获得了以下收获：
1.工程实践能力：
 - 学会了大型C++项目的编译和配置
 - 掌握了跨平台开发的基本流程
 - 理解了CMake构建系统的使用
2.技术知识提升：
 - 深入理解了物理仿真引擎的工作原理
 - 学习了OpenGL 2D/3D渲染技术
 - 掌握了实时数据可视化的实现方法
 - 了解了模型预测控制的基本概念
3.问题解决能力：
 - 学会了阅读和理解复杂代码库
 - 掌握了调试图形程序的方法
 - 培养了解决技术难题的耐心和方法
4.团队协作经验：
 - 学习使用Git进行版本控制
 - 编写清晰的代码注释和文档
 - 遵循代码规范和最佳实践
### 6.2 不足之处
1.物理模型简化：
 - 油量和温度数据为模拟值
 - 未实现真实的发动机物理模型
 - 缺少车辆动力学参数影响
2.界面定制性不足：
 - 仪表盘布局固定，无法调整
 - 颜色主题不可配置
 - 缺少用户交互功能
3.功能扩展有限：
 - 未实现导航地图
 - 缺少多媒体功能（声音、视频）
 - 不支持数据记录和回放
 
### 6.3 未来改进方向
短期改进（1-2周）：
    真实数据集成：
// 从真实的物理传感器获取数据
double real_rpm = GetEngineRPMFromPhysicsModel();
double real_fuel = CalculateFuelConsumption();
    用户界面增强：
    添加仪表盘配置菜单
       支持拖拽调整位置
    添加多个预设主题
    功能扩展：
    实现小地图/导航显示
    添加档位指示器
    实现驾驶模式切换
中期改进（1-2月）：
    高级渲染技术：
    使用着色器实现更炫酷的视觉效果
    添加粒子系统（尾气、雨滴）
    实现动态天气效果
    物理模型完善：
    集成真实的车辆动力学模型
    添加轮胎摩擦、空气阻力等因素
    实现复杂的路况模拟
    系统集成：
    支持多车辆同时显示
    添加数据记录和分析功能
    实现网络通信，支持远程监控
长期愿景（3-6月）：
    产品化开发：
    开发独立的汽车仿真平台
    支持插件系统，可扩展功能
    提供友好的用户界面
    AI集成：
    集成机器学习算法
    实现智能驾驶决策
    支持强化学习训练
    行业应用：
    开发驾驶培训模拟器
    用于自动驾驶算法测试
    作为汽车HMI开发平台
    
## 七、参考资料
1.官方文档
    MuJoCo Documentation: https://mujoco.readthedocs.io/
    MuJoCo API Reference: https://mujoco.readthedocs.io/en/stable/APIreference.html
    MuJoCo MPC GitHub: https://github.com/google-deepmind/mujoco_mpc
2.学习资源
    MuJoCo官方示例代码
    C++ Primer（第5版）
    OpenGL编程指南
3.开发工具
    操作系统: Ubuntu 22.04 LTS
    编译器: GCC 11.3.0
    构建工具: CMake 3.22.1
    版本控制: Git
