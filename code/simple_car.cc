// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/tasks/simple_car/simple_car.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include <absl/random/random.h>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {

std::string SimpleCar::XmlPath() const {
  return GetModelPath("simple_car/task.xml");
}

std::string SimpleCar::Name() const { return "SimpleCar"; }

// ------- Residuals for simple_car task ------
//     Position: Car should reach goal position (x, y)
//     Control:  Controls should be small
// ------------------------------------------
void SimpleCar::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                     double* residual) const {
  // ---------- Position (x, y) ----------
  // Goal position from mocap body
  residual[0] = data->qpos[0] - data->mocap_pos[0];  // x position
  residual[1] = data->qpos[1] - data->mocap_pos[1];  // y position

  // ---------- Control ----------
  residual[2] = data->ctrl[0];  // forward control
  residual[3] = data->ctrl[1];  // turn control
}

// ============ 更新仪表盘数据 ============
void SimpleCar::UpdateDashboardData(const mjModel* model, const mjData* data) const {
  // 获取车身速度
  double vx = data->qvel[0];  // X方向速度
  double vy = data->qvel[1];  // Y方向速度
  double speed = std::sqrt(vx * vx + vy * vy);
  
  // 转换为km/h
  dashboard_.speed_kmh = speed * 3.6;

  // 模拟转速
  dashboard_.rpm = dashboard_.speed_kmh * 40.0 + 800.0;
  if (dashboard_.rpm > 8000.0) dashboard_.rpm = 8000.0;
  if (dashboard_.rpm < 800.0) dashboard_.rpm = 800.0;

  // 模拟油量消耗
  dashboard_.simulated_fuel -= 0.001;
  if (dashboard_.simulated_fuel < 0.0) dashboard_.simulated_fuel = 100.0;
  dashboard_.fuel = dashboard_.simulated_fuel;

  // 模拟温度
  dashboard_.temperature = 60.0 + (dashboard_.rpm / 8000.0) * 60.0;
  if (dashboard_.temperature > 120.0) dashboard_.temperature = 120.0;

  // 调试输出
  if (fmod(data->time, 1.0) < 0.01) {
    printf("Dashboard - Speed: %.1f km/h, RPM: %.0f, Fuel: %.1f%%, Temp: %.1f°C\n",
           dashboard_.speed_kmh, dashboard_.rpm, dashboard_.fuel, dashboard_.temperature);
  }
}

// -------- Transition for simple_car task --------
//   If car is within tolerance of goal ->
//   move goal randomly.
// ------------------------------------------------
void SimpleCar::TransitionLocked(mjModel* model, mjData* data) {
  // Car position (x, y)
  double car_pos[2] = {data->qpos[0], data->qpos[1]};
  
  // Goal position from mocap
  double goal_pos[2] = {data->mocap_pos[0], data->mocap_pos[1]};
  
  // Distance to goal
  double car_to_goal[2];
  mju_sub(car_to_goal, goal_pos, car_pos, 2);
  
  // If within tolerance, move goal to random position
  if (mju_norm(car_to_goal, 2) < 0.2) {
    absl::BitGen gen_;
    data->mocap_pos[0] = absl::Uniform<double>(gen_, -2.0, 2.0);
    data->mocap_pos[1] = absl::Uniform<double>(gen_, -2.0, 2.0);
    data->mocap_pos[2] = 0.01;  // keep z at ground level
  }

  // 更新仪表盘数据
  UpdateDashboardData(model, data);
}

// ============ 2D绘制辅助函数 ============
void SimpleCar::Draw2DRectangle(mjvScene* scene, float x, float y, float width, float height,
                               float r, float g, float b, float a) const {
  if (scene->ngeom >= scene->maxgeom) return;
  
  mjvGeom* geom = scene->geoms + scene->ngeom;
  geom->type = mjGEOM_BOX;
  geom->size[0] = width;
  geom->size[1] = height;
  geom->size[2] = 0.001f;  // 非常薄的2D矩形
  
  geom->pos[0] = x;
  geom->pos[1] = y;
  geom->pos[2] = 0.0f;  // 2D平面，z=0
  
  geom->rgba[0] = r;
  geom->rgba[1] = g;
  geom->rgba[2] = b;
  geom->rgba[3] = a;
  
  // 使用单位矩阵
  float mat[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
  };
  for (int i = 0; i < 9; i++) {
    geom->mat[i] = mat[i];
  }
  
  geom->category = mjCAT_DECOR;
  scene->ngeom++;
}

void SimpleCar::Draw2DLine(mjvScene* scene, float x1, float y1,
                          float x2, float y2, float width,
                          float r, float g, float b, float a) const {
  // 使用一个薄的BOX作为线条
  float dx = x2 - x1;
  float dy = y2 - y1;
  float length = std::sqrt(dx*dx + dy*dy);
  float angle = std::atan2(dy, dx);
  
  if (scene->ngeom >= scene->maxgeom) return;
  
  mjvGeom* geom = scene->geoms + scene->ngeom;
  geom->type = mjGEOM_BOX;
  geom->size[0] = length / 2.0f;
  geom->size[1] = width / 2.0f;
  geom->size[2] = 0.001f;
  
  geom->pos[0] = (x1 + x2) / 2.0f;
  geom->pos[1] = (y1 + y2) / 2.0f;
  geom->pos[2] = 0.0f;
  
  geom->rgba[0] = r;
  geom->rgba[1] = g;
  geom->rgba[2] = b;
  geom->rgba[3] = a;
  
  // 旋转矩阵
  float cos_a = std::cos(angle);
  float sin_a = std::sin(angle);
  float mat[9] = {
    cos_a, -sin_a, 0.0f,
    sin_a, cos_a,  0.0f,
    0.0f,  0.0f,   1.0f
  };
  for (int i = 0; i < 9; i++) {
    geom->mat[i] = mat[i];
  }
  
  geom->category = mjCAT_DECOR;
  scene->ngeom++;
}

void SimpleCar::Draw2DCircle(mjvScene* scene, float x, float y, float radius,
                            float r, float g, float b, float a) const {
  if (scene->ngeom >= scene->maxgeom) return;
  
  mjvGeom* geom = scene->geoms + scene->ngeom;
  geom->type = mjGEOM_ELLIPSOID;  // 使用椭球体而不是球体
  geom->size[0] = radius;
  geom->size[1] = radius;
  geom->size[2] = 0.001f;  // 非常薄的2D圆形
  
  geom->pos[0] = x;
  geom->pos[1] = y;
  geom->pos[2] = 0.0f;  // 2D平面，z=0
  
  geom->rgba[0] = r;
  geom->rgba[1] = g;
  geom->rgba[2] = b;
  geom->rgba[3] = a;
  
  // 使用单位矩阵
  float mat[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
  };
  for (int i = 0; i < 9; i++) {
    geom->mat[i] = mat[i];
  }
  
  geom->category = mjCAT_DECOR;
  scene->ngeom++;
}

// ============ 2D速度表（调整为0-50 km/h范围） ============
void SimpleCar::DrawSpeedometer2D(mjvScene* scene, float x, float y, float size) const {
  // 表盘背景（亮灰色圆形）
  Draw2DCircle(scene, x, y, size, 0.7f, 0.7f, 0.75f, 0.7f);  // 降低透明度到0.7
  
  // 外圈边框（亮蓝色）
  Draw2DCircle(scene, x, y, size * 1.05f, 0.4f, 0.7f, 1.0f, 0.6f);  // 降低透明度到0.6
  Draw2DCircle(scene, x, y, size * 0.95f, 0.3f, 0.3f, 0.4f, 0.8f);  // 降低透明度到0.8
  
  // 刻度线（保持12个刻度）
  for (int i = 0; i < 12; i++) {
    float angle = i * (2.0f * M_PI / 12.0f);
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    
    float inner_radius = size * 0.8f;
    float outer_radius = size * 0.9f;
    
    Draw2DLine(scene, 
              x + inner_radius * cos_a, y + inner_radius * sin_a,
              x + outer_radius * cos_a, y + outer_radius * sin_a,
              0.02f, 0.1f, 0.1f, 0.2f, 0.8f);  // 降低透明度
  }
  
  // 主要刻度
  for (int i = 0; i < 4; i++) {
    float angle = i * (2.0f * M_PI / 4.0f);
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    
    float inner_radius = size * 0.75f;
    float outer_radius = size * 0.9f;
    
    Draw2DLine(scene, 
              x + inner_radius * cos_a, y + inner_radius * sin_a,
              x + outer_radius * cos_a, y + outer_radius * sin_a,
              0.03f, 0.0f, 0.5f, 1.0f, 0.9f);
  }
  
  // 数字标签（0, 10, 20, 30, 40, 50 km/h）- 改为0-50范围
  // 总共显示6个标签
  for (int i = 0; i < 6; i++) {
    float angle = i * (2.0f * M_PI / 6.0f);  // 6等分
    float cos_a = std::cos(angle - M_PI/2.0f);  // 从顶部开始
    float sin_a = std::sin(angle - M_PI/2.0f);
    
    float label_radius = size * 0.7f;
    char label[10];
    std::snprintf(label, sizeof(label), "%d", i * 10);  // 0, 10, 20, 30, 40, 50
    
    AddLabel(scene, 
             x + label_radius * cos_a, 
             y + label_radius * sin_a, 
             0.01f,  // 稍微高一点
             label, 0.1f, 0.1f, 0.1f, 0.9f);
  }
  
  // 指针 - 改为鲜艳的红色，范围调整为0-50 km/h
  float speed_ratio = dashboard_.speed_kmh / 50.0f;  // 改为50 km/h最大
  if (speed_ratio > 1.0f) speed_ratio = 1.0f;
  float angle = speed_ratio * 2.0f * M_PI - M_PI/2.0f;  // 从顶部开始
  float pointer_length = size * 0.6f;
  
  float end_x = x + pointer_length * std::cos(angle);
  float end_y = y + pointer_length * std::sin(angle);
  
  // 指针主体 - 鲜艳红色
  Draw2DLine(scene, x, y, end_x, end_y, 0.025f, 1.0f, 0.0f, 0.0f, 1.0f);
  
  // 指针尾部 - 鲜艳红色
  float tail_length = size * 0.2f;
  float tail_x = x - tail_length * std::cos(angle) * 0.3f;
  float tail_y = y - tail_length * std::sin(angle) * 0.3f;
  Draw2DLine(scene, x, y, tail_x, tail_y, 0.02f, 1.0f, 0.0f, 0.0f, 1.0f);
  
  // 中心点 - 改为黑色增加对比度
  Draw2DCircle(scene, x, y, size * 0.06f, 0.0f, 0.0f, 0.0f, 1.0f);
  Draw2DCircle(scene, x, y, size * 0.04f, 1.0f, 1.0f, 1.0f, 1.0f);
  
  // 当前速度值（中心显示）
  char speed_text[50];
  std::snprintf(speed_text, sizeof(speed_text), "%.1f", dashboard_.speed_kmh);  // 显示1位小数
  AddLabel(scene, x, y, 0.02f, speed_text, 0.15f, 0.15f, 0.1f, 0.9f);
  
  // 单位标签
  AddLabel(scene, x, y - size * 0.25f, 0.02f, "km/h", 0.08f, 0.0f, 0.3f, 0.8f);
  
  // 标题
  AddLabel(scene, x, y + size * 1.2f, 0.02f, "SPEED", 0.15f, 0.0f, 0.5f, 1.0f);
}

// ============ 2D转速表 ============
void SimpleCar::DrawTachometer2D(mjvScene* scene, float x, float y, float size) const {
  // 表盘背景（亮米色圆形）
  Draw2DCircle(scene, x, y, size, 0.75f, 0.75f, 0.7f, 0.7f);  // 降低透明度到0.7
  
  // 外圈边框（亮橙色）
  Draw2DCircle(scene, x, y, size * 1.05f, 1.0f, 0.6f, 0.3f, 0.6f);  // 降低透明度到0.6
  Draw2DCircle(scene, x, y, size * 0.95f, 0.4f, 0.3f, 0.2f, 0.8f);  // 降低透明度到0.8
  
  // 红色警告区域（6000-8000 RPM）
  if (dashboard_.rpm > 6000.0) {
    float warning_ratio = (dashboard_.rpm - 6000.0f) / 2000.0f;
    if (warning_ratio > 1.0f) warning_ratio = 1.0f;
    
    for (int i = 0; i < 3; i++) {
      float alpha = 0.3f + 0.7f * (i / 3.0f);
      Draw2DCircle(scene, x, y, size * (0.9f - i * 0.05f), 
                   1.0f, 0.3f, 0.3f, alpha * warning_ratio);
    }
  }
  
  // 刻度线
  for (int i = 0; i < 12; i++) {
    float angle = i * (2.0f * M_PI / 12.0f);
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    
    float inner_radius = size * 0.8f;
    float outer_radius = size * 0.9f;
    
    Draw2DLine(scene, 
              x + inner_radius * cos_a, y + inner_radius * sin_a,
              x + outer_radius * cos_a, y + outer_radius * sin_a,
              0.02f, 0.1f, 0.1f, 0.2f, 0.8f);  // 降低透明度
  }
  
  // 数字标签（0, 2, 4, 6, 8 x1000）
  for (int i = 0; i < 5; i++) {
    float angle = i * (2.0f * M_PI / 5.0f);
    float cos_a = std::cos(angle - M_PI/2.0f);
    float sin_a = std::sin(angle - M_PI/2.0f);
    
    float label_radius = size * 0.7f;
    char label[10];
    std::snprintf(label, sizeof(label), "%d", i * 2);
    
    AddLabel(scene, 
             x + label_radius * cos_a, 
             y + label_radius * sin_a, 
             0.01f,
             label, 0.1f, 0.1f, 0.1f, 0.9f);
  }
  
  // 指针 - 改为鲜艳的绿色
  float rpm_ratio = dashboard_.rpm / 8000.0f;
  if (rpm_ratio > 1.0f) rpm_ratio = 1.0f;
  float angle = rpm_ratio * 2.0f * M_PI - M_PI/2.0f;
  float pointer_length = size * 0.6f;
  
  float end_x = x + pointer_length * std::cos(angle);
  float end_y = y + pointer_length * std::sin(angle);
  
  // 指针主体 - 鲜艳绿色
  Draw2DLine(scene, x, y, end_x, end_y, 0.025f, 0.0f, 1.0f, 0.0f, 1.0f);
  
  // 指针尾部 - 鲜艳绿色
  float tail_length = size * 0.2f;
  float tail_x = x - tail_length * std::cos(angle) * 0.3f;
  float tail_y = y - tail_length * std::sin(angle) * 0.3f;
  Draw2DLine(scene, x, y, tail_x, tail_y, 0.02f, 0.0f, 1.0f, 0.0f, 1.0f);
  
  // 中心点 - 改为黑色增加对比度
  Draw2DCircle(scene, x, y, size * 0.06f, 0.0f, 0.0f, 0.0f, 1.0f);
  Draw2DCircle(scene, x, y, size * 0.04f, 1.0f, 1.0f, 1.0f, 1.0f);
  
  // 当前RPM值（中心显示）
  char rpm_text[50];
  std::snprintf(rpm_text, sizeof(rpm_text), "%.0f", dashboard_.rpm);
  AddLabel(scene, x, y, 0.02f, rpm_text, 0.15f, 0.15f, 0.1f, 0.9f);
  
  // 单位标签
  AddLabel(scene, x, y - size * 0.25f, 0.02f, "RPM", 0.08f, 0.0f, 0.3f, 0.8f);
  
  // 标题
  AddLabel(scene, x, y + size * 1.2f, 0.02f, "TACHOMETER", 0.15f, 1.0f, 0.5f, 0.0f);
  
  // 高转速警告
  if (dashboard_.rpm > 6000.0) {
    AddLabel(scene, x, y - size * 1.4f, 0.02f, "HIGH RPM!", 0.12f, 1.0f, 0.1f, 0.1f);
  }
}

// ============ 2D油量表（简化版） ============
void SimpleCar::DrawFuelGauge2D(mjvScene* scene, float x, float y, float width, float height) const {
  // 移除外部背景和边框，直接绘制油量条
  // 油量条 - 根据油量百分比动态变化
  float fuel_width = (dashboard_.fuel / 100.0f) * width;  // 直接使用全部宽度
  if (fuel_width > 0.01f) {  // 避免绘制过小的条
    float fuel_x = x - (width - fuel_width) / 2.0f;  // 居中计算起始位置
    float fuel_y = y;
    float fuel_height = height * 0.8f;  // 使用大部分高度
    
    float fuel_color_r, fuel_color_g, fuel_color_b;
    
    if (dashboard_.fuel > 50.0f) {
      fuel_color_r = 0.2f; fuel_color_g = 1.0f; fuel_color_b = 0.2f;  // 绿色
    } else if (dashboard_.fuel > 20.0f) {
      fuel_color_r = 1.0f; fuel_color_g = 1.0f; fuel_color_b = 0.2f;  // 黄色
    } else {
      fuel_color_r = 1.0f; fuel_color_g = 0.2f; fuel_color_b = 0.2f;  // 红色
    }
    
    // 绘制动态的油量条 - 增加对比度
    Draw2DRectangle(scene, fuel_x, fuel_y, 
                    fuel_width, fuel_height, 
                    fuel_color_r, fuel_color_g, fuel_color_b, 1.0f);  // 保持不透明
    
    // 绘制油量条边框，使其更明显
    Draw2DRectangle(scene, fuel_x, fuel_y, 
                    fuel_width, fuel_height, 
                    0.0f, 0.0f, 0.0f, 0.3f);  // 黑色边框
    
    // 添加油量变化动画效果（当油量低于20%时闪烁）
    if (dashboard_.fuel < 20.0f) {
      static float blink_timer = 0.0f;
      blink_timer += 0.1f;  // 简单的计时器
      if (fmod(blink_timer, 1.0f) > 0.5f) {
        // 闪烁效果：绘制一个半透明的红色覆盖层
        Draw2DRectangle(scene, x, y, width, height, 1.0f, 0.2f, 0.2f, 0.3f);
      }
    }
  } else {
    // 油量为0时显示空的背景
    Draw2DRectangle(scene, x, y, width, height * 0.6f, 0.3f, 0.3f, 0.3f, 0.5f);
  }
  
  // 标签
  char fuel_text[50];
  std::snprintf(fuel_text, sizeof(fuel_text), "FUEL: %.1f%%", dashboard_.fuel);
  AddLabel(scene, x, y + height * 0.8f, 0.02f, fuel_text, 0.1f, 0.1f, 0.1f, 1.0f);
  
  // 低油量警告
  if (dashboard_.fuel < 20.0) {
    AddLabel(scene, x, y - height * 0.8f, 0.02f, "LOW FUEL!", 0.12f, 1.0f, 0.1f, 0.1f);
  }
  
  // 添加油量刻度线（简化版）
  for (int i = 0; i <= 5; i++) {
    float marker_x = x - width/2.0f + (width / 5.0f) * i;
    float marker_width = 0.02f;
    Draw2DLine(scene, marker_x, y - height * 0.4f, marker_x, y - height * 0.2f, 
               marker_width, 0.2f, 0.2f, 0.3f, 0.8f);
  }
}

// ============ 2D温度表（简化版） ============
void SimpleCar::DrawTemperatureGauge2D(mjvScene* scene, float x, float y, float width, float height) const {
  // 移除外部背景和边框，直接绘制温度条
  float min_temp = 60.0f;
  float max_temp = 120.0f;
  float temp_range = max_temp - min_temp;
  
  // 计算温度比例
  float temp_ratio = (dashboard_.temperature - min_temp) / temp_range;
  if (temp_ratio < 0.0f) temp_ratio = 0.0f;
  if (temp_ratio > 1.0f) temp_ratio = 1.0f;
  
  float temp_width = temp_ratio * width;  // 直接使用全部宽度
  if (temp_width > 0.01f) {
    float temp_x = x - (width - temp_width) / 2.0f;  // 居中计算起始位置
    float temp_y = y;
    float temp_height = height * 0.8f;
    
    float temp_color_r, temp_color_g, temp_color_b;
    
    if (temp_ratio < 0.5f) {
      // 低温到中温：蓝到绿
      float t = temp_ratio / 0.5f;
      temp_color_r = 0.3f * (1.0f - t);
      temp_color_g = 0.5f + 0.5f * t;
      temp_color_b = 1.0f * (1.0f - t);
    } else if (temp_ratio < 0.8f) {
      // 中温到高温：绿到黄
      float t = (temp_ratio - 0.5f) / 0.3f;
      temp_color_r = 0.3f + 0.7f * t;
      temp_color_g = 1.0f * (1.0f - 0.2f * t);
      temp_color_b = 0.5f * (1.0f - t);
    } else {
      // 高温：黄到红
      float t = (temp_ratio - 0.8f) / 0.2f;
      temp_color_r = 1.0f;
      temp_color_g = 0.8f * (1.0f - t);
      temp_color_b = 0.2f * (1.0f - t);
    }
    
    // 绘制动态的温度条 - 增加对比度
    Draw2DRectangle(scene, temp_x, temp_y, 
                    temp_width, temp_height, 
                    temp_color_r, temp_color_g, temp_color_b, 1.0f);  // 保持不透明
    
    // 绘制温度条边框，使其更明显
    Draw2DRectangle(scene, temp_x, temp_y, 
                    temp_width, temp_height, 
                    0.0f, 0.0f, 0.0f, 0.3f);  // 黑色边框
    
    // 添加温度过高动画效果
    if (dashboard_.temperature > 100.0f) {
      static float heat_timer = 0.0f;
      heat_timer += 0.05f;
      float pulse = 0.3f + 0.3f * sin(heat_timer * 5.0f);  // 脉冲效果
      Draw2DRectangle(scene, x, y, width, height, 1.0f, 0.3f, 0.3f, pulse);
    }
  } else {
    // 温度为最低时显示空的背景
    Draw2DRectangle(scene, x, y, width, height * 0.6f, 0.3f, 0.3f, 0.3f, 0.5f);
  }
  
  // 标签
  char temp_text[50];
  std::snprintf(temp_text, sizeof(temp_text), "TEMP: %.1f°C", dashboard_.temperature);
  AddLabel(scene, x, y + height * 0.8f, 0.02f, temp_text, 0.1f, 0.1f, 0.1f, 1.0f);
  
  // 高温警告
  if (dashboard_.temperature > 100.0) {
    AddLabel(scene, x, y - height * 0.8f, 0.02f, "OVERHEAT!", 0.12f, 1.0f, 0.1f, 0.1f);
  }
  
  // 添加温度刻度线（简化版）
  for (int i = 0; i <= 5; i++) {
    float marker_x = x - width/2.0f + (width / 5.0f) * i;
    float marker_width = 0.02f;
    Draw2DLine(scene, marker_x, y - height * 0.4f, marker_x, y - height * 0.2f, 
               marker_width, 0.2f, 0.2f, 0.3f, 0.8f);
  }
  
  // 添加当前温度值标记
  float marker_ratio = (dashboard_.temperature - min_temp) / temp_range;
  if (marker_ratio >= 0.0f && marker_ratio <= 1.0f) {
    float marker_x = x - width/2.0f + (width * marker_ratio);
    // 绘制当前位置标记（三角形）
    Draw2DLine(scene, marker_x, y - height * 0.4f, marker_x - 0.05f, y - height * 0.2f,
               0.03f, 0.0f, 0.0f, 0.0f, 0.8f);
    Draw2DLine(scene, marker_x, y - height * 0.4f, marker_x + 0.05f, y - height * 0.2f,
               0.03f, 0.0f, 0.0f, 0.0f, 0.8f);
  }
}

// ============ 添加标签 ============
void SimpleCar::AddLabel(mjvScene* scene, float x, float y, float z, const char* text, 
                        float size, float r, float g, float b) const {
  if (scene->ngeom < scene->maxgeom) {
    mjvGeom* geom = scene->geoms + scene->ngeom;
    geom->type = mjGEOM_LABEL;
    geom->size[0] = geom->size[1] = geom->size[2] = size;
    geom->pos[0] = x;
    geom->pos[1] = y;
    geom->pos[2] = z;
    geom->rgba[0] = r;
    geom->rgba[1] = g;
    geom->rgba[2] = b;
    geom->rgba[3] = 1.0f;
    std::strncpy(geom->label, text, sizeof(geom->label) - 1);
    geom->label[sizeof(geom->label) - 1] = '\0';
    geom->category = mjCAT_DECOR;
    scene->ngeom++;
  }
}

// draw task-related geometry in the scene
void SimpleCar::ModifyScene(const mjModel* model, const mjData* data,
                             mjvScene* scene) const {
  // 检查 scene 是否有效
  if (!scene || scene->maxgeom == 0) return;
  
  // ===== 在屏幕上方固定位置绘制仪表盘 =====
  // 使用相对坐标，将仪表盘放在屏幕顶部中间
  
  float screen_center_x = 0.0f;  // 屏幕中心
  float screen_top = 3.0f;       // 屏幕顶部位置
  
  // ===== 绘制仪表盘标题 =====
  AddLabel(scene, screen_center_x, screen_top - 0.5f, 0.5f, 
           "CAR DASHBOARD", 0.25f, 0.0f, 0.5f, 1.0f);
  
  // ===== 绘制仪表（固定在屏幕上方） =====
  // 速度表（左侧）
  DrawSpeedometer2D(scene, screen_center_x - 2.5f, screen_top - 2.0f, 0.8f);
  
  // 转速表（右侧）
  DrawTachometer2D(scene, screen_center_x + 2.5f, screen_top - 2.0f, 0.8f);
  
  // 油量表（左下方，简化版）
  DrawFuelGauge2D(scene, screen_center_x - 2.5f, screen_top - 3.5f, 1.5f, 0.4f);
  
  // 温度表（右下方，简化版）
  DrawTemperatureGauge2D(scene, screen_center_x + 2.5f, screen_top - 3.5f, 1.5f, 0.4f);
  
  // ===== 绘制目标标记（红色球）- 原有3D物体 =====
  if (scene->ngeom < scene->maxgeom) {
    mjvGeom* geom = scene->geoms + scene->ngeom;
    geom->type = mjGEOM_SPHERE;
    geom->size[0] = geom->size[1] = geom->size[2] = 0.15;
    geom->pos[0] = data->mocap_pos[0];
    geom->pos[1] = data->mocap_pos[1];
    geom->pos[2] = 0.2;
    geom->rgba[0] = 1.0f; geom->rgba[1] = 0.0f; 
    geom->rgba[2] = 0.0f; geom->rgba[3] = 0.8f;
    geom->category = mjCAT_DECOR;
    scene->ngeom++;
  }
  
  // 车辆当前位置标签 - 跟随车辆移动
  int car_body_id = mj_name2id(model, mjOBJ_BODY, "car");
  if (car_body_id >= 0) {
    double* car_pos = data->xpos + 3 * car_body_id;
    char pos_text[50];
    // 显示车辆的位置坐标，而不是温度
    std::snprintf(pos_text, sizeof(pos_text), "Car: (%.2f, %.2f)", car_pos[0], car_pos[1]);
    AddLabel(scene, car_pos[0], car_pos[1], car_pos[2] + 2.0f, pos_text, 0.1f, 0.0f, 1.0f, 0.0f);
  }
  
  // ===== 绘制目标位置标签 =====
  // 在红色球上方添加目标位置标签
  char goal_text[50];
  std::snprintf(goal_text, sizeof(goal_text), "Goal: (%.2f, %.2f)", 
                data->mocap_pos[0], data->mocap_pos[1]);
  AddLabel(scene, data->mocap_pos[0], data->mocap_pos[1], 0.5f, goal_text, 0.1f, 1.0f, 0.0f, 0.0f);
}

}  // namespace mjpc
