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

#ifndef MJPC_TASKS_SIMPLE_CAR_SIMPLE_CAR_H_
#define MJPC_TASKS_SIMPLE_CAR_SIMPLE_CAR_H_

#include <string>
#include <memory>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {
class SimpleCar : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public BaseResidualFn {
   public:
    explicit ResidualFn(const SimpleCar* task) : BaseResidualFn(task) {}
    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;
  };

  SimpleCar() : residual_(this) {
    visualize = 1;  // enable visualization
  }

  void TransitionLocked(mjModel* model, mjData* data) override;
  void ModifyScene(const mjModel* model, const mjData* data,
                   mjvScene* scene) const override;

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;
  
  // 仪表盘数据结构
  struct DashboardData {
    double speed_kmh = 0.0;      // 速度 (km/h)
    double rpm = 0.0;            // 转速
    double fuel = 100.0;         // 油量 (%)
    double temperature = 60.0;   // 温度 (°C)
    
    // 模拟数据
    mutable double simulated_fuel = 100.0;
  };
  
  mutable DashboardData dashboard_;
  
  // 辅助函数：更新仪表盘数据
  void UpdateDashboardData(const mjModel* model, const mjData* data) const;
  
  // 2D绘制函数
  void Draw2DRectangle(mjvScene* scene, float x, float y,
                      float width, float height,
                      float r, float g, float b, float a) const;
  void Draw2DLine(mjvScene* scene, float x1, float y1,
                 float x2, float y2, float width,
                 float r, float g, float b, float a) const;
  void Draw2DCircle(mjvScene* scene, float x, float y, float radius,
                   float r, float g, float b, float a) const;
  
  // 仪表盘绘制函数（2D版本）
  void DrawSpeedometer2D(mjvScene* scene, float x, float y, float size) const;
  void DrawTachometer2D(mjvScene* scene, float x, float y, float size) const;
  void DrawFuelGauge2D(mjvScene* scene, float x, float y, float width, float height) const;
  void DrawTemperatureGauge2D(mjvScene* scene, float x, float y, float width, float height) const;
  
  // 添加标签
  void AddLabel(mjvScene* scene, float x, float y, float z, const char* text, 
                float size, float r, float g, float b) const;
};
}  // namespace mjpc

#endif  // MJPC_TASKS_SIMPLE_CAR_SIMPLE_CAR_H_
