/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/component/component.h"
#include "modules/perception/base/object.h"
#include "modules/perception/fusion/lib/interface/base_multisensor_fusion.h"
#include "modules/perception/fusion/lib/interface/base_fusion_system.h"
#include "modules/perception/map/hdmap/hdmap_input.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/onboard/proto/fusion_component_config.pb.h"

namespace apollo {
namespace perception {
namespace onboard {

class FusionComponent : public cyber::Component<SensorFrameMessage> {
 public:
  FusionComponent() = default;
  ~FusionComponent() = default;
  bool Init() override;
  bool Proc(const std::shared_ptr<SensorFrameMessage>& message) override;

 private:
  bool InitAlgorithmPlugin();
  bool search_path(int u);
  int Kuhn_Munkras();
  void CopyObstacleAttributes(const PerceptionObstacle* source, PerceptionObstacle* target);
  bool InternalProc(const std::shared_ptr<SensorFrameMessage const>& in_message,
                    std::shared_ptr<PerceptionObstacles> out_message,
                    std::shared_ptr<SensorFrameMessage> viz_message);

 private:
  static std::mutex s_mutex_;
  static uint32_t s_seq_num_;

  std::string fusion_name_;
  std::string fusion_method_;
  std::string fusion_main_sensor_;
  bool object_in_roi_check_ = false;
  double radius_for_roi_object_check_ = 0;

  int nx = 0;
  int ny = 0;
  int lx[50];
  int ly[50];
  int match[50];
  int slack[50];
  int weight[50][50];
  bool visited_x[50];
  bool visited_y[50];
  const int INF = 0x3f3f3f3f;
  bool result_fault_injection_ = false;
  double position_offset_ = 0.0;
  double dimension_offset_ = 0.0;
  double velocity_offset_ = 0.0;
  double acceleration_offset_ = 0.0;
  double theta_offset_ = 0.0;
  int type_offset_ = 0;
  int sub_type_offset_ = 0;
  bool use_ideal_module_data_ = false;
  bool use_ideal_component_data_ = false;
  bool use_lidar_component_ = false;
  bool use_radar_component_ = false;
  bool use_camera_component_ = false;
  std::unique_ptr<fusion::BaseMultiSensorFusion> fusion_;
  map::HDMapInput* hdmap_input_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;
  std::shared_ptr<apollo::cyber::Writer<SensorFrameMessage>> inner_writer_;
  std::shared_ptr<cyber::Reader<PerceptionObstacles>> obstacles_gt_reader_ =
      nullptr;
};

CYBER_REGISTER_COMPONENT(FusionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
