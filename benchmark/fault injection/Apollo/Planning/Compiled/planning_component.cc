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
#include "modules/planning/planning_component.h"
#include<cmath>
#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/navi_planning.h"
#include "modules/planning/on_lane_planning.h"

namespace apollo {
namespace planning {

using apollo::cyber::ComponentBase;
using apollo::hdmap::HDMapUtil;
using apollo::perception::TrafficLightDetection;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::storytelling::Stories;

bool PlanningComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();

  if (FLAGS_use_navigation_mode) {
    planning_base_ = std::make_unique<NaviPlanning>(injector_);
  } else {
    planning_base_ = std::make_unique<OnLanePlanning>(injector_);
  }

  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load planning config file "
      << ComponentBase::ConfigFilePath();

  if (FLAGS_planning_offline_learning ||
      config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    if (!message_process_.Init(config_, injector_)) {
      AERROR << "failed to init MessageProcess";
      return false;
    }
  }

  result_fault_injection_ = config_.result_fault_injection();
  if (result_fault_injection_)
  {
    theta_offset_ = config_.theta_offset();
    v_offset_ = config_.v_offset();
    a_offset_ = config_.a_offset();
  }
  use_ideal_module_data_ = config_.use_ideal_module_data();
  if (use_ideal_module_data_)
  {
    trajectory_writer_ = node_->CreateWriter<apollo::contrib::lgsvl_msgs::Detection2DArray>("/apollo/simulation/trajectory_result");
  }
  planning_base_->Init(config_);

  routing_reader_ = node_->CreateReader<RoutingResponse>(
      config_.topic_config().routing_response_topic(),
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        AINFO << "Received routing data: run routing callback."
              << routing->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });

  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      config_.topic_config().traffic_light_detection_topic(),
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      config_.topic_config().planning_pad_topic(),
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  story_telling_reader_ = node_->CreateReader<Stories>(
      config_.topic_config().story_telling_topic(),
      [this](const std::shared_ptr<Stories>& stories) {
        ADEBUG << "Received story_telling data: run story_telling callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        stories_.CopyFrom(*stories);
      });

  if (FLAGS_use_navigation_mode) {
    relative_map_reader_ = node_->CreateReader<MapMsg>(
        config_.topic_config().relative_map_topic(),
        [this](const std::shared_ptr<MapMsg>& map_message) {
          ADEBUG << "Received relative map data: run relative map callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          relative_map_.CopyFrom(*map_message);
        });
  }
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());

  rerouting_writer_ = node_->CreateWriter<RoutingRequest>(
      config_.topic_config().routing_request_topic());

  planning_learning_data_writer_ = node_->CreateWriter<PlanningLearningData>(
      config_.topic_config().planning_learning_data_topic());

  return true;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  ACHECK(prediction_obstacles != nullptr);

  // check and process possible rerouting request
  CheckRerouting();

  // process fused input data
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;
  double localization_timestamp = localization_estimate->measurement_time();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.routing ||
        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
      local_view_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.stories = std::make_shared<Stories>(stories_);
  }

  if (!CheckInput()) {
    AERROR << "Input check failed";
    return false;
  }

  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    // data process for online training
    message_process_.OnChassis(*local_view_.chassis);
    message_process_.OnPrediction(*local_view_.prediction_obstacles);
    message_process_.OnRoutingResponse(*local_view_.routing);
    message_process_.OnStoryTelling(*local_view_.stories);
    message_process_.OnTrafficLightDetection(*local_view_.traffic_light);
    message_process_.OnLocalization(*local_view_.localization_estimate);
  }

  // publish learning data frame for RL test
  if (config_.learning_mode() == PlanningConfig::RL_TEST) {
    PlanningLearningData planning_learning_data;
    LearningDataFrame* learning_data_frame =
        injector_->learning_based_data()->GetLatestLearningDataFrame();
    if (learning_data_frame) {
      planning_learning_data.mutable_learning_data_frame()
                            ->CopyFrom(*learning_data_frame);
      common::util::FillHeader(node_->Name(), &planning_learning_data);
      planning_learning_data_writer_->Write(planning_learning_data);
    } else {
      AERROR << "fail to generate learning data frame";
      return false;
    }
    return true;
  }

  ADCTrajectory adc_trajectory_pb;
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  // modify trajectory relative time due to the timestamp change in header
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  int trajectory_length = 50;
  double pre_timestamp = -1.0;
  double pre_position_x = -1.0;
  double pre_position_y = -1.0;
  apollo::contrib::lgsvl_msgs::Detection2DArray trajectory_point_array;
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
    if (result_fault_injection_)
    {
      if (theta_offset_ != 0.0)
      {
        double new_theta = p.mutable_path_point()->theta() + theta_offset_;
        new_theta = std::max(-3.1415926, new_theta);
        new_theta = std::min(new_theta, 3.1415926);
        if (pre_position_x == -1.0 && pre_position_y == -1.0)
        {
          pre_position_x = p.mutable_path_point()->x();
          pre_position_y = p.mutable_path_point()->y();
        }
        else
        {
          double x_diff = p.mutable_path_point()->x() - pre_position_x;
          double y_diff = p.mutable_path_point()->y() - pre_position_y;
          double distance = std::sqrt(x_diff * x_diff + y_diff * y_diff);
          p.mutable_path_point()->set_x(p.mutable_path_point()->x() + distance * (std::cos(new_theta) - std::cos(p.mutable_path_point()->theta())));
          p.mutable_path_point()->set_y(p.mutable_path_point()->y() + distance * (std::sin(new_theta) - std::sin(p.mutable_path_point()->theta())));
        }
        p.mutable_path_point()->set_theta(new_theta);
      }
      if (pre_timestamp == -1.0)
        pre_timestamp = p.relative_time();
      else
      {
        double time_dif = p.relative_time() - pre_timestamp;
        pre_timestamp = p.relative_time();
        if (v_offset_ != 0.0)
        {
          p.set_v(p.v() + v_offset_);
          p.mutable_path_point()->set_s(p.mutable_path_point()->s() + v_offset_ * time_dif);
          p.mutable_path_point()->set_x(p.mutable_path_point()->x() + v_offset_ * time_dif * std::cos(p.mutable_path_point()->theta()));
          p.mutable_path_point()->set_y(p.mutable_path_point()->y() + v_offset_ * time_dif * std::sin(p.mutable_path_point()->theta()));
        }
        if (a_offset_ != 0.0)
        {
          p.set_a(p.a() + a_offset_);
          double offset = a_offset_ * time_dif;
          p.mutable_path_point()->set_s(p.mutable_path_point()->s() + offset * time_dif);
          p.mutable_path_point()->set_x(p.mutable_path_point()->x() + offset * time_dif * std::cos(p.mutable_path_point()->theta()));
          p.mutable_path_point()->set_y(p.mutable_path_point()->y() + offset * time_dif * std::sin(p.mutable_path_point()->theta()));
        }
      }
    }
    if (use_ideal_module_data_)
    {
      if (trajectory_length && p.relative_time() > 0.0)
      {
        id_ += 1;
        apollo::contrib::lgsvl_msgs::Detection2D* trajectory_point = trajectory_point_array.add_detections();
        trajectory_point->set_id(id_);
        trajectory_point->set_label("Ponit");
        trajectory_point->set_score(p.relative_time());
        trajectory_point->mutable_bbox()->set_x(p.path_point().x());
        trajectory_point->mutable_bbox()->set_y(p.path_point().y());
        trajectory_point->mutable_bbox()->set_width(::apollo::cyber::Clock::NowInSeconds());
        trajectory_point->mutable_bbox()->set_height(dt);
        trajectory_point->mutable_velocity()->mutable_linear()->set_x(p.path_point().theta());
        trajectory_point->mutable_velocity()->mutable_linear()->set_y(p.v());
        trajectory_point->mutable_velocity()->mutable_linear()->set_z(p.a());
        trajectory_point->mutable_velocity()->mutable_angular()->CopyFrom(trajectory_point->velocity().linear());
        trajectory_length -= 1;
      }
    }
  }
  uint64_t radar_timestamp_ns = static_cast<uint64_t>(localization_timestamp * 1000000000);
  adc_trajectory_pb.mutable_header()->set_radar_timestamp(radar_timestamp_ns);
  planning_writer_->Write(adc_trajectory_pb);

  // record in history
  auto* history = injector_->history();
  history->Add(adc_trajectory_pb);

  return true;
}

void PlanningComponent::CheckRerouting() {
  auto* rerouting = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_rerouting();
  if (!rerouting->need_rerouting()) {
    return;
  }
  common::util::FillHeader(node_->Name(), rerouting->mutable_routing_request());
  rerouting->set_need_rerouting(false);
  rerouting_writer_->Write(rerouting->routing_request());
}

bool PlanningComponent::CheckInput() {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  } else {
    // nothing
  }

  if (FLAGS_use_navigation_mode) {
    if (!local_view_.relative_map->has_header()) {
      not_ready->set_reason("relative map not ready");
    }
  } else {
    if (!local_view_.routing->has_header()) {
      not_ready->set_reason("routing not ready");
    }
  }

  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
