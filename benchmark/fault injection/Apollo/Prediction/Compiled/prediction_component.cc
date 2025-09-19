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
#include <map>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <limits.h>
#include <algorithm>
#include <unordered_set>
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/prediction/common/prediction_map.h" 
#include "modules/prediction/prediction_component.h"

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/junction_analyzer.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/validation_checker.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/offline_features.pb.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/prediction/scenario/scenario_manager.h"
#include "modules/prediction/util/data_extraction.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::cyber::Clock;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ADCTrajectory;

constexpr double kStaticThreshold = 0.001;
constexpr double kPositionThreshold = 0.00001;

struct waypoint_info {
    double x;
    double y;
    double z;
    double v;
    double a;
    double theta;
    int priority;
};
struct npc_data {
    int id;
    bool is_static;
    std::vector<waypoint_info> waypoints;
};
std::vector<npc_data> npcs;
std::unordered_set<int> has_recorded; 
std::unordered_map<int, int> to_lgsvl;
std::unordered_map<int, int> priority;
std::unordered_map<int, int> interactive;

void read_csv(const std::string& filename) {
    std::string line, word;
    std::ifstream file(filename);

    if (!file.is_open()) {
        AERROR << "file isn't exist!";
    }

    std::getline(file, line);

    std::ofstream out_file("csv_content.txt");
    if (!out_file.is_open()) {
        std::cerr << "Failed to open output file!" << std::endl;
        return;
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        waypoint_info waypoint;
        std::string npc_state;
        std::string npc_id;
        bool is_static;

        std::getline(ss, npc_id, ',');

        std::getline(ss, npc_state, ',');
        is_static = (npc_state == "True");

        std::getline(ss, word, ',');
        waypoint.x = std::stod(word);

        std::getline(ss, word, ',');
        waypoint.y = std::stod(word);

        std::getline(ss, word, ',');
        waypoint.z = std::stod(word);

        std::getline(ss, word, ',');
        waypoint.v = std::stod(word);

        std::getline(ss, word, ',');
        waypoint.a = std::stod(word);

        std::getline(ss, word, ',');
        double theta = std::stod(word);
        if (theta > M_PI)
            theta -= 2 * M_PI;
        waypoint.theta = theta;

        std::getline(ss, word, ',');
        waypoint.priority = std::stoi(word);

        int npc_id_int = std::stoi(npc_id);
        auto it = std::find_if(npcs.begin(), npcs.end(), [npc_id_int](const npc_data& n) {
            return n.id == npc_id_int;
        });

        if (it != npcs.end()) {

            it->waypoints.push_back(waypoint);
        } else {

            npc_data npc;
            npc.id = npc_id_int;
            npc.is_static = is_static;
            npc.waypoints.push_back(waypoint);
            npcs.push_back(npc);
        }
    }

    file.close();
    out_file << "The context of the ideal information is:" << std::endl;

    out_file << "npc_id,is_static,x,y,z,v,a,theta,priority" << std::endl;

    for (const auto& npc : npcs) {
        for (const auto& wp : npc.waypoints) {
            out_file << npc.id << ","
                     << (npc.is_static ? "True" : "False") << ","
                     << std::fixed << std::setprecision(6) << wp.x << ","
                     << wp.y << ","
                     << wp.z << ","
                     << wp.v << ","
                     << wp.a << ","
                     << wp.theta << ","
                     << wp.priority << std::endl;
        }
    }

    out_file.close();
}

PredictionComponent::~PredictionComponent() {}

std::string PredictionComponent::Name() const {
  return FLAGS_prediction_module_name;
}

void PredictionComponent::OfflineProcessFeatureProtoFile(
    const std::string& features_proto_file_name) {
  auto obstacles_container_ptr =
      container_manager_->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  obstacles_container_ptr->Clear();
  Features features;
  apollo::cyber::common::GetProtoFromBinaryFile(features_proto_file_name,
                                                &features);
  for (const Feature& feature : features.feature()) {
    obstacles_container_ptr->InsertFeatureProto(feature);
    Obstacle* obstacle_ptr = obstacles_container_ptr->GetObstacle(feature.id());
    evaluator_manager_->EvaluateObstacle(obstacle_ptr, obstacles_container_ptr);
  }
}

bool PredictionComponent::Init() {
  component_start_time_ = Clock::NowInSeconds();

  container_manager_ = std::make_shared<ContainerManager>();
  evaluator_manager_.reset(new EvaluatorManager());
  predictor_manager_.reset(new PredictorManager());
  scenario_manager_.reset(new ScenarioManager());

  PredictionConf prediction_conf;
  if (!ComponentBase::GetProtoConfig(&prediction_conf)) {
    AERROR << "Unable to load prediction conf file: "
           << ComponentBase::ConfigFilePath();
    return false;
  }
  ADEBUG << "Prediction config file is loaded into: "
         << prediction_conf.ShortDebugString();

  if (!MessageProcess::Init(container_manager_.get(), evaluator_manager_.get(),
                            predictor_manager_.get(), prediction_conf)) {
    return false;
  }
  result_fault_injection_ = prediction_conf.pattern_conf().result_fault_injection();
  if (result_fault_injection_)
  {

    path_point_offset_ = prediction_conf.pattern_conf().path_point_offset();
    theta_offset_ = prediction_conf.pattern_conf().theta_offset();
    v_offset_ = prediction_conf.pattern_conf().v_offset();
    a_offset_ = prediction_conf.pattern_conf().a_offset();

    priority_offset_ = prediction_conf.pattern_conf().priority_offset();

    interaction_offset_ = prediction_conf.pattern_conf().interaction_offset();
  }

  use_ideal_module_data_ = prediction_conf.pattern_conf().use_ideal_module_data();
  use_ideal_component_data_ = prediction_conf.pattern_conf().use_ideal_component_data();
  if (use_ideal_component_data_)
  {
    use_priority_component_ = prediction_conf.pattern_conf().use_priority_component();
    use_interaction_component_ = prediction_conf.pattern_conf().use_interaction_component();
    use_trajectory_component_ = prediction_conf.pattern_conf().use_trajectory_component();
  }

  if (use_ideal_module_data_ || use_ideal_component_data_)
    read_csv("npcs_data.csv");

  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      prediction_conf.topic_conf().planning_trajectory_topic(), nullptr);

  localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          prediction_conf.topic_conf().localization_topic(), nullptr);

  storytelling_reader_ = node_->CreateReader<storytelling::Stories>(
      prediction_conf.topic_conf().storytelling_topic(), nullptr);

  prediction_writer_ = node_->CreateWriter<PredictionObstacles>(
      prediction_conf.topic_conf().prediction_topic());

  container_writer_ = node_->CreateWriter<SubmoduleOutput>(
      prediction_conf.topic_conf().container_topic_name());

  adc_container_writer_ = node_->CreateWriter<ADCTrajectoryContainer>(
      prediction_conf.topic_conf().adccontainer_topic_name());

  perception_obstacles_writer_ = node_->CreateWriter<PerceptionObstacles>(
      prediction_conf.topic_conf().perception_obstacles_topic_name());

  return true;
}

bool PredictionComponent::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  if (FLAGS_use_lego) {
    return ContainerSubmoduleProcess(perception_obstacles);
  }
  return PredictionEndToEndProc(perception_obstacles);
}

bool PredictionComponent::ContainerSubmoduleProcess(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  constexpr static size_t kHistorySize = 10;
  const auto frame_start_time = Clock::Now();
  // Read localization info. and call OnLocalization to update
  // the PoseContainer.
  localization_reader_->Observe();
  auto ptr_localization_msg = localization_reader_->GetLatestObserved();
  if (ptr_localization_msg == nullptr) {
    AERROR << "Prediction: cannot receive any localization message.";
    return false;
  }
  MessageProcess::OnLocalization(container_manager_.get(),
                                 *ptr_localization_msg);

  // Read planning info. of last frame and call OnPlanning to update
  // the ADCTrajectoryContainer
  planning_reader_->Observe();
  auto ptr_trajectory_msg = planning_reader_->GetLatestObserved();
  if (ptr_trajectory_msg != nullptr) {
    MessageProcess::OnPlanning(container_manager_.get(), *ptr_trajectory_msg);
  }

  // Read storytelling message and call OnStorytelling to update the
  // StoryTellingContainer
  storytelling_reader_->Observe();
  auto ptr_storytelling_msg = storytelling_reader_->GetLatestObserved();
  if (ptr_storytelling_msg != nullptr) {
    MessageProcess::OnStoryTelling(container_manager_.get(),
                                   *ptr_storytelling_msg);
  }

  MessageProcess::ContainerProcess(container_manager_, *perception_obstacles,
                                   scenario_manager_.get());

  auto obstacles_container_ptr =
      container_manager_->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK_NOTNULL(obstacles_container_ptr);

  auto adc_trajectory_container_ptr =
      container_manager_->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK_NOTNULL(adc_trajectory_container_ptr);

  SubmoduleOutput submodule_output =
      obstacles_container_ptr->GetSubmoduleOutput(kHistorySize,
                                                  frame_start_time);
  submodule_output.set_curr_scenario(scenario_manager_->scenario());
  container_writer_->Write(submodule_output);
  adc_container_writer_->Write(*adc_trajectory_container_ptr);
  perception_obstacles_writer_->Write(*perception_obstacles);
  return true;
}

bool PredictionComponent::PredictionEndToEndProc(
    const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
  std::ofstream out_file("csv_content.txt", std::ios::app);
  out_file << "Start PredictionEndToEndProc!" << std::endl;
  if (FLAGS_prediction_test_mode &&
      (Clock::NowInSeconds() - component_start_time_ >
       FLAGS_prediction_test_duration)) {
    ADEBUG << "Prediction finished running in test mode";
  }

  // Update relative map if needed
  if (FLAGS_use_navigation_mode && !PredictionMap::Ready()) {
    AERROR << "Relative map is empty.";
    return false;
  }

  frame_start_time_ = Clock::NowInSeconds();
  auto perception_msg = *perception_obstacles;

  // File_location: apollo/modules/prediction/proto/prediction_obstacle.proto
  PredictionObstacles prediction_obstacles;
  if (use_ideal_module_data_)
  {

    if (npcs.size() == 0)
        out_file << "WRONG" << std::endl;
    else
        out_file << "OK" << std::endl;

    for (auto & perception_obstacle : perception_obstacles->perception_obstacle())
    {
        out_file << "check" << std::endl;

        PredictionObstacle prediction_obstacle;

        prediction_obstacle.set_timestamp(perception_obstacle.timestamp());
        out_file << perception_obstacle.timestamp() << std::endl;
        int id = perception_obstacle.id();
        double position_x = perception_obstacle.position().x();
        double position_y = perception_obstacle.position().y();
        double position_z = perception_obstacle.position().z();
        double position_theta = perception_obstacle.theta();

        if (position_theta > M_PI)
            position_theta -= 2 * M_PI;
        out_file << position_x << std::endl;

        double position_v = std::sqrt(std::pow(perception_obstacle.velocity().x(), 2)  + std::pow(perception_obstacle.velocity().y(), 2));

        double position_a = std::sqrt(std::pow(perception_obstacle.acceleration().x(), 2)  + std::pow(perception_obstacle.acceleration().y(), 2));

        if (to_lgsvl.find(id) == to_lgsvl.end())
        {
            out_file << "Start find!" << std::endl;
            for (size_t i = 0; i < npcs.size(); i++)
            {
                double position_lgsvl_x = npcs[i].waypoints[0].x;
                double position_lgsvl_y = npcs[i].waypoints[0].y;
                double pos_diff = std::sqrt(std::pow((position_x - position_lgsvl_x), 2)  + std::pow((position_y - position_lgsvl_y), 2));
                out_file << pos_diff << std::endl;
                if (pos_diff <= kPositionThreshold)
                {

                    to_lgsvl[id] = i;
                    out_file << "Find it!" << i << std::endl;
                    break;
                }
            }
        }

        bool stay_static = npcs[to_lgsvl[id]].is_static;

        if(stay_static)
        {
            prediction_obstacle.set_is_static(true);
            auto static_waypoints = npcs[to_lgsvl[id]].waypoints;
            int static_priority = static_waypoints[0].priority;

            switch (static_priority)
            {
                case 0:
                    prediction_obstacle.mutable_priority()->set_priority(ObstaclePriority::IGNORE);
                    prediction_obstacle.mutable_interactive_tag()->set_interactive_tag(ObstacleInteractiveTag::NONINTERACTION);
                    break;

                case 1:
                    prediction_obstacle.mutable_priority()->set_priority(ObstaclePriority::NORMAL);
                    prediction_obstacle.mutable_interactive_tag()->set_interactive_tag(ObstacleInteractiveTag::NONINTERACTION);
                    break;

                default:
                    AERROR << "Unknown static priority: " << static_priority;
                    break;
            }
        }

        else
        {
            out_file << "not static!" << std::endl;

            auto npc_waypoints = npcs[to_lgsvl[id]].waypoints;
            double min_diff = std::numeric_limits<double>::infinity();
            size_t best_index = 0;

            for (size_t i = 0; i < npc_waypoints.size(); i++)
            {
                auto waypoint = npc_waypoints[i];
                double pos_diff = std::sqrt(std::pow((position_x - waypoint.x), 2) + std::pow((position_y - waypoint.y), 2));
                if (pos_diff < min_diff)
                {
                    min_diff = pos_diff;
                    best_index = i;
                }
            }
            auto npc_waypoint = npc_waypoints[best_index];

            double velocity = std::sqrt(std::pow(perception_obstacle.velocity().x(), 2)  + std::pow(perception_obstacle.velocity().y(), 2));
            double acceleration = std::sqrt(std::pow(perception_obstacle.acceleration().x(), 2)  + std::pow(perception_obstacle.acceleration().y(), 2));
            out_file << "velocity:" << velocity << acceleration << std::endl;

            if(velocity < kStaticThreshold && acceleration < kStaticThreshold)
                prediction_obstacle.set_is_static(true);
            else
                prediction_obstacle.set_is_static(false);

            if (npc_waypoint.priority == 0 || prediction_obstacle.is_static())
            {
                if (npc_waypoint.priority == 0)
                {
                    prediction_obstacle.mutable_priority()->set_priority(ObstaclePriority::IGNORE);
                    prediction_obstacle.mutable_interactive_tag()->set_interactive_tag(ObstacleInteractiveTag::NONINTERACTION);
                }
                else
                {
                    int static_priority = npc_waypoint.priority;
                    switch (static_priority)
                    {
                        case 1:
                            prediction_obstacle.mutable_priority()->set_priority(ObstaclePriority::NORMAL);
                            prediction_obstacle.mutable_interactive_tag()->set_interactive_tag (ObstacleInteractiveTag::NONINTERACTION);
                            break;

                        case 2:
                            prediction_obstacle.mutable_priority()->set_priority(ObstaclePriority::CAUTION);
                            prediction_obstacle.mutable_interactive_tag()->set_interactive_tag    (ObstacleInteractiveTag::INTERACTION);
                            break;

                        default:
                            AERROR << "Unknown static priority: " << static_priority;
                            break;
                    }
                    
                }
            }

            else
            {
                out_file << "Creat Trajectory!" << std::endl;
                // File_location: apollo/modules/prediction/proto/feature.proto
                auto* trajectory = prediction_obstacle.add_trajectory();
                trajectory->set_probability(1.0);

                double relative_time = 0.0;
                for (size_t index = best_index; index < npc_waypoints.size(); index ++)
                {
                    std::string lane_id = "";
                    npc_waypoint = npc_waypoints[index];

                    apollo::common::PointENU position_enu;
                    position_enu.set_x(npc_waypoint.x);
                    position_enu.set_y(npc_waypoint.y);
                    double radius = 1.0;
                    double heading = npc_waypoint.theta;
                    double angle_diff_threshold = M_PI / 3.0;

                    auto lane_info_ptr = PredictionMap::GetMostLikelyCurrentLane(position_enu, radius, heading,  angle_diff_threshold);
                    if(lane_info_ptr != nullptr)
                        lane_id = lane_info_ptr->id().id();
                    out_file << lane_id << std::endl;

                    // File_location: apollo/modules/common/proto/pnc_point.proto
                    auto* trajectory_point = trajectory->add_trajectory_point();
                    trajectory_point->mutable_path_point()->set_lane_id(lane_id);

                    if (index == best_index)
                    {
                        /*
                        double pos_diff = std::sqrt(std::pow((position_x - npc_waypoint.x), 2) + std::pow((position_y - npc_waypoint.y), 2));
                        double v_diff = abs(velocity - npc_waypoint.v);

                        if (v_diff > 0.0)
                            relative_time = pos_diff / v_diff;
                        else
                            relative_time = 0.0;
                        */

                        trajectory_point->mutable_path_point()->set_x(position_x);
                        trajectory_point->mutable_path_point()->set_y(position_y);
                        trajectory_point->mutable_path_point()->set_z(position_z);
                        trajectory_point->mutable_path_point()->set_theta(position_theta);

                        trajectory_point->set_v(position_v);
                        trajectory_point->set_a(position_a);
                    }
                    else
                    {
                        relative_time += 0.1;

                        trajectory_point->mutable_path_point()->set_x(npc_waypoint.x);
                        trajectory_point->mutable_path_point()->set_y(npc_waypoint.y);
                        trajectory_point->mutable_path_point()->set_z(npc_waypoint.z);
                        trajectory_point->mutable_path_point()->set_theta(npc_waypoint.theta);

                        trajectory_point->set_v(npc_waypoint.v);
                        trajectory_point->set_a(npc_waypoint.a);
                    }
                    trajectory_point->set_relative_time(relative_time);
                }

                switch (npc_waypoint.priority)
                {
                    case 1:
                        prediction_obstacle.mutable_priority()->set_priority(ObstaclePriority::NORMAL);
                        prediction_obstacle.mutable_interactive_tag()->set_interactive_tag(ObstacleInteractiveTag::NONINTERACTION);
                        break;

                    case 2:
                        prediction_obstacle.mutable_priority()->set_priority(ObstaclePriority::CAUTION);
                        prediction_obstacle.mutable_interactive_tag()->set_interactive_tag(ObstacleInteractiveTag::INTERACTION);
                        break;

                    default:
                        AERROR << "Unknown static priority: " << npc_waypoint.priority;
                        break;
                }
            }
        }

        prediction_obstacle.set_predicted_period(FLAGS_prediction_trajectory_time_length);
        prediction_obstacle.mutable_perception_obstacle()->CopyFrom(perception_obstacle);
        prediction_obstacles.add_prediction_obstacle()->CopyFrom(prediction_obstacle);
    }
    prediction_obstacles.set_start_timestamp(frame_start_time_);
    prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());
    prediction_obstacles.mutable_header()->set_lidar_timestamp(perception_msg.header().lidar_timestamp());
    prediction_obstacles.mutable_header()->set_camera_timestamp(perception_msg.header().camera_timestamp());
    prediction_obstacles.mutable_header()->set_radar_timestamp(perception_msg.header().radar_timestamp());
    prediction_obstacles.set_perception_error_code(perception_msg.error_code());
    
  }

  else
  {
  auto end_time1 = std::chrono::system_clock::now();

  // Read localization info. and call OnLocalization to update
  // the PoseContainer.
  localization_reader_->Observe();
  auto ptr_localization_msg = localization_reader_->GetLatestObserved();
  if (ptr_localization_msg == nullptr) {
    AERROR << "Prediction: cannot receive any localization message.";
    return false;
  }
  MessageProcess::OnLocalization(container_manager_.get(),
                                 *ptr_localization_msg);
  auto end_time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time2 - end_time1;
  ADEBUG << "Time for updating PoseContainer: " << diff.count() * 1000
         << " msec.";

  // Read storytelling message and call OnStorytelling to update the
  // StoryTellingContainer
  storytelling_reader_->Observe();
  auto ptr_storytelling_msg = storytelling_reader_->GetLatestObserved();
  if (ptr_storytelling_msg != nullptr) {
    MessageProcess::OnStoryTelling(container_manager_.get(),
                                   *ptr_storytelling_msg);
  }

  // Read planning info. of last frame and call OnPlanning to update
  // the ADCTrajectoryContainer
  planning_reader_->Observe();
  auto ptr_trajectory_msg = planning_reader_->GetLatestObserved();
  if (ptr_trajectory_msg != nullptr) {
    MessageProcess::OnPlanning(container_manager_.get(), *ptr_trajectory_msg);
  }
  auto end_time3 = std::chrono::system_clock::now();
  diff = end_time3 - end_time2;
  ADEBUG << "Time for updating ADCTrajectoryContainer: " << diff.count() * 1000
         << " msec.";

  if (use_ideal_component_)
  {
    out_file << "Start use grade component" << std::endl;
    for (auto & perception_obstacle : perception_obstacles->perception_obstacle())
    {
        out_file << "Traverse obstacles in perception!" << std::endl;

        int id = perception_obstacle.id();
        double position_x = perception_obstacle.position().x();
        double position_y = perception_obstacle.position().y();

        if (has_recorded.find(id) == has_recorded.end())
        {
            out_file << "Start find matched id!" << std::endl;
            for (size_t i = 0; i < npcs.size(); i++)
            {
                double position_lgsvl_x = npcs[i].waypoints[0].x;
                double position_lgsvl_y = npcs[i].waypoints[0].y;
                double pos_diff = std::sqrt(std::pow((position_x - position_lgsvl_x), 2)  + std::pow((position_y - position_lgsvl_y), 2));

                if (pos_diff <= kPositionThreshold)
                {

                    has_recorded.insert(id);
                    out_file << "Find " << i << "matched with " << id << std::endl;

                    bool stay_static = npcs[i].is_static;

                    if (stay_static)
                    {
                        interactive[id] = 0;
                        auto static_waypoint = npcs[i].waypoints[0];
                        priority[id] = static_waypoint.priority;
                    }
                    else
                    {
                        size_t best_index = 0;
                        auto npc_waypoints = npcs[i].waypoints;
                        double min_diff = std::numeric_limits<double>::infinity();

                        for (size_t i = 0; i < npc_waypoints.size(); i++)
                        {
                            auto waypoint = npc_waypoints[i];
                            double pos_diff = std::sqrt(std::pow((position_x - waypoint.x), 2) + std::pow((position_y - waypoint.y), 2));
                            if (pos_diff < min_diff)
                            {
                                min_diff = pos_diff;
                                best_index = i;
                            }
                        }
                        auto npc_waypoint = npc_waypoints[best_index];

                        double velocity = std::sqrt(std::pow(perception_obstacle.velocity().x(), 2)  + std::pow(perception_obstacle.velocity().y(), 2));
                        double acceleration = std::sqrt(std::pow(perception_obstacle.acceleration().x(), 2)  + std::pow(perception_obstacle.acceleration().y(), 2));
                        out_file << "velocity is :" << velocity << "and acceleration is :" << acceleration << std::endl;
                        auto npc_waypoint_priority = npc_waypoint.priority;
                        if(velocity < kStaticThreshold && acceleration < kStaticThreshold)
                        {
                            interactive[id] = 0;
                            priority[id] = npc_waypoint_priority;
                        }
                        else
                        {
                            if (npc_waypoint_priority != 2)
                                interactive[id] = 0;
                            else
                                interactive[id] = 1;
                            priority[id] = npc_waypoint_priority;
                        }
                    }
                    out_file << "Priority: " << priority[id] << " Interactive: " << interactive[id] << std::endl;
                    break;
                }
            }
        }
        else
            out_file << "The obstacle has been recorded!" << std::endl;

    }

    MessageProcess::OnPerception(
      perception_msg, container_manager_, evaluator_manager_.get(),
      predictor_manager_.get(), scenario_manager_.get(), &prediction_obstacles, interactive, priority);
  }
  else
  {
    MessageProcess::OnPerception(
      perception_msg, container_manager_, evaluator_manager_.get(),
      predictor_manager_.get(), scenario_manager_.get(), &prediction_obstacles);
  }
  auto end_time4 = std::chrono::system_clock::now();
  diff = end_time4 - end_time3;
  ADEBUG << "Time for updating PerceptionContainer: " << diff.count() * 1000
         << " msec.";

  // Postprocess prediction obstacles message
  prediction_obstacles.set_start_timestamp(frame_start_time_);
  prediction_obstacles.set_end_timestamp(Clock::NowInSeconds());
  prediction_obstacles.mutable_header()->set_lidar_timestamp(
      perception_msg.header().lidar_timestamp());
  prediction_obstacles.mutable_header()->set_camera_timestamp(
      perception_msg.header().camera_timestamp());
  prediction_obstacles.mutable_header()->set_radar_timestamp(
      perception_msg.header().radar_timestamp());

  prediction_obstacles.set_perception_error_code(perception_msg.error_code());

  if (FLAGS_prediction_test_mode) {
    for (auto const& prediction_obstacle :
         prediction_obstacles.prediction_obstacle()) {
      for (auto const& trajectory : prediction_obstacle.trajectory()) {
        for (auto const& trajectory_point : trajectory.trajectory_point()) {
          if (!ValidationChecker::ValidTrajectoryPoint(trajectory_point)) {
            AERROR << "Invalid trajectory point ["
                   << trajectory_point.ShortDebugString() << "]";
            break;
          }
        }
      }
    }
  }

  out_file << "_________________________________________________" << std::endl;
  auto end_time5 = std::chrono::system_clock::now();
  diff = end_time5 - end_time1;
  ADEBUG << "End to end time elapsed: " << diff.count() * 1000 << " msec.";
  }

  if (result_fault_injection_)
  {
    for (int i = 0; i < prediction_obstacles.prediction_obstacle_size(); ++i)
    {

        /*
        message PredictionObstacles
        {
            optional apollo.common.Header header = 1;
            repeated PredictionObstacle prediction_obstacle = 2;
        }
        */
        PredictionObstacle* prediction_obstacle_ = prediction_obstacles.mutable_prediction_obstacle(i);
        for (int j = 0; j < prediction_obstacle_->trajectory_size(); ++j)
        {
            /*
            message PredictionObstacle
            {
                optional apollo.perception.PerceptionObstacle perception_obstacle = 1;
                optional double timestamp = 2;
                optional double predicted_period = 3;
                repeated Trajectory trajectory = 4;
            }
            */
            Trajectory* trajectory_point_ = prediction_obstacle_->mutable_trajectory(j);
            /*
            message Trajectory
            {
                optional double probability = 1;  // probability of this trajectory
                repeated apollo.common.TrajectoryPoint trajectory_point = 2;
            }
            */
            for (int k = 0; k < trajectory_point_->trajectory_point_size(); ++k)
            {
                apollo::common::TrajectoryPoint* trajectory_point = trajectory_point_->mutable_trajectory_point(k);
                if (path_point_offset_ != 0.0)
                {
                    double theta = trajectory_point->path_point().theta();
                    trajectory_point->mutable_path_point()->set_x(trajectory_point->path_point().x() +  path_point_offset_ * std::cos(theta));
                    trajectory_point->mutable_path_point()->set_y(trajectory_point->path_point().y() +  path_point_offset_ * std::sin(theta));
                }
                if (theta_offset_ != 0.0)
                {
                    double theta = fmod(trajectory_point->path_point().theta() + theta_offset_, 2 * M_PI);
                    if (theta > M_PI)
                        theta -= 2 * M_PI;
                    else if (theta < -M_PI)
                        theta += 2 * M_PI;
                    trajectory_point->mutable_path_point()->set_theta(theta);
                }
                if (v_offset_ != 0.0)
                {
                    trajectory_point->set_v(trajectory_point->v() + v_offset_);
                }
                if (a_offset_ != 0.0)
                {
                    trajectory_point->set_a(trajectory_point->a() + a_offset_);
                }
            }
        }
    }
  }

  // Publish output
  common::util::FillHeader(node_->Name(), &prediction_obstacles);
  prediction_writer_->Write(prediction_obstacles);
  return true;
}

}  // namespace prediction
}  // namespace apollo
