/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/rtk/rtk_localization_component.h"
#include "modules/common/math/quaternion.h"
#include "cyber/time/clock.h"
#include <cmath>

namespace apollo {
namespace localization {

RTKLocalizationComponent::RTKLocalizationComponent()
    : localization_(new RTKLocalization()) {}

bool RTKLocalizationComponent::Init() {
  tf2_broadcaster_.reset(new apollo::transform::TransformBroadcaster(node_));
  if (!InitConfig()) {
    AERROR << "Init Config falseed.";
    return false;
  }

  if (!InitIO()) {
    AERROR << "Init Interval falseed.";
    return false;
  }

  return true;
}

bool RTKLocalizationComponent::InitConfig() {
  rtk_config::Config rtk_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &rtk_config)) {
    return false;
  }
  AINFO << "Rtk localization config: " << rtk_config.DebugString();

  localization_topic_ = rtk_config.localization_topic();
  localization_gt_topic_ = rtk_config.localization_gt_topic();
  localization_status_topic_ = rtk_config.localization_status_topic();
  imu_topic_ = rtk_config.imu_topic();
  gps_topic_ = rtk_config.gps_topic();
  gps_status_topic_ = rtk_config.gps_status_topic();
  broadcast_tf_frame_id_ = rtk_config.broadcast_tf_frame_id();
  broadcast_tf_child_frame_id_ = rtk_config.broadcast_tf_child_frame_id();
  result_fault_injection_ = rtk_config.result_fault_injection();
  if (result_fault_injection_)
  {
    position_offset_ = rtk_config.position_offset();
    orientation_offset_ = rtk_config.orientation_offset();
    linear_velocity_offset_ = rtk_config.linear_velocity_offset();
    linear_acceleration_offset_ = rtk_config.linear_acceleration_offset();
    angular_velocity_offset_ = rtk_config.angular_velocity_offset();
    euler_angles_offset_ = rtk_config.euler_angles_offset();
  }
  use_ideal_module_data_ = rtk_config.use_ideal_module_data(); 
  use_ideal_component_data_ = rtk_config.use_ideal_component_data();
  if (use_ideal_component_data_)
  {
    use_gps_component_ = rtk_config.use_gps_component();
    use_imu_component_ = rtk_config.use_imu_component();
    use_corrected_imu_component_ = rtk_config.use_corrected_imu_component();
  }
  localization_->InitConfig(rtk_config);

  return true;
}

bool RTKLocalizationComponent::InitIO() {
  if (!use_ideal_module_data_)
  {
    corrected_imu_listener_ = node_->CreateReader<localization::CorrectedImu>(
      imu_topic_, std::bind(&RTKLocalization::ImuCallback, localization_.get(),
                            std::placeholders::_1));
    ACHECK(corrected_imu_listener_);

    gps_status_listener_ = node_->CreateReader<drivers::gnss::InsStat>(
      gps_status_topic_, std::bind(&RTKLocalization::GpsStatusCallback,
                                   localization_.get(), std::placeholders::_1));
    ACHECK(gps_status_listener_);

    localization_gt_listener_ =  node_->CreateReader<LocalizationEstimate>(localization_gt_topic_, nullptr);
  }

  localization_talker_ =
      node_->CreateWriter<LocalizationEstimate>(localization_topic_);
  ACHECK(localization_talker_);

  localization_status_talker_ =
      node_->CreateWriter<LocalizationStatus>(localization_status_topic_);
  ACHECK(localization_status_talker_);
  return true;
}

bool RTKLocalizationComponent::Proc(
    const std::shared_ptr<localization::Gps>& gps_msg) {
      
  if (use_ideal_module_data_)
    return true;
  localization_->GpsCallback(gps_msg);
  LocalizationEstimate localization_gt;
  if (use_ideal_component_data_)
  {
    localization_gt_listener_->Observe();
    localization_gt = *localization_gt_listener_->GetLatestObserved();
  }
  if (localization_->IsServiceStarted()) {
    LocalizationEstimate localization;
    localization_->GetLocalization(&localization);
    LocalizationStatus localization_status;
    localization_->GetLocalizationStatus(&localization_status);

    if (result_fault_injection_)
    {
      auto mutable_pose = localization.mutable_pose();
      if (orientation_offset_ != 0.0)
      {
        double ori_x = mutable_pose->orientation().qx() + orientation_offset_;
        double ori_y = mutable_pose->orientation().qy() + orientation_offset_;
        double ori_z = mutable_pose->orientation().qz() + orientation_offset_;
        double ori_w = mutable_pose->orientation().qw() + orientation_offset_;
        double norm = std::sqrt(ori_x * ori_x + ori_y * ori_y + ori_z * ori_z + ori_w * ori_w);

        mutable_pose->mutable_orientation()->set_qx(ori_x / norm);
        mutable_pose->mutable_orientation()->set_qy(ori_y / norm);
        mutable_pose->mutable_orientation()->set_qz(ori_z / norm);
        mutable_pose->mutable_orientation()->set_qw(ori_w / norm);
        double heading = common::math::QuaternionToHeading(
            mutable_pose->orientation().qw(), mutable_pose->orientation().qx(),
            mutable_pose->orientation().qy(), mutable_pose->orientation().qz());
        mutable_pose->set_heading(heading);
      }
      if (position_offset_ != 0.0)
      {
        double heading = mutable_pose->heading();
        mutable_pose->mutable_position()->set_x(mutable_pose->position().x() + position_offset_ * std::cos(heading));
        mutable_pose->mutable_position()->set_y(mutable_pose->position().y() + position_offset_ * std::sin(heading));
      }
      if (linear_velocity_offset_ != 0.0)
      {
        double heading = mutable_pose->heading();
        mutable_pose->mutable_linear_velocity()->set_x(mutable_pose->linear_velocity().x() + linear_velocity_offset_ * std::cos(heading));
        mutable_pose->mutable_linear_velocity()->set_y(mutable_pose->linear_velocity().y() + linear_velocity_offset_ * std::sin(heading));
      }
      if (linear_acceleration_offset_ != 0.0)
      {
        double heading = mutable_pose->heading();
        mutable_pose->mutable_linear_acceleration()->set_x(mutable_pose->linear_acceleration().x() + linear_acceleration_offset_ * std::cos(heading));
        mutable_pose->mutable_linear_acceleration()->set_y(mutable_pose->linear_acceleration().y() + linear_acceleration_offset_ * std::sin(heading));
      }
      if (angular_velocity_offset_ != 0.0)
      {
        double heading = mutable_pose->heading();
        mutable_pose->mutable_angular_velocity()->set_x(mutable_pose->angular_velocity().x() + angular_velocity_offset_ * std::cos(heading));
        mutable_pose->mutable_angular_velocity()->set_y(mutable_pose->angular_velocity().y() + angular_velocity_offset_ * std::sin(heading));
      }
      if (euler_angles_offset_ != 0.0)
      {
        double heading = mutable_pose->euler_angles().z() + euler_angles_offset_;
        heading = std::max(0.0, heading);
        heading = std::min(heading, 6.283185);
        mutable_pose->mutable_euler_angles()->set_z(heading);
      }
    }
    if (use_ideal_component_data_)
    {
      auto mutable_pose = localization.mutable_pose();
      auto mutable_pose_gt = localization_gt.mutable_pose();
      if (use_gps_component_)
      {
        // position
        if (mutable_pose_gt->has_position()) {
        mutable_pose->mutable_position()->set_x(mutable_pose_gt->position().x());
        mutable_pose->mutable_position()->set_y(mutable_pose_gt->position().y());
        mutable_pose->mutable_position()->set_z(mutable_pose_gt->position().z());
        }

        // orientation
        if (mutable_pose_gt->has_orientation()) {
          mutable_pose->mutable_orientation()->CopyFrom(mutable_pose_gt->orientation());
          mutable_pose->set_heading(mutable_pose_gt->heading());
        }

        // linear velocity
        if (mutable_pose_gt->has_linear_velocity()) {
          mutable_pose->mutable_linear_velocity()->CopyFrom(mutable_pose_gt->linear_velocity());
        }
      }
      if (use_imu_component_)
      {
        // linear acceleration
        if (mutable_pose_gt->has_linear_acceleration()) {
          mutable_pose->mutable_linear_acceleration()->set_x(mutable_pose_gt->linear_acceleration().x());
          mutable_pose->mutable_linear_acceleration()->set_y(mutable_pose_gt->linear_acceleration().y());
          mutable_pose->mutable_linear_acceleration()->set_z(mutable_pose_gt->linear_acceleration().z());

          // linear_acceleration_vfr
          mutable_pose->mutable_linear_acceleration_vrf()->CopyFrom(mutable_pose_gt->linear_acceleration());
        }

        // angular velocity
        if (mutable_pose_gt->has_angular_velocity()) {
        mutable_pose->mutable_angular_velocity()->set_x(mutable_pose_gt->angular_velocity().x());
        mutable_pose->mutable_angular_velocity()->set_y(mutable_pose_gt->angular_velocity().y());
        mutable_pose->mutable_angular_velocity()->set_z(mutable_pose_gt->angular_velocity().z());

        // angular_velocity_vf
        mutable_pose->mutable_angular_velocity_vrf()->CopyFrom(mutable_pose_gt->angular_velocity());
        }
      }
      if (use_corrected_imu_component_)
      {
        // euler angle
        if (mutable_pose_gt->has_euler_angles()) {
          mutable_pose->mutable_euler_angles()->CopyFrom(mutable_pose_gt->euler_angles());
        }
      }
    }

    // publish localization messages
    PublishPoseBroadcastTopic(localization);
    PublishPoseBroadcastTF(localization);
    PublishLocalizationStatus(localization_status);
    ADEBUG << "[OnTimer]: Localization message publish success!";
  }

  return true;
}

void RTKLocalizationComponent::PublishPoseBroadcastTF(
    const LocalizationEstimate& localization) {
  // broadcast tf message
  apollo::transform::TransformStamped tf2_msg;

  auto mutable_head = tf2_msg.mutable_header();
  mutable_head->set_timestamp_sec(localization.measurement_time());
  mutable_head->set_frame_id(broadcast_tf_frame_id_);
  tf2_msg.set_child_frame_id(broadcast_tf_child_frame_id_);

  auto mutable_translation = tf2_msg.mutable_transform()->mutable_translation();
  mutable_translation->set_x(localization.pose().position().x());
  mutable_translation->set_y(localization.pose().position().y());
  mutable_translation->set_z(localization.pose().position().z());

  auto mutable_rotation = tf2_msg.mutable_transform()->mutable_rotation();
  mutable_rotation->set_qx(localization.pose().orientation().qx());
  mutable_rotation->set_qy(localization.pose().orientation().qy());
  mutable_rotation->set_qz(localization.pose().orientation().qz());
  mutable_rotation->set_qw(localization.pose().orientation().qw());

  tf2_broadcaster_->SendTransform(tf2_msg);
}

void RTKLocalizationComponent::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
}

void RTKLocalizationComponent::PublishLocalizationStatus(
    const LocalizationStatus& localization_status) {
  localization_status_talker_->Write(localization_status);
}

}  // namespace localization
}  // namespace apollo
