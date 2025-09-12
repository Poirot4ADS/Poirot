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
#include "cyber/time/clock.h"

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
  localization_status_topic_ = rtk_config.localization_status_topic();
  imu_topic_ = rtk_config.imu_topic();
  gps_topic_ = rtk_config.gps_topic();
  gps_status_topic_ = rtk_config.gps_status_topic();
  broadcast_tf_frame_id_ = rtk_config.broadcast_tf_frame_id();
  broadcast_tf_child_frame_id_ = rtk_config.broadcast_tf_child_frame_id();

  use_pose_gt = rtk_config.use_pose_gt();

  localization_->InitConfig(rtk_config);

  return true;
}

bool RTKLocalizationComponent::InitIO() {
  corrected_imu_listener_ = node_->CreateReader<localization::CorrectedImu>(
      imu_topic_, std::bind(&RTKLocalization::ImuCallback, localization_.get(),
                            std::placeholders::_1));
  ACHECK(corrected_imu_listener_);

  gps_status_listener_ = node_->CreateReader<drivers::gnss::InsStat>(
      gps_status_topic_, std::bind(&RTKLocalization::GpsStatusCallback,
                                   localization_.get(), std::placeholders::_1));
  ACHECK(gps_status_listener_);

  pose_gt_reader = node_->CreateReader<LocalizationEstimate>("/apollo/custom/pose", nullptr);

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
  localization_->GpsCallback(gps_msg);

  if (localization_->IsServiceStarted()) {
    LocalizationEstimate localization;
    localization_->GetLocalization(&localization);
    LocalizationStatus localization_status;
    localization_->GetLocalizationStatus(&localization_status);
    /*
    // inject : rtk_post
    auto* p = localization.mutable_pose()->mutable_position();
    p->set_x(p->x()+20);
    p->set_y(p->y()+21);
    */
    
    if(use_pose_gt)
    {
      LocalizationEstimate localization_gt;
      pose_gt_reader->Observe();
      auto latest_msg = pose_gt_reader->GetLatestObserved();
      if(latest_msg && latest_msg->header().sequence_num() != 0)
      {
        localization_gt = *latest_msg;
        auto mutable_pose = localization.mutable_pose();
        auto mutable_pose_gt = localization_gt.mutable_pose();
      
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

        // // linear velocity
        // if (mutable_pose_gt->has_linear_velocity()) {
        //   mutable_pose->mutable_linear_velocity()->CopyFrom(mutable_pose_gt->linear_velocity());
        // }
        
        // // linear acceleration
        // if (mutable_pose_gt->has_linear_acceleration()) {
        //   mutable_pose->mutable_linear_acceleration()->set_x(mutable_pose_gt->linear_acceleration().x());
        //   mutable_pose->mutable_linear_acceleration()->set_y(mutable_pose_gt->linear_acceleration().y());
        //   mutable_pose->mutable_linear_acceleration()->set_z(mutable_pose_gt->linear_acceleration().z());

        //   // linear_acceleration_vfr
        //   mutable_pose->mutable_linear_acceleration_vrf()->CopyFrom(mutable_pose_gt->linear_acceleration());
        // }

        // // angular velocity
        // if (mutable_pose_gt->has_angular_velocity()) {
        // mutable_pose->mutable_angular_velocity()->set_x(mutable_pose_gt->angular_velocity().x());
        // mutable_pose->mutable_angular_velocity()->set_y(mutable_pose_gt->angular_velocity().y());
        // mutable_pose->mutable_angular_velocity()->set_z(mutable_pose_gt->angular_velocity().z());

        // // angular_velocity_vf
        // mutable_pose->mutable_angular_velocity_vrf()->CopyFrom(mutable_pose_gt->angular_velocity());
        // }

        // // euler angle
        // if (mutable_pose_gt->has_euler_angles()) {
        //   mutable_pose->mutable_euler_angles()->CopyFrom(mutable_pose_gt->euler_angles());
        // }
      }
      // localization_gt = *pose_gt_reader->GetLatestObserved();

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
