#include <fstream>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include "cyber/cyber.h"
#include "cyber/time/clock.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/transform/transform_broadcaster.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"

using apollo::cyber::Clock;
using apollo::localization::Gps;
using apollo::drivers::gnss::Imu;
using apollo::drivers::gnss::InsStat;
using apollo::drivers::gnss::SolutionType;

using apollo::localization::CorrectedImu;
using apollo::transform::TransformStamped;
using apollo::transform::TransformBroadcaster;
using apollo::localization::LocalizationStatus;
using apollo::localization::LocalizationEstimate;

std::mutex gps_status_list_mutex_;
int64_t localization_seq_num_ = 0;
std::list<InsStat> gps_status_list_;
size_t gps_status_list_max_size_ = 10;
double gps_status_time_diff_threshold_ = 1.0;


std::unordered_map<double, std::shared_ptr<Imu>> imu_data;
std::unique_ptr<apollo::transform::TransformBroadcaster> tf2_broadcaster_;
std::unordered_map<double, std::shared_ptr<CorrectedImu>> corrected_imu_data;

std::shared_ptr<apollo::cyber::Writer<LocalizationEstimate>> localization_writer = nullptr;
std::shared_ptr<apollo::cyber::Writer<LocalizationStatus>> localization_status_writer = nullptr;

void PublishPoseBroadcastTF(const LocalizationEstimate& localization) {
  // broadcast tf message
  TransformStamped tf2_msg;

  auto mutable_head = tf2_msg.mutable_header();
  mutable_head->set_timestamp_sec(localization.measurement_time());
  mutable_head->set_frame_id("world");
  tf2_msg.set_child_frame_id("localization");

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

void PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  localization_writer->Write(localization);
}

void PublishLocalizationStatus(
    const LocalizationStatus& localization_status) {
  localization_status_writer->Write(localization_status);
}

bool FindNearestGpsStatus(const double gps_timestamp_sec, InsStat *status) {
  CHECK_NOTNULL(status);

  std::cout << "Start find!" << std::endl;
  std::unique_lock<std::mutex> lock(gps_status_list_mutex_);
  auto gps_status_list = gps_status_list_;
  lock.unlock();

  double timestamp_diff_sec = 1e8;
  auto nearest_itr = gps_status_list.end();
  for (auto itr = gps_status_list.begin(); itr != gps_status_list.end();
       ++itr) {
    double diff = std::abs(itr->header().timestamp_sec() - gps_timestamp_sec);
    if (diff < timestamp_diff_sec) {
      timestamp_diff_sec = diff;
      nearest_itr = itr;
    }
  }

  if (nearest_itr == gps_status_list.end()) {
    return false;
  }

  if (timestamp_diff_sec > gps_status_time_diff_threshold_) {
    return false;
  }

  *status = *nearest_itr;
  return true;
}

void FillLocalizationStatusMsg(const InsStat& status, LocalizationStatus* localization_status) {
    apollo::common::Header* header = localization_status->mutable_header();
    double timestamp = Clock::NowInSeconds();
    header->set_timestamp_sec(timestamp);
    localization_status->set_measurement_time(status.header().timestamp_sec());

    if (!status.has_pos_type()) {
        localization_status->set_fusion_status(apollo::localization::MeasureState::ERROR);
        localization_status->set_state_message("Error: Current Localization Status Is Missing.");
        return;
    }

    auto pos_type = static_cast<SolutionType>(status.pos_type());
    switch (pos_type) {
        case SolutionType::INS_RTKFIXED:
            localization_status->set_fusion_status(apollo::localization::MeasureState::OK);
            localization_status->set_state_message("");
            break;
        case SolutionType::INS_RTKFLOAT:
            localization_status->set_fusion_status(apollo::localization::MeasureState::WARNNING);
            localization_status->set_state_message("Warning: Current Localization Is Unstable.");
            break;
        default:
            localization_status->set_fusion_status(apollo::localization::MeasureState::ERROR);
            localization_status->set_state_message("Error: Current Localization Is Very Unstable.");
            break;
    }
}

void FillLocalizationMsgHeader(LocalizationEstimate *localization) {
  auto *header = localization->mutable_header();
  double timestamp = apollo::cyber::Clock::NowInSeconds();
  header->set_module_name("localization");
  header->set_timestamp_sec(timestamp);
  header->set_sequence_num(static_cast<unsigned int>(++localization_seq_num_));
}

void merge_and_save(double timestamp, const std::shared_ptr<Gps>& gps) {
    std::cout << "enter merge" << std::endl;
    auto imu_it = imu_data.find(timestamp);
    auto corrected_imu_it = corrected_imu_data.find(timestamp);

    if (imu_it != imu_data.end() && corrected_imu_it != corrected_imu_data.end()) {
        std::cout << "Start merge!" << std::endl;
        auto imu = imu_it->second;
        auto corrected_imu = corrected_imu_it->second;

        LocalizationEstimate localization_msg;
        FillLocalizationMsgHeader(&localization_msg);
        localization_msg.set_measurement_time(timestamp);
        
        auto* pose = localization_msg.mutable_pose();
        pose->mutable_position()->set_x(gps->localization().position().x());
        pose->mutable_position()->set_y(gps->localization().position().y());
        pose->mutable_position()->set_z(gps->localization().position().z());

        pose->mutable_orientation()->set_qx(gps->localization().orientation().qx());
        pose->mutable_orientation()->set_qy(gps->localization().orientation().qy());
        pose->mutable_orientation()->set_qz(gps->localization().orientation().qz());
        pose->mutable_orientation()->set_qw(gps->localization().orientation().qw());

        pose->mutable_linear_velocity()->set_x(gps->localization().linear_velocity().x());
        pose->mutable_linear_velocity()->set_y(gps->localization().linear_velocity().y());
        pose->mutable_linear_velocity()->set_z(gps->localization().linear_velocity().z());

        double heading = gps->localization().heading();
        double heading_in_radians = (heading - 270) * M_PI / 180.0;
        pose->set_heading(heading_in_radians);

        pose->mutable_linear_acceleration()->set_x(imu->linear_acceleration().x());
        pose->mutable_linear_acceleration()->set_y(imu->linear_acceleration().y());
        pose->mutable_linear_acceleration()->set_z(imu->linear_acceleration().z());
        pose->mutable_linear_acceleration_vrf()->CopyFrom(imu->linear_acceleration());

        pose->mutable_angular_velocity()->set_x(imu->angular_velocity().x());
        pose->mutable_angular_velocity()->set_y(imu->angular_velocity().y());
        pose->mutable_angular_velocity()->set_z(imu->angular_velocity().z());
        pose->mutable_angular_velocity_vrf()->CopyFrom(imu->angular_velocity());
        
        pose->mutable_euler_angles()->set_x(corrected_imu->imu().euler_angles().x());
        pose->mutable_euler_angles()->set_y(corrected_imu->imu().euler_angles().y());
        pose->mutable_euler_angles()->set_z(corrected_imu->imu().euler_angles().z());

        PublishPoseBroadcastTopic(localization_msg);
        //PublishPoseBroadcastTF(localization_msg);
        
        InsStat gps_status;
        LocalizationStatus localization_status;
        FindNearestGpsStatus(timestamp, &gps_status);
        FillLocalizationStatusMsg(gps_status, &localization_status);
        PublishLocalizationStatus(localization_status);
        

        imu_data.erase(imu_it);
        corrected_imu_data.erase(corrected_imu_it);
    }
}

void gps_callback(const std::shared_ptr<Gps>& msg) {
    std::cout << "gps get" << std::endl;
    double timestamp = msg->header().timestamp_sec();
    merge_and_save(timestamp, msg);
}

void imu_callback(const std::shared_ptr<Imu>& msg) {
    std::cout << "imu get" << std::endl;
    double timestamp = msg->header().timestamp_sec();
    imu_data[timestamp] = msg;
}

void corrected_imu_callback(const std::shared_ptr<CorrectedImu>& msg) {
    std::cout << "c-imu get" << std::endl;
    double timestamp = msg->header().timestamp_sec();
    corrected_imu_data[timestamp] = msg;
}

void gps_status_callback(const std::shared_ptr<InsStat> &status_msg) {
  std::unique_lock<std::mutex> lock(gps_status_list_mutex_);
  if (gps_status_list_.size() < gps_status_list_max_size_)
    gps_status_list_.push_back(*status_msg);
  else
  {
    gps_status_list_.pop_front();
    gps_status_list_.push_back(*status_msg);
  }
}

int main(int argc, char* argv[]) {

    apollo::cyber::Init("custom_Localization");
    std::string node_name = "pose" + std::to_string(getpid());
    auto node = apollo::cyber::CreateNode(node_name);
    std::shared_ptr<apollo::cyber::Node> shared_node = std::move(node);
    std::cout << "TF init!" << std::endl;
    tf2_broadcaster_.reset(new apollo::transform::TransformBroadcaster(shared_node));
    std::cout << "Init success!" << std::endl;

    localization_writer = shared_node->CreateWriter<LocalizationEstimate>("/apollo/custom/pose");
    auto gps_reader = shared_node->CreateReader<Gps>("/apollo/sensor/gnss/odometry", gps_callback);
    auto imu_reader = shared_node->CreateReader<Imu>("/apollo/sensor/gnss/imu", imu_callback);
    auto corrected_imu_reader = shared_node->CreateReader<CorrectedImu>("/apollo/sensor/gnss/corrected_imu", corrected_imu_callback);

   
    localization_status_writer = shared_node->CreateWriter<LocalizationStatus>("/apollo/localization/msf_status");
    auto ins_stat_reader = shared_node->CreateReader<InsStat>("/apollo/sensor/gnss/ins_stat", gps_status_callback);


    apollo::cyber::WaitForShutdown();
    return 0;
}
