#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>
#include <memory>
#include <iomanip> // 需要包含此头文件以使用 std::fixed 和 std::setprecision

#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "cyber/timer/timer.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

using apollo::prediction::PredictionObstacles;

// Pair: <float_prefix, filepath>
using MsgFile = std::pair<double, std::string>;

class PredictionFolderReplayer {
 public:
  explicit PredictionFolderReplayer(const std::string& folder)
      : folder_(folder) {}

  void Run() {
    if (!LoadFileList()) {
      std::cerr << "Failed to load message file list" << std::endl;
      return;
    }

    apollo::cyber::Init("custom_Prediction");
    std::string node_name = "prediction" + std::to_string(getpid());
    auto node = apollo::cyber::CreateNode(node_name);
    auto writer = node->CreateWriter<PredictionObstacles>(
        "/apollo/custom/prediction");

    std::shared_ptr<PredictionObstacles> msg = std::make_shared<PredictionObstacles>();

    timer_ = std::make_unique<apollo::cyber::Timer>(500, [this, writer, msg]() {
      if (current_index_ >= sorted_files_.size()) {
        auto end_msg = std::make_shared<PredictionObstacles>();
        end_msg->mutable_header()->set_sequence_num(0); // 特殊标志
        writer->Write(end_msg);
        AINFO << "All messages published.";
        timer_->Stop();
        return;
      }

      const auto& path = sorted_files_[current_index_].second;
      if (!LoadFromFile(path, msg.get())) {
        AERROR << "Failed to load message from: " << path;
        return;
      }

      writer->Write(msg);
      // AINFO << "Published msg from: " << path;
      current_index_++;

    }, false);  // false = periodic

    timer_->Start();
    apollo::cyber::WaitForShutdown();
  }

 private:
  std::string folder_;
  std::vector<MsgFile> sorted_files_;
  size_t current_index_ = 0;
  std::unique_ptr<apollo::cyber::Timer> timer_;

  bool LoadFileList() {
    DIR* dir = opendir(folder_.c_str());
    if (!dir) {
      std::cerr << "Cannot open directory: " << folder_ << std::endl;
      return false;
    }

    struct dirent* entry;
    //std::regex float_file_pattern(R"(^([0-9]+\.?[0-9]*)\.pb\.bin$)");
    std::regex float_file_pattern(R"(^([0-9]+(?:\.[0-9]+)?)\.pb\.bin$)");


    while ((entry = readdir(dir)) != nullptr) {
      std::string filename(entry->d_name);
      std::smatch match;
      if (std::regex_match(filename, match, float_file_pattern)) {
        double prefix = std::stod(match[1].str());
        //std::cout << std::fixed << std::setprecision(7) << "match " << match[1] << ", prefix " << prefix << ", filename " << filename << std::endl;
        sorted_files_.emplace_back(prefix, folder_ + "/" + filename);
      }
    }

    closedir(dir);
    std::sort(sorted_files_.begin(), sorted_files_.end(),
              [](const MsgFile& a, const MsgFile& b) {
                return a.first < b.first;
              });
    
    // for (const auto& file : sorted_files_) {
    // std::cout << std::fixed << std::setprecision(7) << "Prefix: " << file.first 
    //           << ", Path: " << file.second << std::endl;
      // }

    std::cout << "Loaded " << sorted_files_.size() << " files." << std::endl;
    return !sorted_files_.empty();
  }

  bool LoadFromFile(const std::string& path, PredictionObstacles* msg) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
      std::cerr << "Cannot open file: " << path << std::endl;
      return false;
    }
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());
    return msg->ParseFromString(content);
  }
};

int main(int argc, char** argv) {
  // if (argc < 2) {
  //   std::cerr << "Usage: directory_player <folder_path>" << std::endl;
  //   return -1;
  // }
  // 要根据场景名称拼接
  std::string folder = argv[1];
  PredictionFolderReplayer replayer(folder);
  replayer.Run();

  return 0;
}
