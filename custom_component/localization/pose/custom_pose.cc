
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <google/protobuf/util/json_util.h>
#include "cyber/cyber.h"
#include "modules/localization/proto/localization.pb.h"

using apollo::localization::LocalizationEstimate;
using google::protobuf::util::JsonStringToMessage;

int main(int argc, char** argv) {
  // if (argc < 2) {
  //   std::cerr << "Usage: directory_player <folder_path>" << std::endl;
  //   return -1;
  // }
  // 要根据场景名称拼接
  std::string json_file = argv[1];

  std::ifstream file(json_file);

  if(!file.is_open())
  {
    std::cerr << "Cannot open json file, " << json_file;
    return -1;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string json_str = buffer.str();
  file.close();

  //std::string json_str(std::istreambuf_iterator<char>(lfs), std::istream_iterator<char>());
  LocalizationEstimate msg;
  auto status = JsonStringToMessage(json_str, &msg);
  std::cerr << "status" << status;
  if(!status.ok())
  {
    std::cerr << "Failed to parse json";
    return -1;
  }

  apollo::cyber::Init("custom_Localization");
  std::string node_name = "pose" + std::to_string(getpid());
  auto node = apollo::cyber::CreateNode(node_name);
  auto writer = node->CreateWriter<LocalizationEstimate>(
      "/apollo/custom/pose");

  writer->Write(std::make_shared<LocalizationEstimate>(msg));
  apollo::cyber::SleepFor(std::chrono::milliseconds(100));
  return 0;
}
