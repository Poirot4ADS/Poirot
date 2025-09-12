cc_binary(
    name = "custom_pose",
    srcs = ["custom_pose.cc"],
    deps = [
        "//cyber",
        "//modules/localization/proto:localization_cc_proto",
        "@com_google_protobuf//:protobuf"
    ],
)
cc_binary(
    name = "pose_from_topic",  # 目标名称
    srcs = ["pose_from_topic.cc"],  # 源文件列表
    deps = [
        "//cyber",  # 引入 cyber 模块
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",  # 引入 GNSS 模块的 proto 文件
        "//modules/drivers/gnss/proto:ins_cc_proto",  # 引入 GNSS 的 INS proto 文件
        "//modules/drivers/gnss/proto:imu_cc_proto",  # 引入 IMU 模块的 proto 文件
        "//modules/localization/proto:gps_cc_proto",  # 引入 GPS 模块的 proto 文件
        "//modules/localization/proto:imu_cc_proto",  # 引入 IMU 模块的 proto 文件
        "//modules/transform:transform_broadcaster",  # 引入 transform_broadcaster 模块
        "//modules/localization/proto:localization_cc_proto",  # 引入 localization 模块的 proto 文件
        "@com_google_protobuf//:protobuf",  # 引入 Protocol Buffers 库
    ],
)
