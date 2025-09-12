protoc -I=/mnt/sda/apollo7 --python_out=. /mnt/sda/apollo7/modules/localization/proto/localization.proto
protoc -I=/mnt/sda/apollo7 --python_out=. /mnt/sda/apollo7/modules/perception/proto/perception_obstacle.proto
protoc -I=/mnt/sda/apollo7 --python_out=. /mnt/sda/apollo7/modules/perception/proto/traffic_light_detection.proto
protoc -I=/mnt/sda/apollo7 --python_out=. /mnt/sda/apollo7/modules/prediction/proto/prediction_obstacle.proto
protoc -I=/mnt/sda/apollo7 --python_out=. /mnt/sda/apollo7/modules/planning/proto/planning.proto