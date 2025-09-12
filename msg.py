import pickle
from cyber_record.record import Record
from google.protobuf import json_format
import os
import json
import argparse



from proto import localization_pb2, perception_obstacle_pb2, prediction_obstacle_pb2, traffic_light_detection_pb2, planning_pb2


def pb2json(proto_msg):
    str = json_format.MessageToJson(proto_msg)
    json_msg = json.loads(str)
    return json_msg

def save_message(frame, filedir_path):
    #print(frame)
    msg_path = str(frame['header']['timestampSec']) + '.json'    
    if not os.path.exists(filedir_path):
        os.makedirs(filedir_path)
    with open(os.path.join(filedir_path, msg_path),'w') as f:
        json.dump(frame, f, indent=4)

def refine_message(json_msg_file, msg_type):
    localization_proto = localization_pb2.LocalizationEstimate()
    obstacle_proto = perception_obstacle_pb2.PerceptionObstacles()
    traffic_light_proto = traffic_light_detection_pb2.TrafficLightDetection()
    prediction_proto = prediction_obstacle_pb2.PredictionObstacles()
    proto_type_dict = {'localization':localization_proto, 'obstacles': obstacle_proto, 'prediction': prediction_proto, 'traffic_lights': traffic_light_proto}

    scenario_path = os.path.join(json_msg_file, msg_type)
    for scenario_name in os.listdir(scenario_path):
        print(scenario_name)
        json_path = os.path.join(scenario_path, scenario_name)
        for json_file in os.listdir(json_path):
            proto_type = proto_type_dict[msg_type]
            json_msg_path = os.path.join(json_path, json_file)

            with open(json_msg_path, 'r') as f:
                json_data = json.load(f)
            json_format.ParseDict(json_data, proto_type)
            save_path = os.path.join("data/protobuf_msg", msg_type, scenario_name)
            if not os.path.exists(save_path):
                os.makedirs(save_path)
            save_path = os.path.join(save_path, json_file[:-5] + '.pb.bin')
            with open(save_path, 'wb') as f:
                f.write(proto_type.SerializeToString())

def messages_withdraw(record_dir):

    for filename in os.listdir(record_dir):
        file_path = os.path.join(record_dir, filename)
            
        record = Record(file_path)

        for channel, message, t in record.read_messages():

            
            if channel == "/apollo/localization/pose":
                frame = pb2json(message)  
                filedir_path = 'data/json_msg_error/localization/' + filename
                save_message(frame, filedir_path)

            elif channel == "/apollo/perception/obstacles":
                frame = pb2json(message)
                filedir_path = 'data/json_msg_error/obstacles/' + filename
                save_message(frame, filedir_path)
            elif channel == "/apollo/perception/traffic_light":
                frame = pb2json(message)
                filedir_path = 'data/json_msg_error/traffic_lights/' + filename
                save_message(frame, filedir_path)
            elif channel == "/apollo/prediction":
                frame = pb2json(message)
                filedir_path = 'data/json_msg_error/prediction/' + filename
                save_message(frame, filedir_path)
            elif channel == "/apollo/planning":
                frame = pb2json(message)
                filedir_path = 'data/json_msg_error/planning/' + filename
                save_message(frame, filedir_path)

        print(str(filename) + ' done')


parser = argparse.ArgumentParser()

parser.add_argument('--process', action='store_true', help='process raw record data')
parser.add_argument('--refine', action='store_true', help='transform ground-truth data to protobf msg')

args = parser.parse_args()
if args.process:
    record_path = 'data/record_error'
    messages_withdraw(record_path)
elif args.refine:
    refine_message('data/json_msg_gt', 'localization')
    refine_message('data/json_msg_gt', 'obstacles')
    refine_message('data/json_msg_gt', 'prediction')
    refine_message('data/json_msg_gt', 'traffic_lights')


