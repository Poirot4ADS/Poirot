import json
import os
import pickle
import lgsvl
import math
def cal_localization_pose(scenario_name):
    topic = "/apollo/localization/pose"
    err_path = os.path.join("data/json_msg_error/localization", scenario_name) + '.record.00000'
    gt_path = os.path.join("data/json_msg_gt/localization", scenario_name) + '.record.00000'
    json_name = sorted(os.listdir(err_path), key=lambda x : x[:-5])
    weight = len(json_name)
    score = 0
    ave_dis = 10
    diff_ = ["position", "orientation", "linearVelocity", "linearAcceleration", "angularVelocity", "linearAccelerationVrf", "angularVelocityVrf", "eulerAngles"]
    for i, filename in enumerate(json_name):
        err_path_ = os.path.join(err_path, filename) 
        gt_path_ = os.path.join(gt_path, filename) 
        with open(err_path_, 'r') as f:
            json_err = json.load(f)
            json_err = json_err["pose"]
        with open(gt_path_, 'r') as f:
            json_gt = json.load(f)  
            json_gt = json_gt["pose"]  

        for diff in diff_:
            err = json_err[diff]
            gt = json_gt[diff]
            for k, v in err.items():
                score += i * max(0, math.sqrt((v - gt[k]) ** 2) - ave_dis) / ave_dis
    save_path = os.path.join("data/score", scenario_name) + '.json'
    with open(save_path,'r') as f:
        data = json.load(f)
    if topic in data.keys():
        data[topic] = score / weight
    else:
        data.setdefault(topic, score)
    with open(save_path, 'w') as f:
        json.dump(data, f, indent=4)
    print("calculate locaization pose done : " + scenario_name)

def cal_perception_obstacles(scenario_name):
    topic = "/apollo/perception/obstacles"
    err_path = os.path.join("data/json_msg_error/obstacles", scenario_name) + '.record.00000'
    gt_path = os.path.join("data/json_msg_gt/obstacles", scenario_name) + '.record.00000'
    json_name = sorted(os.listdir(err_path), key=lambda x : x[:-5])
    weight = len(json_name)
    score = 0
    ave_dis = 10
    ave_dis_ = 1.5
    diff_ = ["position", "velocity", "acceleration", "anchorPoint"]
    diff_2 = ["theta", "length", "width", "height"]
    for i, filename in enumerate(json_name):
        err_path_ = os.path.join(err_path, filename) 
        gt_path_ = os.path.join(gt_path, filename) 
        try:
            with open(err_path_, 'r') as f:
                json_err = json.load(f)
                json_err = json_err["perceptionObstacle"]
            with open(gt_path_, 'r') as f:
                json_gt = json.load(f)  
                json_gt = json_gt["perceptionObstacle"]  

            for j in range(len(json_err)):
                for diff in diff_:
                    err = json_err[j][diff]
                    gt = json_gt[j][diff]
                    for k, v in err.items():
                        score += i * max(0, math.sqrt((v - gt[k]) ** 2) - ave_dis) / ave_dis
                for diff in diff_2:
                    score += i * max(0, math.sqrt((json_err[j][diff] - json_gt[j][diff]) ** 2) - ave_dis_) / ave_dis_
        except:
            pass
    save_path = os.path.join("data/score", scenario_name) + '.json'
    with open(save_path,'r') as f:
        data = json.load(f)
    if topic in data.keys():
        data[topic] = score / weight
    else:
        data.setdefault(topic, score)
    with open(save_path, 'w') as f:
        json.dump(data, f, indent=4)
    print("calculate perception obstacles done : " + scenario_name)

def cal_perception_traffic_lights(scenario_name):
    topic = "/apollo/perception/traffic_light"
    err_path = os.path.join("data/json_msg_error/traffic_lights", scenario_name) + '.record.00000'
    gt_path = os.path.join("data/json_msg_gt/traffic_lights", scenario_name) + '.record.00000'
    json_name = sorted(os.listdir(err_path), key=lambda x : x[:-5])
    weight = len(json_name)
    score = 0
    diff_ = ["color", "id"]
    for i, filename in enumerate(json_name):
        err_path_ = os.path.join(err_path, filename) 
        gt_path_ = os.path.join(gt_path, filename) 
        with open(err_path_, 'r') as f:
            json_err = json.load(f)   
        with open(gt_path_, 'r') as f:
            json_gt = json.load(f)  
        try:
            if json_err["containLights"] != json_gt["containLights"]:
                score += i * 100
            elif json_err["containLights"] is False and json_gt["containLights"] is False:
                continue
            else:
                for diff in diff_:
                    err = json_err["trafficLight"]
                    gt = json_gt["trafficLight"]
                    for j in range(len(err)):
                        err_ = err[j][diff]
                        gt_ = gt[j][diff]
                        if err_ != gt_:
                            score += 100 * i
        except:
            pass
    save_path = os.path.join("data/score", scenario_name) + '.json'
    with open(save_path,'r') as f:
        data = json.load(f)
    if topic in data.keys():
        data[topic] = score / weight
    else:
        data.setdefault(topic, score)
    with open(save_path, 'w') as f:
        json.dump(data, f, indent=4)
    print("calculate perception traffic_lights done : " + scenario_name)

def cal_prediction(scenario_name):
    topic = "/apollo/prediction"
    err_path = os.path.join("data/json_msg_error/prediction", scenario_name) + '.record.00000'
    gt_path = os.path.join("data/json_msg_gt/prediction", scenario_name) + '.record.00000'
    json_name = sorted(os.listdir(err_path), key=lambda x : x[:-5])
    weight = len(json_name)
    score = 0
    ave_dis = 10
    ave_dis_ = 1.5
    diff_ = ["position", "velocity", "acceleration", "anchorPoint"]
    diff_2 = ["theta", "length", "width", "height"]
    diff_3 = ["priority", "isStatic"]
    for i, filename in enumerate(json_name):
        err_path_ = os.path.join(err_path, filename) 
        gt_path_ = os.path.join(gt_path, filename) 
        try:
            with open(err_path_, 'r') as f:
                json_err = json.load(f)
                json_err = json_err["predictionObstacle"]
            with open(gt_path_, 'r') as f:
                json_gt = json.load(f)  
                json_gt = json_gt["predictionObstacle"]  

            for j in range(len(json_err)):
                for diff in diff_:
                    err = json_err[j]["perceptionObstacle"][diff]
                    gt = json_gt[j]["perceptionObstacle"][diff]
                    for k, v in err.items():
                        score += i * max(0, math.sqrt((v - gt[k]) ** 2) - ave_dis) / ave_dis
                for diff in diff_2:
                    score += i * max(0, math.sqrt((json_err[j]["perceptionObstacle"][diff] - json_gt[j]["perceptionObstacle"][diff]) ** 2) - ave_dis_) / ave_dis_
                for diff in diff_3:
                    score += i * (100 if json_err[j][diff] != json_gt[j][diff] else 0)
        except:
            pass
    save_path = os.path.join("data/score", scenario_name) + '.json'
    with open(save_path,'r') as f:
        data = json.load(f)
    if topic in data.keys():
        data[topic] = score / weight
    else:
        data.setdefault(topic, score)
    with open(save_path, 'w') as f:
        json.dump(data, f, indent=4)
    print("calculate prediction done : " + scenario_name)

def cal_score(scenario_name):
    if not os.path.exists("data/score/"+scenario_name+'.json'):
        with open("data/score/"+scenario_name+'.json', 'w') as f:
            json.dump({}, f, indent=4)
    cal_localization_pose(scenario_name)
    cal_perception_obstacles(scenario_name)
    cal_perception_traffic_lights(scenario_name)
    cal_prediction(scenario_name)