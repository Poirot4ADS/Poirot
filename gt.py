import argparse
import pickle
import os
import json
import lgsvl
import numpy as np
import math

def write_score(scenario, component, score):
    save_path = "data/score/" + scenario + '.json'
    with open(save_path, "r") as f:
        data = json.load(f)
    data.setdefault(component, score)
    with open(save_path, "w") as f:
        json.dump(data, f, indent=4)

def read_gt(gt_name):
    gt_dir = os.path.join("data/gt", gt_name)
    with open(gt_dir, 'rb') as f:
        gt = pickle.load(f)
        '''
        data[x][0] = vehicle.state
        data[x][1] = vehicle.BoundingBox .min .max .center .size
        data[x][2] = vehicle.GpsData .northing(y) .easting(x) .altitude(z)
        '''
    return gt

def align_time(js_name):
    time_index = [float(i[:-5]) for i in js_name]
    indices = ((np.array(time_index) - time_index[0]) // 0.1).astype(int)
    return indices

def save_modified_json(save_path, json_name, data):
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    with open(os.path.join(save_path, json_name), 'w') as f:
        json.dump(data, f, indent=4)

def modify_localization(json_msg_er_path):
    er_msg_path = os.path.join(json_msg_er_path, 'localization')

    for scenario_name in os.listdir(er_msg_path): 
        # read scenario ground truth 
        gt_name = scenario_name[:-len(".record.00000")] 
        gt = read_gt(gt_name + '.obj')
        ego_data = gt['ego']

        js_path = os.path.join(er_msg_path, scenario_name)
        js_name = sorted(os.listdir(js_path), key=lambda x : x[:-5]) # ordered json file name list

        indices = align_time(js_name)
        for i in range(len(js_name)):
            p = os.path.join(js_path, js_name[i])
            with open(p, 'r') as f:
                json_data = json.load(f)
            index = indices[i] if indices[i] < len(ego_data) else -1
            gps_data = ego_data[index][2]

            json_data["pose"]["position"]["x"] = gps_data.easting
            json_data["pose"]["position"]["y"] = gps_data.northing
            json_data["pose"]["position"]["z"] = gps_data.altitude
            
            save_path = os.path.join("data/json_msg_gt/localization", scenario_name)
            save_modified_json(save_path, js_name[i], json_data)
        # print("scenario : "+scenario_name+" done")
    print("localizaiton done")
    
            
def modify_traffic_lights(json_msg_er_path):
    er_msg_path = os.path.join(json_msg_er_path, 'traffic_lights')

    for scenario_name in os.listdir(er_msg_path): 
        # read scenario ground truth 
        gt_name = scenario_name[:-len(".record.00000")] 
        gt = read_gt(gt_name + '.obj')
        

        js_path = os.path.join(er_msg_path, scenario_name)
        
        js_name = sorted(os.listdir(js_path), key=lambda x : x[:-5]) # ordered json file name list
        #print(js_name)
        indices = align_time(js_name)
        for i in range(len(js_name)):
            p = os.path.join(js_path, js_name[i])
            with open(p, 'r') as f:
                json_data = json.load(f)
    
            try:
                json_data["trafficLight"]["color"] = "GREEN"
            except:
                pass
            
            save_path = os.path.join("data/json_msg_gt/traffic_lights", scenario_name)
            save_modified_json(save_path, js_name[i], json_data)
    print("traffic_lights done")

def theta_trans(orientation):
    radians = orientation * math.pi / 180
    # if radians > math.pi:
    #     radians -= 2 * math.pi
    # elif radians < -math.pi:
    #     radians += 2 * math.pi
    return radians + 2 * math.pi

def match_id(id_map, gt_map):
    # gt >= id 
    id_to_npc = {}
    pass_npc = []
    for id, id_state in id_map.items():
        distance = 9999999
        target_npc = None
        id_to_npc.setdefault(id, 'None')
        for npc, npc_state in gt_map.items():
            if npc in pass_npc:
                continue
            d = (id_state[0][0] - npc_state[0][0]) ** 2 + (id_state[0][1] - npc_state[0][1]) ** 2 + (id_state[0][2] - npc_state[0][2]) ** 2 + (id_state[1] - npc_state[1]) ** 2
            if d < distance:
                distance = d 
                target_npc = npc
        id_to_npc[id] = target_npc
        pass_npc.append(target_npc)
    return id_to_npc

def modify_obstacles(json_msg_er_path):
    er_msg_path = os.path.join(json_msg_er_path, 'obstacles')

    for scenario_name in os.listdir(er_msg_path): 
        # read scenario ground truth 
        gt_name = scenario_name[:-len(".record.00000")] 
        gt = read_gt(gt_name + '.obj')
        ego_data = gt['ego']
        npc_list = {}
        for vehicle, rec in gt.items():
            if vehicle != 'ego':
                npc_list.setdefault(vehicle, rec)
                # print(vehicle, rec[0][2].northing, rec[0][2].easting, rec[0][2].orientation)
        # print({k:[[v[0][2].easting, v[0][2].northing, v[0][2].altitude], theta_trans(v[0][2].orientation)]for k, v in npc_list.items()})
        # break
        js_path = os.path.join(er_msg_path, scenario_name)
        js_name = sorted(os.listdir(js_path), key=lambda x : x[:-5]) # ordered json file name list

        indices = align_time(js_name)

        for i in range(len(js_name)):
            p = os.path.join(js_path, js_name[i])
            with open(p, 'r') as f:
                json_data = json.load(f)
            index = indices[i] if indices[i] < len(ego_data) else -1

            # match id
            try:
                obstacle_map = {}
                for obstacle in json_data['perceptionObstacle']:
                    obstacle_position = [obstacle['position']['x'], obstacle['position']['y'], obstacle['position']['z']]
                    obstacle_map.setdefault(obstacle['id'], [obstacle_position, obstacle['theta']])
                # id -> npc.name
                match_res = match_id(obstacle_map, {k:[[v[index][2].easting, v[index][2].northing, v[index][2].altitude], theta_trans(v[index][2].orientation)]for k, v in npc_list.items()})

                for obstacle in json_data['perceptionObstacle']:
                    id = obstacle['id']
                    npc_name = match_res[id]
                    obstacle['position']['x'] = npc_list[npc_name][index][2].easting
                    obstacle['position']['y'] = npc_list[npc_name][index][2].northing
                    obstacle['position']['z'] = npc_list[npc_name][index][2].altitude
            except KeyError:
                pass

            save_path = os.path.join("data/json_msg_gt/obstacles", scenario_name)
            save_modified_json(save_path, js_name[i], json_data)
    print("obstacles done")

def modify_prediction(json_msg_er_path):
    er_msg_path = os.path.join(json_msg_er_path, 'prediction')

    for scenario_name in os.listdir(er_msg_path): 
        # read scenario ground truth 
        gt_name = scenario_name[:-len(".record.00000")] 
        gt = read_gt(gt_name + '.obj')
        ego_data = gt['ego']
        npc_list = {}
        for vehicle, rec in gt.items():
            if vehicle != 'ego':
                npc_list.setdefault(vehicle, rec)
                # print(vehicle, rec[0][2].northing, rec[0][2].easting, rec[0][2].orientation)

        js_path = os.path.join(er_msg_path, scenario_name)
        js_name = sorted(os.listdir(js_path), key=lambda x : x[:-5]) # ordered json file name list

        indices = align_time(js_name)

        for i in range(len(js_name)):
            p = os.path.join(js_path, js_name[i])
            with open(p, 'r') as f:
                json_data = json.load(f)
            index = indices[i] if indices[i] < len(ego_data) else -1

            # match id
            try:
                obstacle_map = {}
                for obstacle in json_data['predictionObstacle']:
                    obstacle = obstacle['perceptionObstacle']
                    obstacle_position = [obstacle['position']['x'], obstacle['position']['y'], obstacle['position']['z']]
                    obstacle_map.setdefault(obstacle['id'], [obstacle_position, obstacle['theta']])
                # id -> npc.name
                match_res = match_id(obstacle_map, {k:[[v[index][2].easting, v[index][2].northing, v[index][2].altitude], theta_trans(v[index][2].orientation)]for k, v in npc_list.items()})

                for obstacles in json_data['predictionObstacle']:
                    obstacle = obstacles['perceptionObstacle']
                    id = obstacle['id']
                    npc_name = match_res[id]

                    obstacle['position']['x'] = npc_list[npc_name][index][2].easting
                    obstacle['position']['y'] = npc_list[npc_name][index][2].northing
                    obstacle['position']['z'] = npc_list[npc_name][index][2].altitude

                    obstacles['priority']['priority'] = 'NORMAL'

                    try:
                        trajectory = obstacles['trajectory'][0]['trajectoryPoint']
                        start_idx = index
                        end_idx = min(index + 79, len(ego_data) - 1)
                        for traj in trajectory:
                            traj['pathPoint']['x'] = npc_list[npc_name][start_idx][2].easting
                            traj['pathPoint']['y'] = npc_list[npc_name][start_idx][2].northing
                            # traj['pathPoint']['z'] = npc_list[npc_name][start_idx][2].altitude
                            start_idx += 1
                            if start_idx > end_idx: start_idx = end_idx
                    except:
                        obstacles.setdefault("trajectory", [])
                        gt_traj = {}
                        gt_traj.setdefault('probability', 0.8)
                        gt_traj.setdefault('trajectoryPoint', [])
                        start_idx = index
                        end_idx = min(index + 79, len(ego_data) - 1)
                        for j in range(80):
                            path_point = {}
                            path_point.setdefault("pathPoint", {})
                            path_point['pathPoint'].setdefault('x', npc_list[npc_name][start_idx][2].easting)
                            path_point['pathPoint'].setdefault('y', npc_list[npc_name][start_idx][2].northing)
                            path_point['pathPoint'].setdefault('z', npc_list[npc_name][start_idx][2].altitude)

                            path_point.setdefault("v", npc_list[npc_name][start_idx][0].speed)

                            path_point.setdefault("relativeTime", j * 0.1)

                            gt_traj['trajectoryPoint'].append(path_point)

                            start_idx += 1
                            if start_idx > end_idx: start_idx = end_idx
                        obstacles['trajectory'].append(gt_traj)
            except:
                pass

            save_path = os.path.join("data/json_msg_gt/prediction", scenario_name)
            save_modified_json(save_path, js_name[i], json_data)
    print("prediction done")




er_path = 'data/json_msg_error'
modify_localization(er_path)
modify_traffic_lights(er_path)
modify_obstacles(er_path)
modify_prediction(er_path)

