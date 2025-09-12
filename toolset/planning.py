import lgsvl
import json
import math
import pickle
import os
from loguru import logger
import numpy as np


def trajectory_from_gt(scenario):
    gt_path = os.path.join("data/gt", scenario) + '.obj'
    #print(gt_path)
    with open(gt_path, 'rb') as f:
        gt_data = pickle.load(f)
    gt_traj = {}
    for vehicle, states in gt_data.items():
        if vehicle not in gt_traj.keys():
            gt_traj.setdefault(vehicle, [])
        for state in states:
            gt_traj[vehicle].append([state[2].easting, state[2].northing])
    return gt_traj

def collision_key_frame(gt_traj):
    ego_traj = gt_traj['ego']
    key_frame = -1
    for vehicle, positions in gt_traj.items():
        if vehicle != 'ego':
            for i, position in enumerate(positions):
                ego_position = ego_traj[i]
                npc_position = position
                dis = math.sqrt((ego_position[0] - npc_position[0]) ** 2 + (ego_position[1] - npc_position[1]) ** 2)
                if dis < 4:
                    if i > key_frame:
                        key_frame = i
                        break
    return key_frame

def align_time(js_name):
    time_index = [float(i[:-5]) for i in js_name]
    indices = ((np.array(time_index) - time_index[0]) // 0.1).astype(int)
    return indices

def close_to_des(ego_position, ego_des, scope):
    if ego_position is None:
        return False
    dis = math.sqrt((ego_des[0] - ego_position[0]) ** 2 + (ego_des[1] - ego_position[1]) ** 2)
    if dis <= scope:
        return True
    return False

def trajectory_from_planning(scenario, gt_traj):
    jsonfile_path = os.path.join("data/json_msg_error/planning", scenario) + '.record.00000'
    jsonfile_name = sorted(os.listdir(jsonfile_path), key=lambda x : x[:-5])
    
    jsonfile_indices = {}
    for i in range(len(gt_traj['ego'])):
        jsonfile_indices.setdefault(i, -1)

    ego_index = 0
    planning_traj = {}
    is_none = 0
    


    for json_name in jsonfile_name:
        json_path = os.path.join(jsonfile_path, json_name)
        
        with open(json_path, 'r') as f:
            data = json.load(f)
        try:
            # print(1)
            ego_planning_position = data["debug"]["planningData"]["adcPosition"]["pose"]["position"]
            # print(2)
            for i in range(ego_index, len(gt_traj['ego'])):
                ego_simulator_position = gt_traj['ego'][i]
                dis = math.sqrt((ego_simulator_position[0] - ego_planning_position['x']) ** 2 + (ego_simulator_position[1] - ego_planning_position['y']) ** 2)
                if dis < 0.1:
                    if jsonfile_indices[i] == -1:
                        try:
                            # print('==============================')
                            traj_points = data["trajectoryPoint"]
                            time_index = [float(i["relativeTime"]) for i in traj_points]
                            time_index = ((np.array(time_index) - time_index[0]) // 0.1).astype(int)
                            planning_traj.setdefault(i, [])
                            
                            for j in range(0, len(time_index)):
                                planning_frame = time_index[j]
                                ego_position = [traj_points[planning_frame]["pathPoint"]["x"], traj_points[planning_frame]["pathPoint"]["y"]]
                                planning_traj[i].append([ego_position, planning_frame])
                        except KeyError:
                            is_none += 1
                            break
                        # print(5)
                        jsonfile_indices[i] = 1
                        ego_index = i
                        break
        except KeyError:
            continue
    # print("is_none : ", is_none)
    # print(len(gt_traj['ego']))
    threshold = len(planning_traj) / 2
    return None if is_none > threshold else planning_traj

def compare_traj(key_frame, planning_traj, gt_traj):
    if key_frame == -1:
        print("no actual collision / no other NPCs")
        key_frame = len(gt_traj['ego'])
    key_traj = {k: v for k, v in planning_traj.items() if k < key_frame}
    for frame, future_traj in key_traj.items():
        for vehicle, npc_traj in gt_traj.items():
            if vehicle != 'ego':
                align_npc_traj = npc_traj[frame:]
                max_compare_length = min(future_traj[-1][1], len(align_npc_traj) - 1)
                for position in future_traj:
                    ego_position = position[0]
                    shift = position[1]
                    if shift <= max_compare_length:
                        npc_position = align_npc_traj[shift]
                        dis = math.sqrt((npc_position[0] - ego_position[0]) ** 2 + (npc_position[1] - ego_position[1]) ** 2)
                        if dis < 2.5:
                            return True
                    else:
                        break
    print("no planning collision")
    return False

def long_time_stop(planning_traj, ego_des):
    for frame, future_traj in planning_traj.items():
        stop_unreasonable = 0
        threshold = future_traj[-1][1] / 2
        # print(threshold)
        for i in range(1, len(future_traj)):
            ego_position = future_traj[i][0]
            now_frame = future_traj[i][1]
            pre_position = future_traj[i-1][0]
            pre_frame = future_traj[i-1][1]

            if not close_to_des(ego_position, ego_des, 3.5):
                if close_to_des(pre_position, ego_position, (now_frame - pre_frame) * 0.1 * 1.5):
                    #stop_unreasonable += 1 * (now_frame - pre_frame)
                    stop_unreasonable += 1 
                    if stop_unreasonable >= threshold:
                        return True
                else:
                    stop_unreasonable = 0
            else: stop_unreasonable = 0
    return False

def analysis_planning(scenario):
    gt_traj = trajectory_from_gt(scenario)
    key_frame = collision_key_frame(gt_traj)
    planning_traj = trajectory_from_planning(scenario, gt_traj)
    # print(planning_traj)
    if planning_traj is None:
        print("no planning trajectory long time!")
        return True
    if not planning_traj:
        print("no match frame")
        return True
    if compare_traj(key_frame, planning_traj, gt_traj):
        print("planning trajectory conflict!")
        return True
    if long_time_stop(planning_traj, gt_traj['ego'][-1]):
        print("planning long time stop unreasonable!")
        return True
    return False
    res = False
    if planning_traj is None: 
        print("no planning traj / long time no drive")
        return True
    # for key in planning_traj.keys():
    #     print(len(planning_traj[key]))
    print(len(planning_traj.keys()))
    for frame, future_traj in planning_traj.items():
        for vehicle, positions in gt_traj.items():
            if vehicle != 'ego':
                npc_traj = positions[frame:]
                res = compare_traj(npc_traj, future_traj)
                if res: return res
    return res
# scenario = ['7_14327', '7_14489', '7_14551', '7_14627', '7_14800', '7_14870', '7_15061']
# # scenario = ['7_14551']
# for a in scenario:
#     print('============= ',a,' ======================')

#     # scenario1 = a
#     # gt_traj1 = trajectory_from_gt(scenario1)
#     # key_frame1 = key_frame(gt_traj1)
#     # print(key_frame1)
#     # planning_traj1 = trajectory_from_planning(scenario1, key_frame1, gt_traj1)
#     # print(analysis_planning(gt_traj1, planning_traj1))
#     print(analysis_planning(a))
