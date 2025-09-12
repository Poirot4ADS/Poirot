from loguru import logger
from . import cut, straight_forward, turn, utils
import lgsvl
import math
import numpy as np

def genWaypoints(sim, npc, speed, npc_action, npc_des, label):
    forward = lgsvl.utils.transform_to_forward(npc.state.transform)
    right = lgsvl.utils.transform_to_right(npc.state.transform)
    npc_position = npc.state.position
    f_dis = calculate_distance(npc_des, npc_position, forward)
    r_dis = calculate_distance(npc_des, npc_position, right)
    direction = "right" if r_dis > 0 else "left"
    waypoints = []
    if npc_action == "cut":
        logger.info(f"{npc.name} will cut_in_{direction}")
        waypoints = cut.cut(sim, npc, speed, forward, right, 20, f_dis, r_dis)
    elif npc_action == "turn":
        logger.info(f"{npc.name} will turn_{direction}")
        waypoints = turn.turn(sim, npc, speed, forward, right, 20, f_dis, r_dis)
    elif npc_action == "straight":
        logger.info(f"{npc.name} will straight forward")
        waypoints = straight_forward.straight_forward(sim, npc, speed, forward, 20, f_dis)
    elif npc_action == "stop":
        logger.info(f"{npc.name} will stop")
    # angle!!!
    # waypoints = keepSpeed(sim, npc, forward, smoothSpeed(sim, npc, waypoints, forward))
    # waypoints = smoothSpeed(sim, npc, waypoints, forward)
    for waypoint in waypoints:
        if label == "14628":
            waypoint.position = waypoint.position + -2 * right
        # print(waypoint.position)
    return waypoints

def calculate_distance(final_point, now_point, forward):
    final_point = np.array([final_point['x'], final_point['z']])
    now_point = np.array([now_point.x, now_point.z])
    forward = np.array([forward.x, forward.z])
    direction_vector = final_point - now_point
    dot_product = np.dot(direction_vector, forward)
    forward_magnitude = np.linalg.norm(forward)

    return dot_product / forward_magnitude

def smoothSpeed(sim, npc, waypoints, forward):
    if waypoints == []:
        return []
    npc_speed = npc.state.speed
    npc_position = npc.state.position
 
    start_speed = waypoints[0].speed
    before_waypoints = []
    
 
    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    
    acc = start_speed - npc_speed 
    point_num = 8 if npc_speed > 6 else 3 
    for i in range(point_num):
        new_position = npc_position + forward * (i+1)
        new_position = utils.raycast_to_ground(sim, new_position)
   
        ground_point = sim.map_point_on_lane(new_position).position
        new_speed = npc_speed + acc * (i+1) / point_num
        before_waypoints.append(lgsvl.DriveWaypoint(ground_point, new_speed, angle=angle))
    
  
    for i in range(len(waypoints)):
        waypoints[i].position = waypoints[i].position + before_waypoints[-1].position - npc_position
    before_waypoints.extend(waypoints)
    
    return before_waypoints

def keepSpeed(sim, npc, forward, waypoints):
  
    keep_speed = waypoints[-1].speed
    start_position = waypoints[-1].position
    # npc_forward = lgsvl.utils.transform_to_forward(npc.state.transform)
    # angle = waypoints[-1].angle
    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    
    keep_waypoints = []
    for i in range(2):
        new_position = start_position + forward * (i+1)
        new_position = utils.raycast_to_ground(sim, new_position)
        # if new_position is  None:
        #     return True
        ground_point = sim.map_point_on_lane(new_position).position
        keep_waypoints.append(lgsvl.DriveWaypoint(ground_point, keep_speed, angle=angle))
        
    waypoints.extend(keep_waypoints)
    return waypoints