import numpy as np
from . import utils
import lgsvl
import random
from loguru import logger

def cut(sim, npc, speed, forward, right, num_points, f_dis, r_dis=3.5):
    
    waypoints = []
    n = f_dis/3
    
    P0 = npc.state.position + forward * n
    P1 = P0 + forward * n
    P2 = P1 + right * r_dis
    P3 = P2 + forward * n
    points = [P0, P1, P2, P3]

    for i in range(1, num_points + 1):
        t = i / num_points
        current_point = utils.bezier_point(t, *points)
        ground_point = utils.raycast_to_ground(sim, current_point)
        
        tangent = utils.bezier_derivative(t, *points)
        angle = np.degrees(np.arctan2(tangent.x, tangent.z))
        waypoints.append(lgsvl.DriveWaypoint(ground_point, 0, angle=lgsvl.Vector(0, angle, 0)))  
            
            
    for waypoint in waypoints:
        waypoint.speed = speed
    return waypoints
    
def calculate_distance(final_point, ego, forward):
    final_point = np.array([final_point.position.x, final_point.position.z])
    ego_position = np.array([ego.state.position.x, ego.state.position.z])
    forward = np.array([forward.x, forward.z])
    direction_vector = final_point - ego_position
    dot_product = np.dot(direction_vector, forward)
    forward_magnitude = np.linalg.norm(forward)
    return dot_product / forward_magnitude

def calculate_velocity(ego, npc, n_forward, distance):
    is_npc_ahead = npc.state.position.x < ego.state.position.x

    npc_length = npc.bounding_box.max.x - npc.bounding_box.min.x
    t0 = (n_forward - (npc_length / 2)) / max(ego.state.speed, 10)
    t1 = (n_forward + (npc_length / 2)) / max(ego.state.speed, 10)
    v0 = distance / max(t0, 1)
    v1 = distance / max(t1, 1)
    velocity = np.random.uniform(v0, v1)

    if not is_npc_ahead:
        velocity = min(velocity, ego.state.speed)
    print(f"velocity:{velocity}")
    return velocity

