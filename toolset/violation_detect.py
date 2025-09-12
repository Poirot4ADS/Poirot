import lgsvl
import math
from loguru import logger
from shapely.geometry import Polygon, LineString

def judge_violation(sim, ego_state, envs, isViolation):
    v1 = cross_yellow_line(ego_state, envs[1])
    v2 = violate_traffic_lights(ego_state, envs[0])
    if v1 or v2:
        isViolation = True
    # print("isViolation : ", isViolation)  

def cross_yellow_line(ego_state, envs):
    if envs is None:
        return False
    for edge_line in envs:
        if ego_yellow_line_fault(ego_state, edge_line):
            logger.error("hit dangerous lines!")
            return True
    return False

def violate_traffic_lights(ego_state, envs):
    if envs is None:
        return False
    red_lights = envs['red']
    min_dis = 99999999
    for red_light_position in red_lights:
        dis = math.sqrt((ego_state[0].position.x - red_light_position[0]) ** 2 + (ego_state[0].position.z - red_light_position[2]) ** 2)
        if dis < min_dis:
            min_dis = dis
    if min_dis >= 60:
        return False
    else:
        logger.error("violate traffic lights!")
        return True

def close_to_des(ego_position, ego_des, scope):
    if ego_position is None:
        return False
    dis = math.sqrt((ego_des.x - ego_position.x) ** 2 + (ego_des.y - ego_position.y) ** 2)
    if dis <= scope:
        return True
    return False

def ego_yellow_line_fault(ego_state, yellow_line_points):
    ego_bbox = get_bbox(ego_state)
    yellow_line = LineString(yellow_line_points)
    distance_yellow = ego_bbox.distance(yellow_line)
    # print("distance_error : ", distance_yellow)
    if distance_yellow <= 0:
        return True
    else:
        return False  
    
def get_bbox(agent):
    agent_theta = agent[0].transform.rotation.y
    agent_bbox = agent[1] # min max (x_min, y_min, z_min) (x_max, y_max, z_max)
        
    global_x = agent[0].transform.position.x
    global_z = agent[0].transform.position.z
    x_min = agent_bbox.min.x + 0.1
    x_max = agent_bbox.max.x - 0.1
    z_min = agent_bbox.min.z + 0.1
    z_max = agent_bbox.max.z - 0.1

    line1 = [x_min, z_min, x_max, z_max]
    line2 = [x_min, z_max, x_max, z_min]
    x_center, z_center = get_line_cross_point(line1, line2)

    coords = [[x_min, z_min], [x_max, z_min], [x_max, z_max], [x_min, z_max]]
    new_coords = []
    for i in range(len(coords)):
        coord_i = coords[i]
        coord_i[0] = coord_i[0] - x_center
        coord_i[1] = coord_i[1] - z_center
        new_coord_i = right_rotation(coord_i, agent_theta)
        new_coord_i[0] += global_x
        new_coord_i[1] += global_z
        new_coords.append(new_coord_i)
    p1, p2, p3, p4 = new_coords[0], new_coords[1], new_coords[2], new_coords[3]

    agent_poly = Polygon((p1, p2, p3, p4))
    if agent_poly.area <= 0:
        print(agent_poly.area)
        exit(-1)
    return agent_poly

def get_line_cross_point(line1, line2):
    a0, b0, c0 = calc_abc_from_line_2d(*line1)
    a1, b1, c1 = calc_abc_from_line_2d(*line2)
    D = a0 * b1 - a1 * b0
    if D == 0:
        return None
    x = (b0 * c1 - b1 * c0) / D
    y = (a1 * c0 - a0 * c1) / D
    return x, y

def calc_abc_from_line_2d(x0, y0, x1, y1):
    a = y0 - y1
    b = x1 - x0
    c = x0 * y1 - x1 * y0
    return a, b, c

def right_rotation(coord, theta):
    """
    theta : degree
    """
    theta = math.radians(theta) 
    x = coord[1]
    y = coord[0]
    x1 = x * math.cos(theta) - y * math.sin(theta)
    y1 = x * math.sin(theta) + y * math.cos(theta)
    return [y1, x1]