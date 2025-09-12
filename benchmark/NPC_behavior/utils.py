from loguru import logger
import numpy as np
import lgsvl


def raycast_to_ground(sim, position):
    '''
    start_height = 100
    start_point = lgsvl.Vector(
        position.x, position.y + start_height, position.z)
    try:
        hit = sim.raycast(start_point, lgsvl.Vector(0, -1, 0), layer_mask=1 << 0)
    except KeyError:
        print("Error: 'hit[0]' key is missing in the agent's state.")
        # return lgsvl.Vector(position.x, position.y, position.z)
        return position
    return hit.point
    '''
    start_height = 10
    start_point = lgsvl.Vector(
        position.x, position.y + start_height, position.z)

    hit = sim.raycast(start_point, lgsvl.Vector(0, -1, 0), layer_mask=1 << 0)
    #p = lgsvl.geometry.Vector(position.x, position.y, position.z)
    #print('p', p)
    if hit is not None:
        #print(hit.point)
        return hit.point
    else:
        logger.error("hit error")
        return None



def bezier_point(t, P0, P1, P2, P3):
    """ Calculate the cubic Bezier curve point at t """
    return (1 - t) ** 3 * P0 + 3 * (1 - t) ** 2 * t * P1 + 3 * (1 - t) * t ** 2 * P2 + t ** 3 * P3


def calculate_curvature(point1, point2, point3):
    
    if np.array_equal(point1, point2) or np.array_equal(point2, point3):
        return 0
 
    k = 0.5 * (point1.x * (point2.y - point3.y) - point2.x * (point1.y - point3.y) + point3.x * (point1.y - point2.y)) / \
        (point1.x * point2.x + point2.y * point3.y + point3.x * point1.y -
         point1.y * point2.x - point2.y * point3.x - point3.y * point1.x)
    return k

def bezier_derivative(t, P0, P1, P2, P3):
    """Calculate the derivative of a cubic Bezier curve at parameter t."""
    P0, P1, P2, P3 = np.array(P0), np.array(P1), np.array(P2), np.array(P3)
    return (-3 * (1 - t)**2 * P0
            + 3 * (1 - 4*t + 3*t**2) * P1
            + 3 * (2*t - 3*t**2) * P2
            + 3 * t**2 * P3)

def calculate_angle(prev_point, current_point):
    delta_x = current_point.x - prev_point.x
    delta_z = current_point.z - prev_point.z
    angle = np.arctan2(delta_z, delta_x)  
    return np.degrees(angle)  


def smooth_angles(angles, window_size=3):
    smoothed_angles = []
    for i in range(len(angles)):
        if i < window_size:
            smoothed_angles.append(np.mean(angles[:i+1]))
        else:
            smoothed_angles.append(np.mean(angles[i-window_size+1:i+1]))
    return smoothed_angles
