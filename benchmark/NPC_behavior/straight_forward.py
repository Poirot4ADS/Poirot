from . import utils
import lgsvl
import numpy as np
import math


def straight_forward(sim, npc, speed, forward, num_points, f_dis):
    """
    Generate straight path points .
    """
    waypoints = []
    # Set Bezier control points for a right turn
    n = f_dis
    P0 = npc.state.position
    P1 = P0 + forward * (n / 3) # Move forward
    P2 = P1 + forward * (n / 3)  # Start turning right
    P3 = P2 + forward * (n / 3)   # Finish the turn

    # Calculate initial and final angles
    # start_angle, end_angle = calculate_initial_final_angles(forward, right)
    #angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    # Generate waypoints using the Bezier curve and interpolate angles
    for i in range(num_points + 1):
        t = i / num_points
        current_point = utils.bezier_point(t, P0, P1, P2, P3)
        ground_point = utils.raycast_to_ground(sim, current_point)
        
        # print(interpolated_angle)
        # Create a waypoint at this position with the interpolated angle
        wp = lgsvl.DriveWaypoint(
            ground_point, speed, angle=angle)
        waypoints.append(wp)

    return waypoints


