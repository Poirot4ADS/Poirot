from . import utils
import lgsvl
import numpy as np
import math


def turn(sim, npc, speed, forward, right, num_points, f_dis, r_dis):
    """
    Generate right turn path points with interpolated angles.
    """
    waypoints = []
    k = r_dis / 2
    # Set Bezier control points for a right turn
    P0 = npc.state.position
    P1 = P0 + forward * f_dis  # Move forward
    P2 = P1 + right * k  # Start turning right
    P3 = P2 + right * (r_dis-k)  # Finish the turn

    # Calculate initial and final angles
    start_angle, end_angle = calculate_initial_final_angles(forward, r_dis)

    # Generate waypoints using the Bezier curve and interpolate angles
    for i in range(num_points + 1):
        t = i / num_points
        current_point = utils.bezier_point(t, P0, P1, P2, P3)
        ground_point = utils.raycast_to_ground(sim, current_point)
        interpolated_angle = linear_interpolate_angle(
            start_angle, end_angle, t)
        #print(interpolated_angle)
        # Create a waypoint at this position with the interpolated angle
        wp = lgsvl.DriveWaypoint(
            ground_point, speed, angle=lgsvl.Vector(0, interpolated_angle, 0))
        waypoints.append(wp)

    return waypoints


def linear_interpolate_angle(start_angle, end_angle, t):
    """Linearly interpolate between two angles."""
    return start_angle + (end_angle - start_angle) * t


def calculate_initial_final_angles(forward, r_dis):
    """Calculate the initial and final angles based on direction vectors."""
    start_angle = np.degrees(np.arctan2(forward.x, forward.z))
    # end_angle = np.degrees(np.arctan2(right.x, right.z))
    end_angle = start_angle + (90 if r_dis > 0 else -90)
    return start_angle, end_angle
