import numpy as np
from simple_pid import PID


turning_speed = 0.5 #speed at which the robot is commanded to perform the zero point turn such that it aligns to the route segment
def get_cmd_turning(route, pose):
        th = pose[2]
        dx = route[1][0] - route[0][0]  # x difference to end point
        dy = route[1][1] - route[0][1]  # y difference to end point
        line_th = np.arctan2(dy, dx)  # angle to end point
        
        # Handle angle wrapping properly
        angle_diff = line_th - th
        # Normalize angle difference to [-π, π]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        # Add buffer zone around ±180° to prevent oscillation
        BUFFER_ZONE = 0.1  # radians (about 5.7 degrees)
        if abs(abs(angle_diff) - np.pi) < BUFFER_ZONE:
            # If very close to ±180°, choose a consistent direction (e.g., always positive)
            angle_diff = np.pi if angle_diff > 0 else -np.pi
            
        print(f"Current heading: {np.degrees(th):.1f}°, Target angle: {np.degrees(line_th):.1f}°, Difference: {np.degrees(angle_diff):.1f}°")
        return 0.3*np.sign(angle_diff)