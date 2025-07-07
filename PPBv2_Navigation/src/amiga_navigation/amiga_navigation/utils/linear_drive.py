#!/usr/bin/env python3
"""
Linear drive controller for robot navigation.
This module provides functionality for line tracking and path following
with differential drive control.
"""

from typing import Tuple, List, Union
import numpy as np
from simple_pid import PID

# Type aliases
Point2D = List[float]  # [x, y]
Point3D = List[float]  # [x, y, theta]
Route = List[Point2D]  # [start_point, end_point]

#v_x_l = 0.5 #target velocity parallel to the line
#v_y_l_max = 0.3 #maximum velocity perpendicular to the line

MAX_ERROR = 1.0

def is_aligned(route: Route, pose: Point3D, text_publisher) -> bool:
    """
    Check if the robot is aligned with the target path.
    
    Args:
        route: List containing start and end points of the path segment
        pose: Current robot pose [x, y, theta]
        text_publisher: ROS publisher for debug text messages
    
    Returns:
        bool: True if robot is aligned with path, False otherwise
    """
    pt2 = np.array(route[1])
    pt1 = np.array(route[0])
    theta = pose[2]
    path_vector = pt2 - pt1
    path_angle = np.arctan2(path_vector[1], path_vector[0])
    
    angle_diff = path_angle - theta
    # Normalize angle difference to [-pi, pi]
    if angle_diff > np.pi:
        angle_diff -= 2 * np.pi
    elif angle_diff < -np.pi:
        angle_diff += 2 * np.pi
        
    return abs(angle_diff) < 0.1

def get_cmd_turning(route: Route, pose: Point3D, text_publisher) -> Tuple[float, bool]:
    """
    Calculate turning command to align with path.
    
    Args:
        route: List containing start and end points of the path segment
        pose: Current robot pose [x, y, theta]
        text_publisher: ROS publisher for debug text messages
    
    Returns:
        Tuple[float, bool]: (angular_velocity, is_aligned)
    """
    pt2 = np.array(route[1])
    pt1 = np.array(route[0])
    theta = pose[2]
    path_vector = pt2 - pt1
    path_angle = np.arctan2(path_vector[1], path_vector[0])
    
    print(f"Path angle: {path_angle}")
    print(f"Robot angle: {theta}")
    
    angle_diff = path_angle - theta
    # Normalize angle difference to [-pi, pi]
    if angle_diff > np.pi:
        angle_diff -= 2 * np.pi
    elif angle_diff < -np.pi:
        angle_diff += 2 * np.pi
    
    print(f"Angle to align: {angle_diff}")
    
    TURN_SPEED = 0.2
    return TURN_SPEED * np.sign(angle_diff), (abs(angle_diff) < 0.1)

def get_projection_point(pt1: Point2D, pt2: Point2D, pt3: Point2D) -> Tuple[float, float]:
    """
    Calculate the projection of point pt3 onto line defined by pt1-pt2.
    
    Args:
        pt1: Start point of line
        pt2: End point of line
        pt3: Point to project
    
    Returns:
        Tuple[float, float]: Projected point coordinates (x, y)
    """
    x1, y1 = pt1[0], pt1[1]
    x3, y3 = float(pt3[0]), float(pt3[1])
    a = float(pt2[0] - pt1[0])
    b = float(pt2[1] - pt1[1])

    if a == 0:  # Vertical line
        return x3, y1
        
    denom = 1 + (b**2 / a**2)
    x4 = (b/a * (y3-y1) + x3 + (b**2/a**2) * x1) / denom
    y4 = b/a * (x4-x1) + y1

    return x4, y4

def is_goal_reached(route: Route, pose: Point3D, text_publisher) -> bool:
    """
    Check if the current goal point has been reached.
    
    Args:
        route: List containing start and end points of the path segment
        pose: Current robot pose [x, y, theta]
        text_publisher: ROS publisher for debug text messages
    
    Returns:
        bool: True if goal is reached, False otherwise
    """
    GOAL_THRESHOLD = 0.05
    
    pt2 = np.array(route[1])
    pt1 = np.array(route[0])
    pt3 = np.array(pose[0:2])
    pt4 = get_projection_point(pt1, pt2, pt3)
    
    # Distance from projected point to goal
    dist = np.linalg.norm(pt2 - pt4)
    print(f"Distance to goal: {dist}")
    
    # Check if we've reached or passed the goal
    goal_vector = pt2 - pt4
    path_vector = pt2 - pt1
    dot_product = np.dot(goal_vector, path_vector)
    
    # Use actual distance to goal, and also check if we've passed the goal
    return dist < GOAL_THRESHOLD or dot_product < 0

def get_cross_track_error(pt1: Point2D, pt2: Point2D, pt3: Point2D, pt4: Point2D) -> float:
    """
    Calculate the cross-track error (signed distance from path).
    
    Args:
        pt1: Path start point
        pt2: Path end point
        pt3: Current position
        pt4: Projected position
    
    Returns:
        float: Signed cross-track error
    """
    path_vector = np.array(pt2) - np.array(pt1)
    error_vector = np.array(pt3) - np.array(pt4)
    
    # Calculate 2D cross product for sign determination
    # For 2D vectors [a, b] and [c, d], cross product is a*d - b*c
    cross_product = path_vector[0] * error_vector[1] - path_vector[1] * error_vector[0]
    
    # Sign indicates which side of the path we're on
    return np.linalg.norm(error_vector) * np.sign(cross_product)

def getGoal():
    return 0

#input:
    #v_x: world coord x velocity
    #v_y: world coord x velocity
    #th: robot orientation w.r.t world coord x axis
def differential_drive(v_x: float, v_y: float, theta: float, epsilon: float) -> Tuple[float, float]:
    """
    Convert world frame velocities to differential drive commands.
    
    Args:
        v_x: Velocity in world x direction
        v_y: Velocity in world y direction
        theta: Robot orientation
        epsilon: Robot wheelbase parameter
    
    Returns:
        Tuple[float, float]: (linear_velocity, angular_velocity)
    """
    v = v_x * np.cos(theta) + v_y * np.sin(theta)
    w = 1.0/epsilon * (-v_x * np.sin(theta) + v_y * np.cos(theta))
    return v, w

class LineTrackingController:
    """Controller for tracking a line path using PID control."""
    
    def __init__(
        self,
        text_publisher,
        v_x_l: float = 0.5,
        v_y_l_max: float = 0.3,
        epsilon: float = 0.5,
        kp: float = 0.8,
        ki: float = 0.4,
        kd: float = 0.4,
        regulate_v_x_l: bool = False,
        is_turning: bool = False
    ):
        """
        Initialize the line tracking controller.
        
        Args:
            text_publisher: ROS publisher for debug messages
            v_x_l: Target velocity parallel to the line
            v_y_l_max: Maximum velocity perpendicular to the line
            epsilon: Robot wheelbase parameter
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            regulate_v_x_l: If true, regulate forward velocity such that the total velocity is set to be v_x_l or adjust_v_x_l
            is_turning: Whether the robot is in turning mode
        """
        self.pid = PID(kp, ki, kd, setpoint=0)
        self.pid.output_limits = (-v_y_l_max, v_y_l_max)
        self.v_x_l = v_x_l
        self.v_y_l_max = v_y_l_max
        self.epsilon = epsilon
        self.regulate_v_x_l = regulate_v_x_l
        self.is_turning = is_turning
        self.text_publisher = text_publisher

    def get_cmd_linear(
        self,
        route: Route,
        pose: Point3D,
        adjust_v_x_l: float = -1
    ) -> Tuple[float, float, float]:
        """
        Calculate linear tracking commands.
        
        Args:
            route: List containing start and end points of the path segment
            pose: Current robot pose [x, y, theta]
            adjust_v_x_l: Optional adjustment to line following velocity. Otherwise use default v_x_l
        
        Returns:
            Tuple[float, float, float]: (linear_velocity, angular_velocity, cross_track_error)
        """
        pt2 = np.array(route[1])
        pt1 = np.array(route[0])
        pt3 = np.array(pose[0:2])
        pt4 = get_projection_point(pt1, pt2, pt3)
        theta = pose[2]
        
        error = get_cross_track_error(pt1, pt2, pt3, pt4)
        print(f"Cross-track error: {error}")
        
        # Safety check for large deviations
        if not self.is_turning and abs(error) > MAX_ERROR:
            print("Error: Deviation too large. Check localization and restart navigation!")
            
            raise Exception("Deviation too large. Check localization and restart navigation!")
            
        # Calculate corrective velocity left to the line using PID
        v_y_l = self.pid(error) if not self.is_turning else 0
        
        # Calculate path angle and rotation matrix
        path_vector = pt2 - pt1
        path_angle = np.arctan2(path_vector[1], path_vector[0])
        R = np.array([
            [np.cos(path_angle), -np.sin(path_angle)],
            [np.sin(path_angle), np.cos(path_angle)]
        ])
        
        # Determine forward velocity
        current_v_x_l = adjust_v_x_l if adjust_v_x_l != -1 else self.v_x_l
        
        # Adjust forward velocity if needed
        if self.regulate_v_x_l:
            if current_v_x_l**2 <= v_y_l**2:
                raise ValueError("Target velocity too low for required correction")
            current_v_x_l = np.sqrt(current_v_x_l**2 - v_y_l**2)
            
        # Transform to world coordinates
        vx, vy = np.dot(R, [current_v_x_l, v_y_l])
        
        # Convert to differential drive commands
        v, w = differential_drive(vx, vy, theta, self.epsilon)
        
        return v, w, error