# #!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import talos_conf as conf

# ROS
import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


# state machine 
# import smach
# import smach_ros

class AnkleStrategy:
    def __init__(self, Kx=3.0, Kp=1.5, omega=2.3):
        self.Kx = Kx
        self.Kp = Kp
        self.omega = omega

    def compute_desired_velocity(self, x_d, x_ref, p, p_ref, x_ref_dot):
        velocity = x_ref_dot - self.Kx * (x_d - x_ref) + self.Kp * (p - p_ref)
        return velocity


class HipStrategy:
    def __init__(self, K_gamma=1.0):
        self.Kg = K_gamma
    
    def compute_desired_angular_momentum(self, r, r_ref):
        angular_momentum = self.Kg * (r - r_ref)
        return angular_momentum