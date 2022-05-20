"""
Vision Based Navigation Project
Augusta University
3/11/2022

File that contains the VelocityTracker class.
This class is used as an attribute of ID from ids.py.
It manages all the attributes required to determine
the direction of the drone, the direction needed to 
navigate to a flag, and the angle between the 
direction of the drone and the path. 

velocityTracker.py
"""
from dataclasses import dataclass, field
from point import Point
from math import atan2, pi


@dataclass
class velocityTracker():
    """
    Class than determines the direction and angle between a setpoint and a 
    current position
    """
    target_flag : Point = field(init=False, default=None)
    vec_start : Point = field(init=False, default=None)
    vec_end : Point = field(init=False, default=None)
    dir_vec : Point = field(init=False, default=None)
    flag_vec : Point = field(init=False, default=None)
    turn_angle : int = field(init=False, default=None)
    flag_distance : int = field(init=False, default=None)

    def update_vectors(self):
        """ Updates all the vectors and angles
            based on new information. """
            
        self.dir_vec = self.vec_end - self.vec_start
        self.flag_vec = self.target_flag - self.vec_end
        
        dot = self.dir_vec.dot(self.flag_vec)
        det = self.dir_vec.dot(self.flag_vec)

        self.turn_angle = atan2(det, dot) * 180/pi
        self.flag_distance = self.vec_end.dist(self.target_flag)
