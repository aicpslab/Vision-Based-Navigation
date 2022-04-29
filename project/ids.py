"""
Vision Based Navigation Project
Augusta University
3/11/2022

This file contains the ID class that 
is responsible for managing which drone
is which. 

ids.py
"""
# Python Imports
from dataclasses import dataclass, field
from typing import ClassVar
from cv2 import circle, line, putText, FONT_HERSHEY_PLAIN, imshow, waitKey
from collections import deque
# Custom Imports
from point import Point
from velocityTracker import velocityTracker
from drawing import DrawingConstants as dc


@dataclass
class ID:
    """
        ID class. Keeps track of all ID's and attributes.
    """
    # Class Variables
    instances : ClassVar[list] = []
    next_id_num : ClassVar[int] = 1

    # Attributes
    position : Point
    color : tuple = field(default=dc.red)
    id_num : int = field(init=False)
    velocityTracking : velocityTracker = field(init=False)
    position_history : deque = field(init=False)
    
    # Class Methods

    @classmethod
    def createID(cls, point:Point, color:tuple):
        """ Creates an ID and puts it in the instances list """
        id = ID(point, color)
        id.id_num = ID.next_id_num
        ID.next_id_num += 1
        cls.instances.append(id)

    @classmethod
    def update_positions(cls, possible_positions : list[Point]):
        """ Given possible_positions, this function determines
            which drone is closest to the possible_positions
            and reassigns this new position as the drones 
            position. """
        for id in cls.instances:
            distances = []
            for position in possible_positions:
                # Determine how far the current ID is 
                # away from each position. 
                distances.append(position.dist(id.position))
            
            if distances != []:
                # Chose the lowest position
                lowest_index = distances.index(min(distances))
                updated_position = possible_positions[lowest_index]
                # Remove the old position so another drone can not claim it
                del possible_positions[lowest_index]

                id.appendPosition(updated_position)
    
    @classmethod 
    def set_target_flag(cls, possible_flags):
        """ Sets the target_flag attribute of id.velocityTracking 
            for each instance of an ID. """
        if possible_flags:
            for id in cls.instances:
                id.velocityTracking.target_flag = possible_flags[0]
    
    @classmethod
    def set_vec_start(cls):
        """ Sets the vec_start attribute of id.velocityTracking 
            for each instance of an ID. """
        for id in cls.instances:
            id.velocityTracking.vec_start = id.position
    
    @classmethod 
    def set_vec_end(cls):
        """ Sets the vec_end attribute of id.velocityTracking 
            for instance of an ID. """
        for id in cls.instances:
            id.velocityTracking.vec_end = id.position
    
    @classmethod
    def update_vecs(cls):
        """ Calls the update_vector method of velocityTracking
            for each instance of an ID. """
        for id in cls.instances:
            id.velocityTracking.update_vectors()

    @classmethod
    def assign_destination(self, destinations):
        """ Not implemented method to take a list of 
            flags/destinations and split them between
            each ID. """
        pass

    @classmethod
    def draw_histories(cls, img, num_points):
        """ Draws Points that each ID was previously registered at. """
        for id in cls.instances:
            for point in id.position_history:
                img = circle(img, (point.x, point.y) , dc.circle_radius, id.color, dc.circle_thickness)
        return img 
    
    @classmethod
    def draw_vectors(cls, img):
        """ Draws the direction vectors, and the path to get to the flag 
        for each instance of an ID. """
        cir_rad = dc.circle_radius
        cir_thick = dc.circle_thickness
        line_thick = dc.line_thickness
        for id in cls.instances:
            vec_start = id.velocityTracking.vec_start
            target_flag = id.velocityTracking.target_flag 
            dir_vec = id.velocityTracking.dir_vec
            flag_vec = id.velocityTracking.flag_vec
            vec_end = id.velocityTracking.vec_end


            img = circle(img, vec_start, cir_rad, id.color, cir_thick)
            img = circle(img, target_flag, cir_rad, dc.green, cir_thick)
            img = circle(img, vec_end, cir_rad, cir_thick)
            img = line(img, vec_end, vec_end+dir_vec, dc.pink, line_thick)
            img = line(img, vec_end, vec_end + flag_vec, dc.gb, line_thick)
            
        return img
    
    @classmethod
    def draw_id_nums(cls, img):
        """ Draws the ID# near each ID for each instance of an ID. """
        for id in cls.instances:
            id_position = Point(id.position.x + 40, id.position.y)
            img = putText(img, str(id.id_num), id_position,
                          fontFace=FONT_HERSHEY_PLAIN,
                          fontScale=2, color=dc.black, thickness=2) 
        return img

    # Instance Methods
    def __post_init__(self):
        self.position_history = deque([self.position], maxlen=5) #maxlen was 30
        self.velocityTracking = velocityTracker()
    
    def appendPosition(self, position:Point):
        self.position_history.appendleft(position)
        self.position = position
    
    def getHistory(self, num_points:int):
        """History, most recent points at the front"""
        total_length = len(self.position_history)

        if num_points >= total_length:
            return self.position_history[::-1]

        max_index = total_length - 1
        stop_index = max_index - num_points

        return self.position_history[max_index:stop_index:-1]
