"""
Vision Based Navigation Project
Augusta University
3/11/2022

This file contains the DrawingConstants
class that is responsible for 
giving names to colors that represent 
tuples and other numbers used in 
cv2 function calls. 
It helps make cv2 function calls
more readable. 

drawing.py
"""
# Python imports
import cv2
from cv2 import FONT_HERSHEY_SIMPLEX


class DrawingConstants:
    """ 
    Class to give names to colors and other numbers
    that are used when using Cv2 Methods. 
    Makes Cv2 function calls more readable 
    """
    # colors
    red: tuple = (255, 0, 0)
    green: tuple = (0, 255, 0)
    blue: tuple = (0, 0, 255)
    pink: tuple = (255, 0, 255)
    gb: tuple = (0, 255, 255)
    black: tuple = (0, 0, 0)
    light_purple: tuple = (238, 87, 255)
    # fonts 
    font: int = FONT_HERSHEY_SIMPLEX
    font_scale: int = 1
    font_thickness: int = 1
    # circles
    circle_radius: int = 5
    circle_thickness: int = -1
    # lines
    line_thickness: int = 3
