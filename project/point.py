from dataclasses import dataclass
from math import sqrt
from collections import Sequence
from re import X

@dataclass
class Point(Sequence):
    
    x : int
    y : int
    
    def __getitem__(self, key):
        if key == 0: return self.x
        elif key == 1: return self.y
        else: raise IndexError()
        
    def __len__(self):
        return 2

    def __add__(self, other):
        """Point(1, 1) + Point(2, 2) = Point(3, 3)"""
        if not isinstance(other, Point):
            return NotImplemented
        else:
            return Point(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        """Point(1, 1) - Point(2, 2) = Point(-1, -1)"""
        if not isinstance(other, Point):
            return NotImplemented
        else:
            return Point(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        """Point(2, 3) * Point(2, 2) = Point(4, 6)
           Point(2,2) * 5 = Point(10, 10)      """
        if isinstance(other, Point):
            return Point(self.x * other.x, self.y * other.y)
        elif isinstance(other, int):
            return Point(other * self.x, other * self.y)
        else:
            return NotImplemented
    
    def dot(self, other):
        """Dot Product multiplication"""
        if not isinstance(other, Point):
            return NotImplemented
        else:
            return self.x*other.x + self.y*other.y
           
    def determinate(self, other): 
        """Determinate"""
        if not isinstance(other, Point):
            return NotImplemented
        else: 
            return self.x*other.y - self.y*other.x 

    def dist(self, other):
        """Euclidian Distance"""
        if not isinstance(other, Point):
            return NotImplemented
        else:
            return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
