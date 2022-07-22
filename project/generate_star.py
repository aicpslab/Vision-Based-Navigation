import math
from typing import OrderedDict
from point import Point
from matplotlib import pyplot as plt
import cv2


def generate_star(c:Point, r):
    """
    Function to generate a list of points on a circle.

    c: Center
    r: Raidus of the circle for the outer 5 points
    """
    step = 2 * math.pi / 5
    
    r2 = int((1/3)*r)

    small = []
    big = []
    angle = 0 

    ordered_list = []

    for _ in range(5):

        x2 = int(r2 * math.cos(angle) + c.x)
        y2 = int(r2 * math.sin(angle) + c.y)

        x = int(r * math.cos(angle + (math.pi/4)) + c.x)
        y = int(r * math.sin(angle + (math.pi/4)) + c.y)

        big.append(Point(x, y))
        small.append(Point(x2,y2))
        angle += step

    temp = small[0]
    small = small[1:]
    small.append(temp)

    for (big,small) in zip(big,small):
        ordered_list.append(big)
        ordered_list.append(small)

    return ordered_list

if __name__ == "__main__":


    cam = cv2.VideoCapture(0)
    _, img = cam.read()
    print(img.shape)
    for x in range(20):
        _,img = cam.read()

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    ordered_list = generate_star(Point(320, 240), 200)

    for point in ordered_list:
        img = cv2.circle(img, (point.x, point.y), 2, (0,0,255), -1)
        cv2.imshow('test', img)
        cv2.waitKey(0)

    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imshow('test', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows
    cam.release()