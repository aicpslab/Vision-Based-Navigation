import math
from point import Point
from matplotlib import pyplot as plt
import cv2


def generate_circle(c:Point, r, num):
    """
    Function to generate a list of points on a circle.

    c: Center
    r: Raidus
    num: Number of points desired
    """
    step = 2 * math.pi / num

    point_list = []
    angle = 0 

    for _ in range(num):

        x = int(r * math.cos(angle) + c.x)
        y = int(r * math.sin(angle) + c.y)

        point_list.append(Point(x, y))
        angle += step

    return point_list

if __name__ == "__main__":


    cam = cv2.VideoCapture(0)
    _, img = cam.read()
    print(img.shape)
    for x in range(20):
        _,img = cam.read()

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    my_circle = generate_circle(Point(320, 240), 100, 25)

    import pprint 
    pprint.pprint(my_circle)

    for point in my_circle:
        img = cv2.circle(img, (point.x, point.y), 2, (0,0,255), -1)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imshow('test', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows
    cam.release()

