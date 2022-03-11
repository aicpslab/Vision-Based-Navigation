"""
Vision Based Navigation Project
Augusta University
3/11/2022

This is the entry point into the tracking program.

main.py
"""
# Python Imports
import concurrent.futures
import cv2 
from djitellopy import Tello
from djitellopy import TelloSwarm
import time
# Custom Imports
from model import Model as Mod


def TelloSwarmScript(drone_objs):
    """ Commands to send to the drones """

    def do_stuff(i, tello):
        if i == 0:
            tello.curve_xyz_speed(70,70,0,140,0,0,20)
            swarm.sync()
            tello.curve_xyz_speed(-70,-70,0,-140,0,0,20)
            swarm.sync()
        elif i == 1:
            tello.curve_xyz_speed(-70,-70,0,-140,0,0,20)
            swarm.sync()
            tello.curve_xyz_speed(70,70,0,140,0,0,20)
            swarm.sync()

    time.sleep(5)
    drone, drone2, swarm = drone_objs
    swarm.takeoff()
    swarm.parallel(do_stuff)
    swarm.land()

def TelloConnection():
    """ Function to connect to the drones """
    drone = Tello('192.168.1.100')
    drone2 = Tello('192.168.1.200')

    swarm = TelloSwarm([drone, drone2])
    swarm.connect()

    return [drone, drone2, swarm]

def main_with_threading():
    """ Main Function
        Creates a thread for tracking, 
        drone control, 
        and a cv2 windows to view the tracking.
    """
    Model = Mod()

    drone_objs = TelloConnection()
    drone, drone2, swarm = drone_objs

    with concurrent.futures.ThreadPoolExecutor() as executor:
        executor.submit(Model.basic_detection_thread)
        executor.submit(TelloSwarmScript, drone_objs)

        while True:
            cv2.imshow('Test', Model.thread_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()
        Model.cam.release()
        executor.shutdown(wait=False)
        swarm.land()

if __name__ == '__main__':
    main_with_threading()
       