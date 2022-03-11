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
# Custom Imports
from model import Model as Mod


def TelloSwarmScript(swarm):
    """ Commands to send to the drones """
    swarm.takeoff()
    swarm.parallel(lambda i,
                   tello: tello.curve_xyz_speed(25, -25, 0, 25, -75, 0, 20))
    swarm.land()

def TelloConnection():
    """ Function to connect to the drones """
    drone = Tello('192.168.1.200')
    drone2 = Tello('192.168.1.100')

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

    #drone, drone2, swarm = TelloConnection()

    with concurrent.futures.ThreadPoolExecutor() as executor:
        executor.submit(Model.basic_detection_thread)

        while True:
            cv2.imshow('Test', Model.thread_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()
        Model.cam.release()
        executor.shutdown(wait=False)

if __name__ == '__main__':
    main_with_threading()
        
