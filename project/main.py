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
from simple_pid import PID

# Custom Imports
from model import Model as Mod
from ids import ID
from PidDroneControl import PidDroneControl


def TelloSwarmScript(drone_objs):
    """ Commands to send to the drones """

    def do_stuff(i, tello):
        # if i == 0:
        #     tello.curve_xyz_speed(70,70,0,140,0,0,20)
        #     swarm.sync()
        #     tello.curve_xyz_speed(-70,-70,0,-140,0,0,20)
        #     swarm.sync()
        # elif i == 1:
        #     tello.curve_xyz_speed(-70,-70,0,-140,0,0,20)
        #     swarm.sync()
        #     tello.curve_xyz_speed(70,70,0,140,0,0,20)
        #     swarm.sync()
        tello.move_forward(20)
        tello.rotate_clockwise(90)
        tello.move_forward(20)
        tello.rotate_clockwise(90)
        tello.move_forward(20)

    time.sleep(2)
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
    def program_exit():
        cv2.destroyAllWindows()
        Model.cam.release()
        executor.shutdown(wait=False)
        #swarm.land()
        drone.send_rc_control(0,0,0,0)
        #drone2.send_rc_control(0,0,0,0)
        drone.land()
        #drone2.land()
        #Controller.export_to_json()

    Model = Mod()

    #drone_objs = TelloConnection()
    #drone, drone2, swarm = drone_objs
    #pid_args = (Model, drone_objs)

    drone = Tello('192.168.1.200')
    drone.connect()

    #drone2 = Tello('192.168.1.200')
    #drone2.connect()

    with concurrent.futures.ThreadPoolExecutor() as executor:
        executor.submit(Model.basic_detection_thread)

        # Verify Points 
        while True:
            destination_points = Model.flag_centers
            print(destination_points)
            img = Model.thread_img 
            for (x,y) in destination_points:
                img = cv2.circle(img, (x,y) , 10, (0,0,255), -1)
            
            cv2.imshow("Press S to start the program.\nPress any button to try again", img)
            if cv2.waitKey(0) == ord('s'):
                cv2.destroyAllWindows()
                break
        

        # First Drone 
        des = destination_points.pop()
        Controller = PidDroneControl(drone, des, 0)
        
        # Second Drone
        #Controller_2 = PidDroneControl(drone2, x_set_2, y_set_2, 1)

        drone.takeoff()
        #drone2.takeoff()

        while True:
            if not Controller.new_point:
                executor.submit(Controller.update_velocity)
            else:
                if destination_points:
                    des = destination_points.pop()
                    Controller.update_setpoints(des)
                else:
                    program_exit()
                    break

            img = Model.thread_img
            cv2.putText(img, "D2D: {}".format(int(Controller.d2d)), (5,480-5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            #cv2.putText(img, "D2D: {}".format(int(Controller.d2d)), (15,480-5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
            cv2.circle(img, (des.x, des.y), 4, (0,0,255), -1)
            cv2.imshow('Test', img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            
        


if __name__ == '__main__':
    main_with_threading()
       