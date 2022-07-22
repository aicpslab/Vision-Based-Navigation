
from PyQt5.QtWidgets import QMainWindow, QApplication, QLabel, QPushButton, QLineEdit, QDialog, QAction, QMenu, QMenuBar, QTextEdit
import json
from PyQt5 import uic, QtGui
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import numpy as np 
import sys
import cv2
from djitellopy import Tello, TelloSwarm
from PidDroneControl import PidDroneControl
from point import Point
from model import Model
from cvfpscalc import CvFpsCalc
import time
from ids import ID
from generate_circle import generate_circle
import traceback
from generate_star import generate_star

class UI_Single_Mode(QMainWindow):
    """
    Class that handles the UI for the single drone version of the app.
    """

    def __init__(self):
        """ Set Up the UI """

        super(UI_Single_Mode, self).__init__()

        # Load the UI elements from a file. 
        uic.loadUi(ui_single_path, self)
        # self.setFixedSize(1036, 666)

        # Create the Video Stream Thread
        self.stream_thread = StreamThread()
        self.stream_thread.update_img_signal.connect(self.update_image)
        self.stream_thread.stream_off_signal.connect(self.stream_off)

        # Methods to load the widgets we need access to,
        # and connect their buttons. 
        self.load_widgets()
        self.connect_buttons()
        
        # This list will hold the destination points 
        # if the generate circle method is used. 
        self.circle_list = []

        # This list will hold the points the drone has visited 
        # so we can display them as a different color.
        self.visited_list = []

        # Flag for custom list
        self.recording_list = False

        # Show the UI
        self.show()

    def load_widgets(self):
        """
        Method to load the widget objects we use from the UI file.
        """

        # Labels
        self.label_image = self.findChild(QLabel, "label_image")
        self.label_image.mousePressEvent = self.recording_click_event
        self.label_flags = self.findChild(QLabel, "label_flags")
        self.label_flags.setWordWrap(True)

        # Buttons 
        self.pushButton_video_stream = self.findChild(QPushButton, "pushButton_video_stream")
        self.pushButton_program_start = self.findChild(QPushButton, "pushButton_program_start")
        self.pushButton_create_pid = self.findChild(QPushButton, "pushButton_create_pid")
        self.pushButton_emergency_stop = self.findChild(QPushButton, "pushButton_emergency_stop")
        self.pushButton_update_flags = self.findChild(QPushButton, "pushButton_update_flags")
        self.pushButton_drone_connect = self.findChild(QPushButton, "pushButton_drone_connect")
        self.pushButton_drone_disconnect = self.findChild(QPushButton, "pushButton_drone_disconnect")
        self.pushButton_load_presets = self.findChild(QPushButton, "pushButton_load_presets")
        self.pushButton_load_tunings = self.findChild(QPushButton, "pushButton_load_tunings")
        self.pushButton_save_tunings = self.findChild(QPushButton, "pushButton_save_tunings")
        self.pushButton_generate_circle = self.findChild(QPushButton, "pushButton_generate_circle")
        self.pushButton_generate_star = self.findChild(QPushButton, "pushButton_generate_star")
        self.pushButton_record_list = self.findChild(QPushButton, "pushButton_record_list")
        self.pushbutton_finished_recording = self.findChild(QPushButton, "pushButton_finished_recording")

        # Line Edit's 
        self.lineEdit_drone_ip = self.findChild(QLineEdit, "lineEdit_drone_ip")
        self.lineEdit_x_kp = self.findChild(QLineEdit, "lineEdit_x_kp")
        self.lineEdit_x_ki = self.findChild(QLineEdit, "lineEdit_x_ki")
        self.lineEdit_x_kd = self.findChild(QLineEdit, "lineEdit_x_kd")
        self.lineEdit_y_kp = self.findChild(QLineEdit, "lineEdit_y_kp")
        self.lineEdit_y_ki = self.findChild(QLineEdit, "lineEdit_y_ki")
        self.lineEdit_y_kd = self.findChild(QLineEdit, "lineEdit_y_kd")
        self.lineEdit_x_set = self.findChild(QLineEdit, "lineEdit_x_set")
        self.lineEdit_y_set = self.findChild(QLineEdit, "lineEdit_y_set")
        self.lineEdit_cir_centerx = self.findChild(QLineEdit, "lineEdit_cir_centerx")
        self.lineEdit_cir_centery = self.findChild(QLineEdit, "lineEdit_cir_centery")
        self.lineEdit_cir_radius = self.findChild(QLineEdit, "lineEdit_cir_radius")
        self.lineEdit_cir_number = self.findChild(QLineEdit, "lineEdit_cir_num")
        self.lineEdit_cir_centerx_2 = self.findChild(QLineEdit, "lineEdit_cir_centerx_2")
        self.lineEdit_cir_centery_2 = self.findChild(QLineEdit, "lineEdit_cir_centery_2")
        self.lineEdit_cir_radius_2 = self.findChild(QLineEdit, "lineEdit_cir_radius_2")
        # Menu Button
        self.action_swarm = self.findChild(QAction, "action_swarm")
    
    def connect_buttons(self):
        """
        Method to connect the buttons from the UI to the methods they need to call
        """
        self.pushButton_video_stream.clicked.connect(self.stream_thread.clicked_video_stream)
        self.pushButton_program_start.clicked.connect(self.clicked_program_start) 
        self.pushButton_create_pid.clicked.connect(self.clicked_create_pid) 
        self.pushButton_emergency_stop.clicked.connect(self.clicked_emergency_stop) 
        self.pushButton_update_flags.clicked.connect(self.clicked_update_flags) 
        self.pushButton_drone_connect.clicked.connect(self.clicked_drone_connect)
        self.pushButton_drone_disconnect.clicked.connect(self.clicked_drone_disconnect) 
        self.pushButton_load_presets.clicked.connect(self.clicked_load_presets) 
        self.pushButton_load_tunings.clicked.connect(self.clicked_load_tunings)
        self.pushButton_save_tunings.clicked.connect(self.clicked_save_tunings)
        self.pushButton_generate_circle.clicked.connect(self.clicked_generate_circle)
        self.pushButton_generate_star.clicked.connect(self.clicked_generate_star)
        self.pushButton_record_list.clicked.connect(self.clicked_record_list)
        self.pushbutton_finished_recording.clicked.connect(self.clicked_finished_recording)

        self.action_swarm.triggered.connect(self.clicked_view_action_swarm)
    
    def clicked_generate_circle(self):
        """ 
        Responds to the generate_circle button press. 
        Will make a list of points that lie on a circle of 
        center: (center_x, center_y),
        with a radius of radius,
        and contains num number of points. 
        """
        try: 
            # Try to read the values of the text edits 

            # Center Points
            x = int(self.lineEdit_cir_centerx.text())
            y = int(self.lineEdit_cir_centery.text())
            
            # Radius and number of poitns 
            radius = int(self.lineEdit_cir_radius.text())
            num = int(self.lineEdit_cir_number.text())

        except Exception as e:
            print()
            print("There was an error handling clicked_generate_circle:")
            print("\n\n", e, "\n\n")
            
        else:
            # If everything went well

            # Make the center coordinate
            self.cir_center = Point(x,y)

            # Generate the list of points on the circle
            # that is centered at cir_center, with radius of radius, and
            # contains num number of points
            self.circle_list = generate_circle(self.cir_center, radius, num)

            # Set the label for the UI to show the circle points 
            # as the flag points
            self.label_flags.setText(str(self.circle_list))

            # Update the destination points to be the circle_list. 
            self.des_points = self.circle_list

            # Empty the visited signal 
            self.visited_list = []

    def clicked_generate_star(self):
        """ Method to generate a list of points for a star"""
        try: 
            # Try to read the values of the text edits 

            # Center Point
            x = int(self.lineEdit_cir_centerx_2.text())
            y = int(self.lineEdit_cir_centery_2.text())
            
            # Radius
            radius = int(self.lineEdit_cir_radius_2.text())
        except Exception as e:
            print()
            print("There was an error handling clicked_generate_circle:")
            print("\n\n", e, "\n\n")
            
        else:
            # If everything went well

            # Make the center coordinate
            self.cir_center = Point(x,y)

            # Generate the list of points on the circle
            # that is centered at cir_center, with radius of radius, and
            # contains num number of points
            self.circle_list = generate_star(self.cir_center, radius)

            # Set the label for the UI to show the circle points 
            # as the flag points
            self.label_flags.setText(str(self.circle_list))

            # Update the destination points to be the circle_list. 
            self.des_points = self.circle_list

            # Empty the visited signal 
            self.visited_list = []
    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """
        Method to update the label of the video feed label on the UI
        """

        # Check to see if we have generated a circle path. 
        # If so, display that path
        if self.circle_list:
            for point in self.circle_list:
                cv2.circle(cv_img, point, 3, (0,0,255), -1)
        
        # Check to see if we have already visited some of the points on the 
        # circular path. If so, draw those points as green
        if self.visited_list:
            for point in self.visited_list:
                cv2.circle(cv_img, point, 3, (0,255,0), -1)

        # Convert the BGR image to RGB
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        # Convert the image to the PyQT format
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w
        qt_format = QtGui.QImage(rgb_img.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        qt_image = qt_format.scaled(640, 480, Qt.KeepAspectRatio)

        # Set the image on the GUI 
        self.label_image.setPixmap(QPixmap.fromImage(qt_image))
    
    @pyqtSlot()
    def stream_off(self):
        """ Method to turn of the live video feed based on a signal"""
        self.label_image.setPixmap(self.stream_thread.no_stream_img)

    @pyqtSlot(list)
    def update_visited(self, visited):
        """ Method to update the visited list form the program thread """
        self.visited_list = visited

    def clicked_program_start(self):
        """
        This method will respond to the button that starts the program.
        It sets up the ProgramThread class.
        """
        try:

            # If the stream is running, turn it off to prevent concurance issues.
            # The Program Thread will turn it back on. 
            if self.stream_thread._run_flag:
                self.stream_thread._run_flag = False
            
            # Turn off recording mode
            self.recording_list = False
            # Create the ProgramThread object. 
            self.program_thread = SingleProgramThread(self.drone.self, self.pid_controller, self.des_points[:])
            self.program_thread.turn_stream_on_signal.connect(self.stream_thread.clicked_video_stream)
            # The program thread will signal when it has an image to show
            # and when the drone has successfully visited a point. This code
            # connects those singals to their respective functions.
            self.program_thread.update_img_signal.connect(self.update_image)
            self.program_thread.visited_signal.connect(self.update_visited)

            # Start the program thread
            self.program_thread.start()

        except Exception as e:
            print("There was an error during the program start")
            print("\n\n", e, "\n\n")

            # Make sure the program thread stops if it started. 
            try:
                if self.program_thread is not None:
                    self.program_thread._run_flag = False
            except Exception as e:
                print("An error occured while trying to stop the program thread\n\n")
                print(e)
                print("\n\n")

    def clicked_create_pid(self):
        """ 
        Method to create a PID controler object from 
        the appropriate text edit boxes. 
        """

        # Make sure the drone object has been created
        if self.drone is None:
            print("\nPlease connect to your drone\n")
            return 

        # Try to load the the tunings from the text edits
        # Return if there is an error
        try:
            x_kp = float(self.lineEdit_x_kp.text())
            x_ki = float(self.lineEdit_x_ki.text())
            x_kd = float(self.lineEdit_x_kd.text())
            y_kp = float(self.lineEdit_y_kp.text())
            y_ki = float(self.lineEdit_y_ki.text())
            y_kd = float(self.lineEdit_y_kd.text())
            spx = self.lineEdit_x_set.text()
            spy = self.lineEdit_y_set.text()
        except Exception as e:
            print("There was an error in getting the tunings")
            print("\n\n", e, "\n\n")
            return
        
        # This is the optional parameter for a custom setpoint.
        if spx and spy:
            try:
                spx = int(spx)
                spy = int(spy)
                setpoint = Point(spx, spy)
            except Exception as e:
                print("There was an error getting the optional setpoints")
                print("\n\n", e, "\n\n")
                return

        # If the optional points aren't there, then make sure the des_points list is populated
        # and set the first point as the set point for the controller.
        elif self.des_points:
            setpoint = self.des_points[0]
        
        # Try to create the controller object using the information gathered. 
        try:
            self.pid_controller = PidDroneControl(self.drone.self, setpoint, 0, 0, 0, 0)
            self.pid_controller.pidx.tunings = (x_kp, x_ki, x_kd)
            self.pid_controller.pidy.tunings = (y_kp, y_ki, y_kd)

            print("\n", "The tunings for the pid controller are:")
            print(self.pid_controller.pidx.tunings)
            print(self.pid_controller.pidy.tunings)
        except Exception as e:
            print("There was an error in creating the PID controller")
            print("\n\n", e, "\n\n")
            return
        
    def clicked_emergency_stop(self):
        """
        Method that will try to gracefully shut 
        processes down in the event of a failure and 
        the emergency stop button was clicked.
        """

        # Try to land the drone 
        try:
            self.pid_controller.land()
        except Exception as e:
            print("An error occured while trying to land the drone")
            print("\n\n", e, "\n\n")

        # Try to stop the program thread from running
        try:
            if self.program_thread._run_flag:
                self.program_thread._run_flag = False
        except Exception as e:
            print("An error occured while trying to stop the program thread.")
            print("\n\n", e, "\n\n")
        
        # Set the visited list back to empty
        self.visited_list = []

        # Turn back on the normal stream
        self.stream_thread.start()

    def clicked_update_flags(self):
        """ Method that will try to update the flags """
        
        # Normal mode with the model
        if "no_model" not in sys.argv:
            self.label_flags.setText(str(Mod.flag_centers))
            self.des_points = Mod.flag_centers
            self.circle_list = self.des_points
        # No Model mode / Testing Mode
        else:
            self.des_points = [Point(320,240)]
            self.label_flags.setText(str(self.des_points))
            self.circle_list = self.des_points
    
    def clicked_drone_connect(self):
        """ 
        Method to use the ip from the text edit
        to connect to a drone
        """
        try:
            ip = self.lineEdit_drone_ip.text()
            self.drone = Drone(ip)
        except Exception as e:
            print("There was an error connecting to the drone")
            print("\n\n", e, "\n\n")
        
    def clicked_drone_disconnect(self):
        """
        Method the disconnect from the drone 
        """
        try:
            self.drone.disconnect()
            self.drone = None
        except Exception as e:
            print("There was an error disconnecting from the drone")
            print("\n\n", e, "\n\n")
    
    def clicked_load_presets(self):
        """
        Method to load common text edit values
        """

        self.lineEdit_drone_ip.setText(str(drone_ips[0]))
        self.lineEdit_x_kp.setText(".2")
        self.lineEdit_x_ki.setText("0")
        self.lineEdit_x_kd.setText("0")
        self.lineEdit_y_kp.setText(".2")
        self.lineEdit_y_ki.setText("0")
        self.lineEdit_y_kd.setText("0")
        self.lineEdit_cir_centerx.setText("320")
        self.lineEdit_cir_centery.setText("240")
        self.lineEdit_cir_radius.setText("100")
        self.lineEdit_cir_number.setText("10")

    def clicked_view_action_swarm(self):
        """
        Method to open up the multi drone GUI.
        """
        self.hide()
        ui_swarm_mode = UI_Swarm_Mode(self)
        ui_swarm_mode.show()
    
    def clicked_load_tunings(self):
        """
        Method to grab the saved tunings from a json file
        and set them.
        """
        with open('pid_tunings.json', 'r') as f:
            tunings = json.load(f)

        self.lineEdit_x_kp.setText(tunings['x_kp'])
        self.lineEdit_x_ki.setText(tunings['x_ki'])
        self.lineEdit_x_kd.setText(tunings['x_kd'])
        self.lineEdit_y_kp.setText(tunings['y_kp'])
        self.lineEdit_y_ki.setText(tunings['y_ki'])
        self.lineEdit_y_kd.setText(tunings['y_kd'])
        self.lineEdit_cir_centerx.setText(tunings["c_x"])
        self.lineEdit_cir_centery.setText(tunings["c_y"])
        self.lineEdit_cir_radius.setText(tunings["r"])
        self.lineEdit_cir_number.setText(tunings["n"])

    def clicked_save_tunings(self):
        """
        Method to save tunings to a json file
        """
        tunings = {
            'x_kp': self.lineEdit_x_kp.text(),
            'x_ki': self.lineEdit_x_ki.text(),
            'x_kd': self.lineEdit_x_kd.text(),
            'y_kp': self.lineEdit_y_kp.text(),
            'y_ki': self.lineEdit_y_ki.text(),
            'y_kd': self.lineEdit_y_kd.text(),
            'c_x': self.lineEdit_cir_centerx.text(),
            'c_y': self.lineEdit_cir_centery.text(),
            'r': self.lineEdit_cir_radius.text(),
            'n': self.lineEdit_cir_number.text()
        }

        with open('pid_tunings.json', 'w') as f:
            json.dump(tunings, f)

    def clicked_record_list(self):
        """Method to set up recording a list"""
        self.recording_list = True
        self.circle_list = []
        # Set the label for the UI to show the circle points 
        # as the flag points
        self.label_flags.setText(str(self.circle_list))

        # Update the destination points to be the circle_list. 
        self.des_points = self.circle_list
    
    def clicked_finished_recording(self):
        """ Method to turn of recording a list """
        self.des_points[1:] = self.des_points[1:][::-1]
        print(self.des_points)
        self.recording_list = False
    
    def recording_click_event(self, event):
        """
        If the program is in the state of recording a list,
        append the point of the click to the list. 
        """
        x = event.pos().x()
        y = event.pos().y()
        if self.recording_list:
            if event.button() == Qt.LeftButton: 
                self.circle_list.append(Point(x,y))

        self.label_flags.setText(str(self.circle_list))
        # Update the destination points to be the circle_list. 
        self.des_points = self.circle_list

class UI_Swarm_Mode(QMainWindow):
    
    def __init__(self, parent = None):
        super(UI_Swarm_Mode, self).__init__(parent)
        uic.loadUi(ui_swarm_path, self)

        self.stream_thread = StreamThread()
        self.stream_thread.update_img_signal.connect(self.update_image)
        self.stream_thread.stream_off_signal.connect(self.stream_off)

        self.load_widgets()
        self.connect_buttons()

    def load_widgets(self):

        self.action_single = self.findChild(QAction, "action_single")

        # Labels 
        self.label_image = self.findChild(QLabel, "label_image")
        self.label_flags = self.findChild(QLabel, "label_flags")
        self.label_flags.setWordWrap(True)

        # Buttons 
        self.pushButton_video_stream = self.findChild(QPushButton, "pushButton_video_stream")
        self.pushButton_program_start = self.findChild(QPushButton, "pushButton_program_start")
        self.pushButton_create_pid = self.findChild(QPushButton, "pushButton_create_pid")
        self.pushButton_emergency_stop = self.findChild(QPushButton, "pushButton_emergency_stop")
        self.pushButton_update_flags = self.findChild(QPushButton, "pushButton_update_flags")
        self.pushButton_drone_connect = self.findChild(QPushButton, "pushButton_drone_connect")
        self.pushButton_drone_disconnect = self.findChild(QPushButton, "pushButton_drone_disconnect")
        self.pushButton_load_presets = self.findChild(QPushButton, "pushButton_load_presets")
        self.pushButton_show_info = self.findChild(QPushButton, "pushButton_show_info")

        # Line Edit's 
        self.lineEdit_x_kp = self.findChild(QLineEdit, "lineEdit_x_kp")
        self.lineEdit_x_ki = self.findChild(QLineEdit, "lineEdit_x_ki")
        self.lineEdit_x_kd = self.findChild(QLineEdit, "lineEdit_x_kd")
        self.lineEdit_y_kp = self.findChild(QLineEdit, "lineEdit_y_kp")
        self.lineEdit_y_ki = self.findChild(QLineEdit, "lineEdit_y_ki")
        self.lineEdit_y_kd = self.findChild(QLineEdit, "lineEdit_y_kd")
        
        # Text Edits 
        self.textEdit_drone_info = self.findChild(QTextEdit, "textEdit_drone_info")
        self.textEdit_controllers_info = self.findChild(QTextEdit, "textEdit_controllers_info")
        self.textEdit_navigation_info = self.findChild(QTextEdit, "textEdit_navigation_info")

    def connect_buttons(self):
        
        self.action_single.triggered.connect(self.clicked_view_action_swarm)

        self.pushButton_video_stream.clicked.connect(self.stream_thread.clicked_video_stream)
        self.pushButton_program_start.clicked.connect(self.clicked_program_start)
        self.pushButton_create_pid.clicked.connect(self.clicked_create_pid)
        self.pushButton_emergency_stop.clicked.connect(self.clicked_emergency_stop)
        self.pushButton_update_flags.clicked.connect(self.clicked_update_flags)
        self.pushButton_drone_connect.clicked.connect(self.clicked_drone_connect) 
        self.pushButton_drone_disconnect.clicked.connect(self.clicked_drone_disconnect) 
        self.pushButton_load_presets.clicked.connect(self.clicked_load_presets) 
        self.pushButton_show_info.clicked.connect(self.clicked_show_info)

    @pyqtSlot(str)
    def update_navigation_text(self, nav_str):
        self.textEdit_navigation_info.setPlainText(nav_str)

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w
        qt_format = QtGui.QImage(rgb_img.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        qt_image = qt_format.scaled(640, 480, Qt.KeepAspectRatio)
        self.label_image.setPixmap(QPixmap.fromImage(qt_image))
    
    @pyqtSlot()
    def stream_off(self):
        self.label_image.setPixmap(self.stream_thread.no_stream_img)

    def clicked_program_start(self):
        try:
            self.program_thread = SwarmProgramThread(self.swarm, self.controllers, self.des_points)
            self.program_thread.update_navigation_signal.connect(self.update_navigation_text)
            self.program_thread.start()
        except Exception as e:
            print(e)

    def clicked_create_pid(self):

        self.controllers = []
        try:
            x_kp = float(self.lineEdit_x_kp.text())
            x_ki = float(self.lineEdit_x_ki.text())
            x_kd = float(self.lineEdit_x_kd.text())
            y_kp = float(self.lineEdit_y_kp.text())
            y_ki = float(self.lineEdit_y_ki.text())
            y_kd = float(self.lineEdit_y_kd.text())
        except Exception as e:
            print(e)
            print("Using Default vaules due to error...")
            x_kp = 0.20
            x_ki = 0.05
            x_kd = 0.05
            y_kp = 0.20
            y_ki = 0.05
            y_kd = 0.05
        
        try:
            for ind, ip in enumerate(drone_ips):
                drone = self.drones[ind]
                des = self.des_points.pop()
                controller = PidDroneControl(drone, des, ind, x_kp, x_ki, x_kd)
                controller.pidx.tunings = (x_kp, x_ki, x_kd)
                controller.pidy.tunings = (y_kp, y_ki, y_kd)
                self.controllers.append(controller)
            
            print(self.controllers)
        except Exception as e:
            print(e)
        
        #self.textEdit_controllers_info.setPlainText(f"{self.controllers}")
        #self.textEdit_drone_info.setPlainText(f"{self.drones}\n {self.swarm}")

    def clicked_emergency_stop(self):
        try:
            self.program_thread._run_flag = False
            self.swarm.land()
        except Exception as e:
            print(e)

    def clicked_update_flags(self):
        if "no_model" not in sys.argv:
            self.label_flags.setText(str(Mod.flag_centers))
            self.des_points = Mod.flag_centers
        else:
            self.des_points = [Point(320,240)]
            self.label_flags.setText(str(self.des_points))

    def clicked_drone_connect(self):
        try:
            self.drones=[]

            for ip in drone_ips:
                drone = Tello(ip, retry_count=1)
                drone.connect()
                self.drones.append(drone)

            self.swarm = TelloSwarm(self.drones)

        except Exception as e:
            print(e)

    def clicked_drone_disconnect(self):
        try:
            for drone in self.drones:
                drone.end()
            self.drones = []
            self.swarm = None
        except Exception as e:
            print(e)

    def clicked_load_presets(self):
        self.lineEdit_x_kp.setText("0.20")
        self.lineEdit_x_ki.setText("0.05")
        self.lineEdit_x_kd.setText("0.05")
        self.lineEdit_y_kp.setText("0.20")
        self.lineEdit_y_ki.setText("0.05")
        self.lineEdit_y_kd.setText("0.05")

    def clicked_view_action_swarm(self):
        self.parent().show()
        self.close()
    
    def clicked_show_info(self):
        try:
            self.textEdit_drone_info.setPlainText(f"{self.drones}")
        except Exception as e:
            print(e)
        try:
            string = ""
            for ind, controller in enumerate(self.controllers):
                string += f"Controller of index {ind}:\n"
                string += f"Navigating to: ({controller.pidx.setpoint}, {controller.pidy.setpoint})\n"
                string += f"X Tunings: {controller.pidx.tunings}"
                string += f"Y Tunings: {controller.pidy.tunings}\n"
            self.textEdit_controllers_info.setPlainText(f"{string}")
        except Exception as e:
            print(e)

class StreamThread(QThread):
    """
    Class that Handles the streaming when the controller is not needed
    """

    # Signals for the image and no image 
    update_img_signal = pyqtSignal(np.ndarray)
    stream_off_signal = pyqtSignal()

    def __init__(self):
        """
        Set up the class 
        """
        super().__init__()
        
        # Set a run flag that we can access in the GUI part of
        # the program
        self._run_flag = False

        # Create a image for use when there is no stream
        no_stream_img = QtGui.QImage(640, 480, QtGui.QImage.Format_Indexed8)
        no_stream_img.fill(QtGui.qRgb(128, 128, 128))
        self.no_stream_img = QPixmap.fromImage(no_stream_img)
    
    def run(self):
        """
        Run the main function of this thread. 
        """

        self._run_flag = True

        # Normal mode with the model loaded
        if "no_model" not in sys.argv:
            while self._run_flag:
                # Do a detection and update the image 
                img = Mod.basic_detection()
                self.update_img_signal.emit(img)
            else:
                self.stream_off_signal.emit()
        else:
            # Without Model
            while self._run_flag:
                _, img = cam.read()
                self.update_img_signal.emit(img)
            else:
                self.stream_off_signal.emit()
    
    def clicked_video_stream(self):
        """ Toggle the stream thread based on a button press """
        if self._run_flag:
            self._run_flag = False
        else:
            self.start()
 
class Drone():
    """ Class to handle the connection to the drone """

    def __init__(self, ip:str):
        self.ip = ip
        self.self = Tello(self.ip, retry_count = 1)
        self.connect()
    
    def __repr__(self):
        return f"Drone({self.ip}, self.self= Tello({self.ip}))"

    def connect(self):
        try:
            self.self.connect()
            print(f'Connected to {self.ip}')
        except Exception as e:
            print(e)
    
    def disconnect(self):
        self.self.end()
        print(f'Ended Connection to {self.ip}')

class SingleProgramThread(QThread):
    """
    Class that handles the main program logic.
    This class is responsible for controlling the drone to the set points
    defined by the destination list. 
    """
    
    # A signal to update the points the drone has visisted to the main GUI 
    visited_signal = pyqtSignal(list)
    # The stream thread will no longer handle the webcam, so this thread also needs a image signal. 
    update_img_signal = pyqtSignal(np.ndarray)
    turn_stream_on_signal = pyqtSignal()

    def __init__(self, drone:Tello, controller: PidDroneControl, des_points):
        """
        Set up the class 

        drone: A tello object 
        controller: Our Custom PidDroneControl object
        des_points: A list of points for the drone to visit. 
        """
        super().__init__()
        
        self._run_flag = False
        self.drone: Tello = drone
        self.controller = controller
        self.des_points = des_points
        self.visited = []

    def run(self):
        self._run_flag = True
        
        try:
            # Allow the drone to take off.
            self.drone.takeoff()
            des = self.des_points[0]

            while self._run_flag:
                
                # Use our tensorflow model and custom Model class to make a detection 
                img = Mod.basic_detection()
                # Emit the image for the video stream 
                self.update_img_signal.emit(img)

                # If the controler needs a new point 
                if self.controller.new_point:

                    # Make sure there are still destination points
                    if self.des_points:

                        # Update the visited points with the last destination
                        self.visited.append(des)
                        self.visited_signal.emit(self.visited)

                        # Get a new destination and update the controller 
                        des = self.des_points.pop()
                        self.controller.update_setpoints(des)

                    # If there are not still destination points left 
                    else:
                        # Land the drone 
                        self.drone.land()
                        
                        self.turn_stream_on_signal.emit()
                        # Stop running this loop
                        self._run_flag = False

                # If the controller doesn't need a new point 
                else:
                    
                    # Update the velocity
                    self.controller.update_velocity()

        except Exception as e:
            print('There was an error trying to run the main program')
            print("\n\n", e, "\n\n")   
            print(traceback.format_exc())
            self._run_flag = False

class SwarmProgramThread(QThread):
    update_navigation_signal = pyqtSignal(str)

    def __init__(self, swarm, controllers, des_points):
        super().__init__()
        self._run_flag = False
        self.controllers = controllers
        self.des_points = des_points
        self.swarm = swarm
    
    def run(self):
        self._run_flag = True

        self.swarm.takeoff()

        while self._run_flag:
            for controller in self.controllers:
                if not controller.new_point:
                    controller.update_velocity()
                else:
                    if self.des_points:
                        des = self.des_points.pop()
                        controller.update_setpoints(des)
                        self.update_nav_string()
                    else:
                        controller.drone.land()
                        self.controllers.remove(controller)
            if self.controllers == []:
                self._run_flag = False

    def update_nav_string(self):
        nav_string = ""
        for ind, controller in enumerate(self.controllers):
            nav_string += f"Controller {ind} is navigating to: {controller.setpoint}\n"

        self.update_navigation_signal.emit(nav_string)

class GraphInfo(QThread):
    update_plots_signal = pyqtSignal(tuple)

    def __init__(self):
        super().__init__()
        self.x_list = [0]
        self.y_list = [0]
        self.time_list = [0]
        self._run_flag = False
    
    def run(self):
        self._run_flag = True
        self.start_time = time.time()
        try:
            while self._run_flag: 
                position = ID.instances[0].position
                # self.x_list = self.x_list[1:]
                # self.y_list = self.y_list[1:]
                # self.time_list = self.time_list[1:]
                self.x_list.append(position.x)
                self.y_list.append(position.y)
                self.time_list.append(time.time()-self.start_time)
                self.update_plots_signal.emit((self.x_list, self.y_list, self.time_list))
                time.sleep(0.1)
        except Exception as e:
            print(e)
            self._run_flag = False
    
    def stop(self):
        self._run_flag = False

# Initialize the app
if __name__ == "__main__":

    drone_ips = ['192.168.1.100', '192.168.1.200']
    cam_index = 1

    app = QApplication(sys.argv)

    if "no_model" in sys.argv:
        cam = cv2.VideoCapture(0)
    else:
        Mod = Model(cam_index, low_memory=True)

    ui_single_path = "./resources/gui_views/single_drone.ui"
    ui_swarm_path = "./resources/gui_views/swarm_drone.ui"
    UIWindow = UI_Single_Mode()
    
    sys.exit(app.exec_())