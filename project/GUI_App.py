import concurrent.futures 
from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget, QApplication, QLabel, QVBoxLayout, QPushButton, QLineEdit, QGridLayout, QHBoxLayout
from PyQt5.QtGui import QPixmap
import sys
import cv2
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import numpy as np
from model import Model
from djitellopy import Tello, TelloSwarm
from PidDroneControl import PidDroneControl
from point import Point
import sys

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    no_change_signal = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self._run_flag = True
        #self.cam = cv2.VideoCapture(0)

    def run(self):
        # capture from web cam
        while self._run_flag:
            img = Mod.baisc_detection()
            #_ret, img = self.cam.read()
            self.change_pixmap_signal.emit(img)
        else:
            self.no_change_signal.emit()
    
    def stream_button_click(self):
        if self._run_flag:
            self._run_flag = False
        else:
            self._run_flag = True
            self.start()
    
    def shut_down(self):
        self._run_flag = False
        self.wait()

class DroneConnect(QThread):

    def __init__(self, ip:str):
        super().__init__()
        self.ip = ip
        self.drone = None
        self.start()
        
    def run(self):
        self.connect()

    def connect(self):
        try:
            self.drone = Tello(self.ip, retry_count=2)
            self.drone.connect()
        except Exception as e:
            print(e)
    
    def disconnect(self):
        if self.drone is None:
            print('No Drone Object')
        else:
            self.drone.end()
            print(f'Ended Connection to {self.ip}.')

class ControllerThread(QThread): 
    def __init__(self):
        super().__init__()
        self._run_flag = False
        self.controllers = None
        self.des_points = None
        self.swarm = None
    
    def run(self):
        self._run_flag = True  
        self.swarm.takeoff()
        with concurrent.futures.ThreadPoolExecutor() as Executor:
            while self._run_flag:
                for controller in self.controllers:
                    if not controller.new_point:
                        Executor.submit(controller.update_velocity) 
                    else:
                        if self.des_points:
                            des = self.des_points.pop()
                            controller.update_setpoints(des)
                        else:
                            controller.drone.land()
                        
        

class ControlDrone(QThread):

    def __init__(self):
        super().__init__()
        self._run_flag = False
        self.drone: Tello = None
        self.controller: PidDroneControl = None
        self.des_points = None

    def run(self):
        self._run_flag = True
        
        self.drone.takeoff()
        while self._run_flag: 
            if not self.controller.new_point:
                self.controller.update_velocity()
            else:
                if self.des_points:
                    des = self.des_points.pop()
                    self.controller.update_setpoints(des)
                else:
                    self.drone.land()
                    self._run_flag = False

    
class App(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Controller")
        self.disply_width = 640
        self.display_height = 480
        # create the label that holds the image
        self.image_label = QLabel(self)
        self.image_label.resize(self.disply_width, self.display_height)
        # create a text label
        self.textLabel = QLabel('Webcam')
        self.flag_points_label1 = QLabel('The Flag Points are: ')
        self.flag_points_lable2 = QLabel(str(Mod.flag_centers))#QLabel(str([(20,20)])) 

        # Stream and Drone Stuff
        self.stream_button = QPushButton('Start/Stop Stream', self)
        self.tello_ip_text_box = QLineEdit(self)
        self.tello_ip_button = QPushButton('Create Tello Object with IP', self)
        self.tello_ip_button.clicked.connect(self.tello_ip_button_clicked)
        self.drone: DroneConnect = None
        self.tello_disconnect_button = QPushButton('Disconnect from Current Drone', self)
        self.tello_disconnect_button.clicked.connect(self.drone_disconnect)
        self.tello_make_multiple_drones_button = QPushButton('Make Drone Objects', self)
        self.tello_make_multiple_drones_button.clicked.connect(self.make_drone_objects)

        # Pid Stuff
        self.pid_label = QLabel('PID Control Values: Kp, Ki, Kd')
        self.kp_tb = QLineEdit(self)
        self.ki_tb = QLineEdit(self)
        self.kd_tb = QLineEdit(self)
        self.pid_label_2 = QLabel('Optional: Setpoint X, Setpoint Y')
        self.spx_tb = QLineEdit(self)
        self.spy_tb = QLineEdit(self)
        self.create_pid_button = QPushButton('Create PID Controller', self)
        self.create_pid_button.clicked.connect(self.create_pid)
        self.pid_controller = None
        self.setpoint : Point = Point(320,240)

        # Flag verification Button
        self.flag_button = QPushButton('Update Flags', self)
        self.flag_button.clicked.connect(self.update_flags)

        # Run program button 
        self.run_program_button = QPushButton('Start Program', self)
        if 'swarm' in sys.argv:
            self.main_program = ControllerThread()
            self.run_program_button.clicked.connect(self.start_program_swarm) 
        else:
            self.main_program = ControlDrone()
            self.run_program_button.clicked.connect(self.start_program_single)

        # create a grid box layout and add widgets
        self.mainBox = QVBoxLayout()
        self.firstRow = QHBoxLayout()
        self.secondRow = QHBoxLayout()
        self.thirdRow = QHBoxLayout()

        self.webCamBox = QVBoxLayout()
        self.webCamBox.addWidget(self.image_label)
        self.webCamBox.addWidget(self.textLabel)
        self.webCamBox.addWidget(self.stream_button)
        self.firstRow.addLayout(self.webCamBox)

        self.droneBox = QVBoxLayout()
        self.droneBox.addWidget(self.tello_ip_button)
        self.droneBox.addWidget(self.tello_ip_text_box)
        self.droneBox.addWidget(self.tello_disconnect_button)
        self.droneBox.addWidget(self.tello_make_multiple_drones_button)
        self.secondRow.addLayout(self.droneBox)

        self.PIDBox = QVBoxLayout()
        self.PIDBox.addWidget(self.pid_label)
        self.PIDBox.addWidget(self.kp_tb)
        self.PIDBox.addWidget(self.ki_tb)
        self.PIDBox.addWidget(self.kd_tb)
        self.PIDBox.addWidget(self.pid_label_2)
        self.PIDBox.addWidget(self.spx_tb)
        self.PIDBox.addWidget(self.spy_tb)
        self.PIDBox.addWidget(self.create_pid_button)
        self.secondRow.addLayout(self.PIDBox)

        self.flagBox = QVBoxLayout()
        self.flagBox.addWidget(self.flag_points_label1)
        self.flagBox.addWidget(self.flag_points_lable2)
        self.flagBox.addWidget(self.flag_button)
        self.secondRow.addLayout(self.flagBox)

        self.run_program_box = QVBoxLayout()
        self.run_program_box.addWidget(self.run_program_button)
        self.thirdRow.addLayout(self.run_program_box)
        
        # Set the Layout of the widget 
        self.mainBox.addLayout(self.firstRow)
        self.mainBox.addLayout(self.secondRow)
        self.mainBox.addLayout(self.thirdRow)
        self.setLayout(self.mainBox)

        # create the video capture thread
        self.thread = VideoThread()
        self.stream_button.clicked.connect(self.thread.stream_button_click)
        # create a blank image
        no_stream_img = QtGui.QImage(640, 480, QtGui.QImage.Format_Indexed8)
        no_stream_img.fill(QtGui.qRgb(128, 128, 128))
        self.no_stream_img = QPixmap.fromImage(no_stream_img)
        # connect its signal to the update_image slot
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.no_change_signal.connect(self.no_stream)
        # start the thread
        self.thread.start()
    
    @pyqtSlot()
    def update_flags(self):
        self.flag_points_lable2.setText(str(Mod.flag_centers)) # Mod.flag_centers
        self.des_points = Mod.flag_centers

    @pyqtSlot()
    def start_program_single(self):
        if self.main_program._run_flag:
            self.main_program._run_flag = False
        else: 
            self.main_program.drone = self.drone.drone
            self.main_program.controller = self.pid_controller
            self.main_program.des_points = self.des_points
            self.main_program.start()

    @ pyqtSlot()
    def start_program_swarm(self):
        if self.main_program._run_flag:
            self.main_program._run_flag = False
        else:
            self.main_program.controllers = self.controllers
            self.main_program.swarm = self.swarm
            print(self.swarm)
            self.main_program.start()
        

    @pyqtSlot()
    def create_pid(self):
        try:
            kp = float(self.kp_tb.text())
            ki = float(self.ki_tb.text())
            kd = float(self.kd_tb.text())
        except Exception:
            kp, ki, kd = 0.2, 0.05, 0.05 

        spx = self.spx_tb.text()
        spy = self.spy_tb.text()
        if spx and spy:
            self.setpoint = Point(spx, spy)
        elif self.des_points:
            self.setpoint = self.des_points.pop()
        else:
            self.setpoint = Point(320,240)
        try:
            self.pid_controller = PidDroneControl(self.drone.drone, self.setpoint, 0, kp, ki, kd)
            print(self.pid_controller.pidx.setpoint)
            print(self.pid_controller.pidy.setpoint)
            print(self.pid_controller.pidx.tunings)
            print(self.pid_controller.pidy.tunings)
        except Exception as e:
            print(e)
    
    @pyqtSlot()
    def make_drone_objects(self):

        self.controllers = []
        try:
            kp = float(self.kp_tb.text())
            ki = float(self.ki_tb.text())
            kd = float(self.kd_tb.text())
        except Exception:
            kp, ki, kd = 0.2, 0.05, 0.05 

        # KEEP IN MIND:
        # 1) drone placement is important
        # 2) flag number is important
        # 3) obtaining new flag points once the "land condition" has been met will
        #    need to be implemented
        
        try:
            drones = []
            for ind, ip in enumerate(drone_ips): 
                drone = Tello(ip, retry_count=1)
                drone.connect()
                drones.append(drone)
                des = self.des_points.pop()
                self.controllers.append(PidDroneControl(drone, des, ind, kp, ki, kd))
            self.swarm = TelloSwarm(drones)
        except Exception as e:
            print(e)

        print(self.controllers)

    @pyqtSlot()
    def drone_disconnect(self):
        self.drone.disconnect()

    @pyqtSlot()
    def tello_ip_button_clicked(self):
        ip = self.tello_ip_text_box.text()
        self.drone = DroneConnect(ip)
        print(self.drone) 

    def closeEvent(self, event):
        self.thread.shut_down()
        event.accept()

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.image_label.setPixmap(qt_img)

    @pyqtSlot()
    def no_stream(self):
        self.image_label.setPixmap(self.no_stream_img)

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)
    
if __name__=="__main__":
    Mod = Model()
    drone_ips = ['192.168.1.100', '192.168.1.200']


    app = QApplication(sys.argv)
    a = App()
    a.show()
    sys.exit(app.exec_())