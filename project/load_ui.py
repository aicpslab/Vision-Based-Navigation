from itertools import starmap
from typing import List
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
from pyqtgraph import PlotWidget, plot, InfiniteLine

class UI_Single_Mode(QMainWindow):

    def __init__(self):
        super(UI_Single_Mode, self).__init__()

        uic.loadUi(ui_single_path, self)
        #self.setFixedSize(1036, 666)

        # Create the Video Stream Thread
        self.stream_thread = StreamThread()
        self.stream_thread.update_img_signal.connect(self.update_image)
        self.stream_thread.stream_off_signal.connect(self.stream_off)
        
        # Create the Graph Info Thread
        self.graph_info = GraphInfo()
        self.graph_info.update_plots_signal.connect(self.update_plots)

        # define our widgets
        self.load_widgets()
        self.connect_buttons() 
       
        self.show()
    
    def load_widgets(self):
        
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
        self.pushButton_load_tunings = self.findChild(QPushButton, "pushButton_load_tunings")
        self.pushButton_save_tunings = self.findChild(QPushButton, "pushButton_save_tunings")

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

        # Menu Button
        self.action_swarm = self.findChild(QAction, "action_swarm")

        # Graphs
        self.qtGraph_x = self.findChild(PlotWidget, "qtGraph_x")
        self.qtGraph_y = self.findChild(PlotWidget, "qtGraph_y")

        self.x_line_data = self.qtGraph_x.plot()
        self.y_line_data = self.qtGraph_y.plot()
    
    def connect_buttons(self):
        
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

        self.action_swarm.triggered.connect(self.clicked_view_action_swarm)
    
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

    @pyqtSlot(tuple)
    def update_plots(self, params):
        x_list, y_list, time_list = params
        self.x_line_data.setData(time_list, x_list)
        self.y_line_data.setData(time_list, y_list)
        print('Ran plot commands')

    def clicked_program_start(self):
        try:
            self.program_thread = SingleProgramThread(self.drone.self, self.pid_controller, self.des_points)
            self.program_thread.drone_landed_signal.connect(self.graph_info.stop)
            # self.qtGraph_x.clear()
            # self.qtGraph_y.clear()
            
            self.x_line_data.setData([0],[0])
            self.y_line_data.setData([0],[0])

            self.qtGraph_x.addLine(y=self.pid_controller.pidx.setpoint)
            self.qtGraph_y.addLine(y=self.pid_controller.pidy.setpoint)

            self.program_thread.start()
            self.graph_info.start()
        except Exception as e:
            if self.program_thread._run_flag:
                self.program_thread._run_flag = False
            if self.graph_info._run_flag:
               self.graph_info._run_flag = False
            print(e)

    def clicked_create_pid(self):
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
            print(e)
            return
        
        if spx and spy:

            try:
                spx = int(spx)
                spy = int(spy)
                setpoint = Point(spx, spy)
            except Exception as e:
                print(e)
                return

        elif self.des_points:
            setpoint = self.des_points.pop()
        
        try:
            self.pid_controller = PidDroneControl(self.drone.self, setpoint, 0, 0, 0, 0)
            self.pid_controller.pidx.tunings = (x_kp, x_ki, x_kd)
            self.pid_controller.pidy.tunings = (y_kp, y_ki, y_kd)

            print(self.pid_controller.pidx.setpoint)
            print(self.pid_controller.pidy.setpoint)
            print(self.pid_controller.pidx.tunings)
            print(self.pid_controller.pidy.tunings)
        except Exception as e:
            print(e)
        
    def clicked_emergency_stop(self):
        self.pid_controller.land()
        self.graph_info.stop()
        try:
            if self.program_thread._run_flag:
                self.program_thread._run_flag = False
        except Exception as e:
            print(e)
        
        try:
            self.graph_info.stop()
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
        ip = self.lineEdit_drone_ip.text()
        self.drone = Drone(ip)
        
    def clicked_drone_disconnect(self):
        self.drone.disconnect()
        self.drone = None
        print(f'{self.drone=}')
    
    def clicked_load_presets(self):

        self.lineEdit_drone_ip.setText(str(drone_ips[0]))
        self.lineEdit_x_kp.setText("0.30")
        self.lineEdit_x_ki.setText("0.08")
        self.lineEdit_x_kd.setText("0.3")
        self.lineEdit_y_kp.setText("0.25")
        self.lineEdit_y_ki.setText("0.05")
        self.lineEdit_y_kd.setText("0.1")

    def clicked_view_action_swarm(self):

        self.hide()
        ui_swarm_mode = UI_Swarm_Mode(self)
        ui_swarm_mode.show()
    
    def clicked_load_tunings(self):
        with open('pid_tunings.json', 'r') as f:
            tunings = json.load(f)

        self.lineEdit_x_kp.setText(tunings['x_kp'])
        self.lineEdit_x_ki.setText(tunings['x_ki'])
        self.lineEdit_x_kd.setText(tunings['x_kd'])
        self.lineEdit_y_kp.setText(tunings['y_kp'])
        self.lineEdit_y_ki.setText(tunings['y_ki'])
        self.lineEdit_y_kd.setText(tunings['y_kd'])

    def clicked_save_tunings(self):
        tunings = {
            'x_kp': self.lineEdit_x_kp.text(),
            'x_ki': self.lineEdit_x_ki.text(),
            'x_kd': self.lineEdit_x_kd.text(),
            'y_kp': self.lineEdit_y_kp.text(),
            'y_ki': self.lineEdit_y_ki.text(),
            'y_kd': self.lineEdit_y_kd.text()
        }

        with open('pid_tunings.json', 'w') as f:
            json.dump(tunings, f)

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
    update_img_signal = pyqtSignal(np.ndarray)
    stream_off_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._run_flag = False

        # Create a image for use when there is no stream
        no_stream_img = QtGui.QImage(640, 480, QtGui.QImage.Format_Indexed8)
        no_stream_img.fill(QtGui.qRgb(128, 128, 128))
        self.no_stream_img = QPixmap.fromImage(no_stream_img)
    
    def run(self):
        self._run_flag = True
        if "no_model" not in sys.argv:
            # With Model
            while self._run_flag:
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
        if self._run_flag:
            self._run_flag = False
        else:
            self.start()
    
class Drone():

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
    drone_landed_signal = pyqtSignal()
    def __init__(self, drone:Tello, controller: PidDroneControl, des_points):
        super().__init__()
        self._run_flag = False
        self.drone: Tello = drone
        self.controller = controller
        self.des_points = des_points
    
    def run(self):
        self._run_flag = True
        
        try:
            self.drone.takeoff()
            while self._run_flag:
                if self.controller.new_point:
                    if self.des_points:
                        des = self.des_points.pop()
                        self.controller.update_setpoints(des)
                    else:
                        self.drone.land()
                        self._run_flag = False
                        self.drone_landed_signal.emit()
                else:
                    self.controller.update_velocity()
                    #print('Updated Velocity..')
                    #print(self.controller.d2d)
                    #print(self.controller.wait_time)
        except Exception as e:
            self._run_flag = False
            print(e)

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
    cam_index = 0

    app = QApplication(sys.argv)

    if "no_model" in sys.argv:
        cam = cv2.VideoCapture(0)
    else:
        Mod = Model(cam_index, low_memory=True)

    ui_single_path = "./resources/gui_views/single_drone.ui"
    ui_swarm_path = "./resources/gui_views/swarm_drone.ui"
    UIWindow = UI_Single_Mode()
    
    sys.exit(app.exec_())