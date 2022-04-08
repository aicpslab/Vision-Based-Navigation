from logging import shutdown
from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget, QApplication, QLabel, QVBoxLayout, QPushButton
from PyQt5.QtGui import QPixmap
import sys
import cv2
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import numpy as np
from model import Model


class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    no_change_signal = pyqtSignal()
    Model = Model()

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self._shut_down = False

    def run(self):
        # capture from web cam
        while self._run_flag:
            img = self.Model.baisc_detection()
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

        self.stream_button = QPushButton('Start/Stop Stream', self)
        # create a vertical box layout and add the two labels
        vbox = QVBoxLayout()
        vbox.addWidget(self.image_label)
        vbox.addWidget(self.textLabel)
        vbox.addWidget(self.stream_button)
        # set the vbox layout as the widgets layout
        self.setLayout(vbox)

        # create the video capture thread
        self.thread = VideoThread()
        self.stream_button.clicked.connect(self.thread.stream_button_click)
        no_stream_img = QtGui.QImage(640, 480, QtGui.QImage.Format_Indexed8)
        no_stream_img.fill(QtGui.qRgb(128, 128, 128))
        self.no_stream_img = QPixmap.fromImage(no_stream_img)
        # connect its signal to the update_image slot
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.no_change_signal.connect(self.no_stream)
        # start the thread
        self.thread.start()

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
    app = QApplication(sys.argv)
    a = App()
    a.show()
    sys.exit(app.exec_())