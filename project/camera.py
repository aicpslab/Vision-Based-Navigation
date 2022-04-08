"""
Vision Based Navigation Project
Augusta University
3/11/2022

This file contains the Camera class.
This class manages a cv2 video object
and ensures that each image is convereted 
to RGB color space rather than the cv2 default
of BGR before being used by the Tensorflow Model.

camera.py
"""
# Python Imports
import cv2 
import numpy as np


class Camera:
    """ Class tha manages a cv2.VideoCapture Object
        Converts every picture to RGB format. """
    
    def __init__(self, camera_index:int):
        self.cam = cv2.VideoCapture(camera_index)

        initial_img = self.initial_connect()
        self._img = initial_img
    
    @property
    def img(self):
        return self._img 
    
    @img.setter
    def img(self, value):
        im = cv2.cvtColor(value, cv2.COLOR_BGR2RGB)
        im = np.array(im)
        self._img = im
    
    def click(self):
        ret, retImg = self.cam.read()
        if ret:
            self.img = retImg
        return self.img
    
    def initial_connect(self):
        for x in range(50):
            _, img = self.cam.read()
        return img
    
    def release(self):
        self.cam.release()

if __name__ == '__main__':
    
    def CameraTest():
        cam = Camera(1)
        
        while True:
            img = cam.click()
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imshow("Test", img)
            cv2.waitKey(1)

    CameraTest()
