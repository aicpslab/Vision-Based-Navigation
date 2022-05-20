"""
Vision Based Navigation Project
Augusta University
3/11/2022

This file contains the Camera class.
This class manages a cv2 video object
and ensures that each image is convereted 
to RGB color space rather than the cv2 default
of BGR.

camera.py
"""
# Python Imports
import cv2 
import numpy as np


class Camera:
    """ 
    Class tha manages a cv2.VideoCapture object.
    Converts every picture to RGB format. 
    """
    
    def __init__(self, camera_index:int):
        """ 
        :param camera_index: The index number of the camera that you want to 
                                use. Begins counting at 0. 
        :type camera_index: Int
        """
        self.cam = cv2.VideoCapture(camera_index)

        initial_img = self._initial_connect()
        self._img = initial_img
    
    @property
    def img(self):
        """
        The last picture the camera took.

        :getter: Returns the last picture taken when ''self.click()'' method was called. 
        :setter: Converts an BGR image to RGB colors and into a numpy array. 
        :type: Numpy Array
        """
        return self._img 
    
    @img.setter
    def img(self, value):
        im = cv2.cvtColor(value, cv2.COLOR_BGR2RGB)
        im = np.array(im)
        self._img = im
    
    def click(self):
        """ 
        Method to utlize the camera and set the img property.

        :returns: The current img property
        :rtype: Numpy Array
        """
        ret, retImg = self.cam.read()
        if ret:
            self.img = retImg
        return self.img
    
    def _initial_connect(self):
        """ 
        Method to allow the camera to focus. 
        Used internally, not intedned to be called.
        
        :returns: A BGR image from cv2.VideoCapture.read() 
        """
        for x in range(50):
            _, img = self.cam.read()
        return img
    
    def release(self):
        """ 
        Method to release the cv2.VideoCaputure object. 
        Call this before ending the program.
        """
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
