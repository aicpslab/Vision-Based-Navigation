"""
Vision Based Navigation Project
Augusta University
3/11/2022

This file contains the Model class
that is responsible for loading and
managing the connection to the 
tensorflow Model. 

model.py
"""
# Python imports
from cv2 import COLOR_RGB2BGR, FONT_HERSHEY_SIMPLEX
import tensorflow as tf
import numpy as np
import cv2
# object_detection API Imports
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils
# Custom Imports
from point import Point
from ids import ID
from camera import Camera
from drawing import DrawingConstants as dc
from cvfpscalc import CvFpsCalc

class Model:
    """
        Class Responsible for loading and managing the 
        camera object and performing detections on 
        images. 
    """
    # Class Variables
    PATH_TO_SAVED_MODEL = r'.\resources\model\saved_model'
    PATH_TO_LABELS = r'.\resources\annotations\label_map.pbtxt'
    # How many boxes do we expect?
    MAX_BOXES = 6
    # How confident does the model need to be to display any bouding box?
    MIN_SCORE_THRESH = .65
    # Camera Stuff

    def __init__(self, cam_index, low_memory : bool = False):
        
        self.cam = Camera(cam_index)
        self.imwidth = int(self.cam.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.imheight = int(self.cam.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Suppress TensorFlow logging
        tf.get_logger().setLevel('ERROR')

        if low_memory:
            # enable dynamic memory
            gpus = tf.config.experimental.list_physical_devices('GPU')
            for gpu in gpus:
                tf.config.experimental.set_memory_growth(gpu, True)

        # Load the Model
        self.detector_function = tf.saved_model.load(self.PATH_TO_SAVED_MODEL)
        self.cat_indx = label_map_util.create_category_index_from_labelmap(
            self.PATH_TO_LABELS, use_display_name=True)
        self.drone_category = 1
        self.flag_category = 2

        # Create Two IDS to start
        ID.createID(Point(0, 0), dc.red)
        ID.createID(Point(0,0), dc.blue)

        self.drone_centers = []
        self.flag_centers = []

        self.thread_img = self.cam.click()

        self.detections = self.update_detections()

        self.cv_fps_calc = CvFpsCalc(buffer_len=10)
    
    def update_detections(self):
        """
            Gets a new picture and updates self.detections
        """
        img = self.cam.click()
        # Prepare the img for the model
        input_tensor = tf.convert_to_tensor(img)
        input_tensor = input_tensor[tf.newaxis, ...]

        # get output from the model
        detections = self.detector_function(input_tensor)
        num_detections = int(detections.pop('num_detections'))
        # Clean up detections
        detections = {key: value[0, :num_detections].numpy()
                      for key, value in detections.items()}

        self.detections = detections
    
    def compute_center(self, box):
            """
                Method to compute the center of a box.
                Box is the normalized coords from the model.
            """
            # Normalized coordinates from the model
            (ymin, xmin, ymax, xmax) = tuple(box.tolist())
            # Actual coordinates on the image
            (left, right, top, bottom) = (xmin * self.imwidth,
                                        xmax * self.imwidth,
                                        ymin * self.imheight,
                                        ymax * self.imheight)

            xavg = (left + right) // 2
            yavg = (bottom + top) // 2

            return Point(int(xavg),int(yavg))

    def update_centers(self, max_num_detections= None):
        """ Method to get the flags center from detections.
            Will only look at max_num_detections number of detections. """
        if max_num_detections is None:
            max_num_detections = self.MAX_BOXES

        # Only look at the top few detections
        boxes = self.detections['detection_boxes'][0:max_num_detections]
        confidences = self.detections['detection_scores']
        classes = self.class_category = self.detections['detection_classes']

        drone_cents = []
        flag_cents = []

        for index, box in enumerate(boxes):
            # Determine the cateogry and confidence
            class_category = classes[index]
            confidence = confidences[index]

            if confidence > self.MIN_SCORE_THRESH:
                # Computer the average centers cords
                center = self.compute_center(box)
                # Append it to the proper list
                if class_category == self.drone_category:
                    drone_cents.append(center)
                elif class_category == self.flag_category:
                    flag_cents.append(center)

        self.drone_centers = drone_cents
        self.flag_centers = flag_cents

        # Updat the ids based on the new information
        ID.update_positions(drone_cents)
        ID.set_target_flag(flag_cents)
    
    def draw_bounding_boxes(self, img=None):
        """ Method to draw the boxes and confidences
            around detected objects utilizing 
            the object_detection API. """
        if img is None:
            drawn_img = self.cam.img.copy()
        else:
            drawn_img = img

        classes = self.detections['detection_classes'].astype(np.int64)
        viz_utils.visualize_boxes_and_labels_on_image_array(
            drawn_img,
            self.detections['detection_boxes'],
            classes,
            self.detections['detection_scores'],
            self.cat_indx,
            use_normalized_coordinates=True,
            max_boxes_to_draw=self.MAX_BOXES,
            min_score_thresh=self.MIN_SCORE_THRESH,
            agnostic_mode=False)

        return drawn_img

    def update_vectors_thread(self):
        """ Method desinged to update all information and draw 
            all information on self.thread_img.
            Can be put on a thread and access self.thread_img separately. """
        while True:
            fps = self.cv_fps_calc.get()
            # Update Information
            ID.set_vec_start()
            self.update_detections()
            self.update_centers(self.MAX_BOXES)
            ID.set_vec_end()
            ID.update_vecs()

            # Draw information
            _img = ID.draw_vectors(self.cam.img)
            _img = ID.draw_id_nums(_img)
            _img = self.draw_bounding_boxes(_img)
            _img = cv2.cvtColor(_img, cv2.COLOR_RGB2BGR)
            _img = cv2.putText(_img, str(fps), (5, 50), dc.font, dc.font_scale, dc.light_purple, dc.font_thickness, cv2.LINE_AA)
            self.thread_img = _img

    def basic_detection_thread(self):
        """ Method desinged to update basic information.
            and draw on the self.thread_img. 
            Can be put on a thread and access self.thread_img separately. """
        while True:
            fps = self.cv_fps_calc.get()
            # Update Information
            self.update_detections()
            self.update_centers()

            # Draw Information
            img = self.draw_bounding_boxes(self.cam.img)
            #img = ID.draw_histories(img, 30)
            img = ID.draw_id_nums(img)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = cv2.putText(img, str(fps), (5, 50), dc.font, dc.font_scale, dc.light_purple, dc.font_thickness, cv2.LINE_AA) 
            self.thread_img = img
    
    def basic_detection(self):
        """ Same as basic_detection_thread but not for use with a thread. """
        fps = self.cv_fps_calc.get()
        self.update_detections()
        self.update_centers()
        img = self.draw_bounding_boxes(self.cam.img)
        #img = ID.draw_histories(img, 10)
        img = ID.draw_id_nums(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        img = cv2.putText(img, str(fps), (5, 50), dc.font, dc.font_scale, dc.light_purple, dc.font_thickness, cv2.LINE_AA) 
        self.thread_img = img
        return self.thread_img
