"""
Vison Based Control of Drones Project 
1/21/2022

Dr. Xiang
Patrick
Wesley 

main.py
"""

import time
import tensorflow as tf
import numpy as np
import cv2
## object_detection API imports 
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils
## Custom Classes
from model import Model
from idmanager import IDManager

##############
#### Main ####
##############

PATH_TO_SAVED_MODEL = r'.\resources\model\saved_model'
PATH_TO_LABELS = r'.\resources\annotations\label_map.pbtxt'

Detector = Model(PATH_TO_SAVED_MODEL, PATH_TO_LABELS)

# Drone 1 has password of 12345678 

cam = cv2.VideoCapture(1)

# How many boxes do we want displaying? 
num_boxes = 6
# How many drones do we expect?
num_drones = 2
# How many Destinations do we expect?
num_flags = 4
# How confident does the model need to be to display any bouding box? 
min_score_thresh = .50

########################
# Tracking Stuff
IDS = IDManager()
ret, img = cam.read()
if ret:
    detections = Detector.get_detections(img)
    centers = Detector.get_class_centers(detections, num_drones, 1) # 1 for drone

    print("###########")
    print(len(centers))
    print("###########")

    for center in centers:
        print(center)
        IDS.createID(center)
########################

while True:
    ret, img = cam.read()

    if ret:
        # Convert BGR to RGB.
        color_correct = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Send the picture to the model.
        detected_image, detections = Detector.detect(color_correct, num_boxes, min_score_thresh)
        
        # Convert RGB back to BGR for cv2's imshow function.
        detected_image = cv2.cvtColor(detected_image, cv2.COLOR_RGB2BGR)

        #################
        # Tracking Stuff
        drone_centers = Detector.get_class_centers(detections, num_drones, 1)
        IDS.updatePositions(drone_centers)
        detected_image = IDS.draw_ids(detected_image)
        #################

        # Get the num_boxes number of center coordinates and display them on the image. 
        centers_img, centers = Detector.draw_centers(detected_image, detections, num_boxes)

        

        cv2.imshow('Img', centers_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
"""
Differentiate the Drones
   Drone Class

"""

""" 
What's next?? 

Determine Fly to Points 

Associate Drone Coordinates wtih ID's 

Determine drone direction.
"""

'''
Ideas: 
- Save all paper points once at the beginning of the program
- Use paper points as fly-to-points
- Update paper positions after drone has completed a flight command (i.e. 'checkpoint')
'''
# python exporter_main_v2.py --input_type image_tensor --pipeline_config_path .\exported-models\all_data\pipeline.config --trained_checkpoint_dir .\exported-models\all_data --output_directory .\exported-models\model_final