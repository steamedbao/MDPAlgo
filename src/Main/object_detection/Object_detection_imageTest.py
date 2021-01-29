######## Image Object Detection Using Tensorflow-trained Classifier #########
#
# Author: Evan Juras
# Date: 1/15/18
# Description: 
# This program uses a TensorFlow-trained neural network to perform object detection.
# It loads the classifier and uses it to perform object detection on an image.
# It draws boxes, scores, and labels around the objects of interest in the image.

## Some of the code is copied from Google's example at
## https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

## and some is copied from Dat Tran's example at
## https://github.com/datitran/object_detector_app/blob/master/object_detection_app.py

## but I changed it to make it more understandable to me.

# Import packages
import os
import cv2
import numpy as np
import tensorflow as tf
import sys

# This is needed since the notebook is stored in the object_detection folder.
sys.path.insert(1, os.path.join(sys.path[0], '..'))

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util
import paramiko

ssh_client = paramiko.SSHClient()
ssh_client.load_system_host_keys()

ssh_client.connect("192.168.2.2", username="pi", password="MDPGrp02")
stdin, stdout, stderr = ssh_client.exec_command("ls /home/pi/CZ3004/captured")
image_list = stdout.readlines()
ftp_client=ssh_client.open_sftp()

for i in range(len(image_list)):
    remote_path = "/home/pi/CZ3004/captured/" + image_list[i].replace("\n", "")
    local_path = "/Users/hannancao/Desktop/CZ3004/Updated_Group_2_Algorithm_Codes/src/Main/object_detection/captured/" + image_list[i].replace("\n", "")
    ftp_client.get(remote_path, local_path)
ftp_client.close()
ssh_client.close()
# Name of the directory containing the object detection module we're using
MODEL_NAME = 'inference_graph'
FOLDER_NAME = 'captured'

# IMAGE_NAME = 'imageTest.png'

# Grab path to current working directory
CWD_PATH = os.getcwd()

print(CWD_PATH)
new_path = os.path.join(CWD_PATH, 'src', 'Main', 'object_detection')
print(new_path)
# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(new_path,MODEL_NAME,'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = os.path.join(new_path,'training','labelmap.pbtxt')

# Path to image
PATH_TO_FOLDER = os.path.join(new_path,FOLDER_NAME)

files = []
for r, d, f in os.walk(PATH_TO_FOLDER):
    for file in f:
        if '.png' in file:
            files.append(os.path.join(r, file))

# Number of classes the object detector can identify
NUM_CLASSES = 15
print("File lists")
print(files)
# Load the label map.
# Label maps map indices to category names, so that when our convolution
# network predicts `5`, we know that this corresponds to `king`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.compat.v1.Session(graph=detection_graph)
print("Start labeling")
check_exits = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
for i in range(len(files) - 1, -1, -1):
# for i in range(len(files)):
    # Define input and output tensors (i.e. data) for the object detection classifier

    # Input tensor is the image
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

    # Output tensors are the detection boxes, scores, and classes
    # Each box represents a part of the image where a particular object was detected
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

    # Each score represents level of confidence for each of the objects.
    # The score is shown on the result image, together with the class label.
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

    # Number of objects detected
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    # Load image using OpenCV and
    # expand image dimensions to have shape: [1, None, None, 3]
    # i.e. a single-column array, where each item in the column has the pixel RGB value
    PATH_TO_IMAGE = files[i]
    image = cv2.imread(PATH_TO_IMAGE)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_expanded = np.expand_dims(image_rgb, axis=0)

    # Perform the actual detection by running the model with the image as input
    (boxes, scores, classes, num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections],
                                             feed_dict={image_tensor: image_expanded})
    # Draw the results of the detection (aka 'visulaize the results')
#     vis_util.visualize_boxes_and_labels_on_image_array(image, np.squeeze(boxes), np.squeeze(classes).astype(np.int32),
#                                                        np.squeeze(scores), category_index,
#                                                        use_normalized_coordinates=True, line_thickness=8,
#                                                        min_score_thresh=0.60)
#
#     # All the results have been drawn on image. Now display the image.
#     cv2.imshow('Object detector', image)
#
#     # Press any key to close the image
#     cv2.waitKey(0)
#
#     # Clean up
#     cv2.destroyAllWindows()

    #Test print the labelID
    #print("Class labels: {}".format(np.squeeze(classes).astype(np.int32)[0]))

    #Use a dictionary to map
    labelID = np.squeeze(classes).astype(np.int32)[0]
    print(type(labelID))
    thisDictionary = {
        "1": "11",
        "2": "12",
        "3": "13",
        "4": "14",
        "5": "2",
        "6": "15",
        "7": "10",
        "8": "9",
        "9": "4",
        "10": "6",
        "11": "3",
        "12": "5",
        "13": "8",
        "14": "7",
        "15": "1",
    }
    if np.squeeze(scores)[0] < 0.5:
        imageID = "Ignore"
    else:
        imageID = thisDictionary[str(labelID)]
        if check_exits[int(imageID) - 1] == 0:
            check_exits[int(imageID) - 1]=1
            vis_util.visualize_boxes_and_labels_on_image_array(imageID, image, np.squeeze(boxes), np.squeeze(classes).astype(np.int32),
                                                                                   np.squeeze(scores), category_index,
                                                                                   use_normalized_coordinates=True, line_thickness=8,
                                                                                   min_score_thresh=0.60)
            label_path = files[i].replace("captured", "labels")
            cv2.imwrite(label_path, image)
#             print("Original!!!!")
#             print(label_path)
            tmp_path = label_path.replace("/Users/hannancao/Desktop/CZ3004/Updated_Group_2_Algorithm_Codes/src/Main/object_detection/labels/image_RPI,PC,", "")
#             print("Updated!!!!")
#             print(tmp_path)
            tmp_path = tmp_path.replace(".png", "")
#             print("Updated!!!!")
#             print(tmp_path)
            tmp_path = tmp_path.replace(":", "/")
            if "P" not in tmp_path:
                returb_str = imageID + "/" + tmp_path
                print(returb_str)

    #write into the .txt file

#     #clear contents first
#     f = open('predictedResult.txt', 'w').close()
#     f = open("predictedResult.txt", "a+")
#     f.write("%s\r\n" % imageID)
#     f.close()