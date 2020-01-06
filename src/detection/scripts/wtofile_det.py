import os
import cv2
import numpy as np
import tensorflow as tf
import sys

# This is needed since the notebook is stored in the object_detection folder.
#sys.path.append("..")

# Import utilites
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

print("imports done")


# Name of the directory containing the object detection module we're using
MODEL_NAME = '/home/anveshak/caesar2020/src/detection/trained-inference-graphs/output_inference_graph_v1_19dec.pb'

# Grab path to current working directory
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = '/home/anveshak/caesar2020/src/detection/label_map.pbtxt'

# Number of classes the object detector can identify
NUM_CLASSES = 3

## Load the label map.
# Label maps map indices to category names, so that when our convolution
# network predicts `5`, we know that this corresponds to `king`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

print("starting to load graph")

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.2)
    sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options))
    sess = tf.Session(graph=detection_graph)

print("graph loaded and session created")


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

print("detection variables assigned")

# Initialize webcam feed

#ret = video.set(3,1280)
#ret = video.set(4,720)

image=cv2.imread('/home/anveshak/caesar2020/src/detection/nothing.jpg')
image_expanded = np.expand_dims(image, axis=0)
(boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_expanded})

        # Draw the results of the detection (aka 'visulaize the results')
vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=2,
            min_score_thresh=0.30)

coordinates = vis_util.return_coordinates(
                        image,
                        np.squeeze(boxes),
                        np.squeeze(classes).astype(np.int32),
                        np.squeeze(scores),
                        category_index,
                        use_normalized_coordinates=True,
                        line_thickness=2,
                        min_score_thresh=0.30)
    
        
        
'''    
print(coordinates)

cv2.imshow('first', image)
cv2.waitKey(1000)
cv2.destroyAllWindows()
'''

while True:

	print("entered the first while loop")

	h=open('/home/anveshak/caesar2020/src/detection/save_time.txt',"r")
	lines=h.readlines()
	if len(lines)==0:
		line1='b'
	else: 
		line1=lines[0]
	h.close()

	print("read h")

	if line1[0]=='a':
		video = cv2.VideoCapture(0)
		print("captured video")
		while True:

			print("entered second while loop")
			f=open('/home/anveshak/caesar2020/src/detection/data.txt',"a+") 
        # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
        # i.e. a single-column array, where each item in t he column has the pixel RGB value
			ret, img = video.read()
			dim=(640,480)
			#dim2=(640,480)
			frame=cv2.resize(img,dim,interpolation=cv2.INTER_AREA)
			frame_expanded = np.expand_dims(frame, axis=0)

        # Perform the actual detection by running the model with the image as input
			(boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: frame_expanded})

        # Draw the results of the detection (aka 'visulaize the results')
			vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=2,
            min_score_thresh=0.30)

			coordinates = vis_util.return_coordinates(
                        frame,
                        np.squeeze(boxes),
                        np.squeeze(classes).astype(np.int32),
                        np.squeeze(scores),
                        category_index,
                        use_normalized_coordinates=True,
                        line_thickness=2,
                        min_score_thresh=0.30)
    
        
        
    
    
			print(coordinates) #[xmin, ymin, xmax, ymax, score%, class]
    #f.writelines(coordinates)
			f.write("%d "%coordinates[0])
			f.write("%d "%coordinates[1])
			f.write("%d "%coordinates[2])
			f.write("%d "%coordinates[3])
			f.write("%d "%coordinates[4])
			f.write("%f\n"%coordinates[5])
    #f.write('%d' '%d' '%d' '%d' '%f' '%d\n'%coordinates[0] %coordinates[1] %coordinates[2] %coordinates[3] %coordinates[4] %coordinates[5])
    
    
    # All the results have been drawn on the frame, so it's time to display it.
			cv2.imshow('Object detector', frame)

    # Press 'q' to quit
			if cv2.waitKey(1) == ord('q'):
				break

			h=open('/home/anveshak/caesar2020/src/detection/save_time.txt',"r")
			lines=h.readlines()
			line1=lines[0]
			h.close()

			if not line1[0]=='a':
				break

# Clean up
		video.release()
		cv2.destroyAllWindows()
	
	else:
		pass

"""video = cv2.VideoCapture(0)

while line1[0]=='a':

    f=open('/home/anveshak/caesar2020/src/detection/data.txt',"a+") 
        # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
        # i.e. a single-column array, where each item in t he column has the pixel RGB value
    ret, img = video.read()
    dim=(640,480)
    #dim2=(640,480)
    frame=cv2.resize(img,dim,interpolation=cv2.INTER_AREA)
    frame_expanded = np.expand_dims(frame, axis=0)

        # Perform the actual detection by running the model with the image as input
    (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: frame_expanded})

        # Draw the results of the detection (aka 'visulaize the results')
    vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=2,
            min_score_thresh=0.30)

    coordinates = vis_util.return_coordinates(
                        frame,
                        np.squeeze(boxes),
                        np.squeeze(classes).astype(np.int32),
                        np.squeeze(scores),
                        category_index,
                        use_normalized_coordinates=True,
                        line_thickness=2,
                        min_score_thresh=0.30)
    
        
        
    
    
    print(coordinates) #[xmin, ymin, xmax, ymax, score%, class]
    #f.writelines(coordinates)
    f.write("%d "%coordinates[0])
    f.write("%d "%coordinates[1])
    f.write("%d "%coordinates[2])
    f.write("%d "%coordinates[3])
    f.write("%d "%coordinates[4])
    f.write("%f\n"%coordinates[5])
    #f.write('%d' '%d' '%d' '%d' '%f' '%d\n'%coordinates[0] %coordinates[1] %coordinates[2] %coordinates[3] %coordinates[4] %coordinates[5])
    
    
    # All the results have been drawn on the frame, so it's time to display it.
    cv2.imshow('Object detector', frame)

    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
video.release()
cv2.destroyAllWindows()"""
