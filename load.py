import cv2
import torch
import numpy as np
import pandas
from utils.general import xyxy2xywh


# distance

Known_distance = 76.2
Known_width = 14.3
ref_image_width = 10

def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):

	# finding the focal length
	focal_length = (width_in_rf_image * measured_distance) / real_width
	return focal_length


# distance estimation function
def Distance_finder(Focal_Length, real_arrow_width, arrow_width_in_frame):

	distance = (real_arrow_width * Focal_Length)/arrow_width_in_frame

	# return the distance
	return distance


Focal_length_found = Focal_Length_Finder(
	Known_distance, Known_width, ref_image_width)




cap = cv2.VideoCapture(0)

# _model = torch.hub.load('WongKinYiu/yolov7', 'custom') # _model is the model itself
model = torch.hub.load('/home/mongol-tori/Documents/yolov5', 'custom', path=r'/home/mongol-tori/Documents/yolov5/best.pt', source='local', force_reload=True)
# model = torch.load('yolov7\runs\train\exp2\weights\best.pt', force_reload=True, trust_repo=True)

# model = torch.hub.load('E:/yolov7/', 'custom', path='E:\yolov7\runs\train\exp2\weights\best.pt', source='local', force_reload=True)

while cap.isOpened():
    ret, frame = cap.read()

    # Make detections
    results = model(frame)

    # print(results.pandas().xyxy[0])

    xmin = results.pandas().xyxy[0]['xmin']
    ymin = results.pandas().xyxy[0]['ymin']
    xmax = results.pandas().xyxy[0]['xmax']
    ymax = results.pandas().xyxy[0]['ymax']
    label = results.pandas().xyxy[0]['class']

    # x0, y0, x1, y1, confi = results.pandas().xyxy[0]

    w = xmax - xmin
    h = ymax - ymin


    Distance = Distance_finder(
                Focal_length_found, Known_width, w)


    print(Distance, label)


    cv2.imshow('YOLO', np.squeeze(results.render()))

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
