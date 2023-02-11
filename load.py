import cv2
import torch
import numpy as np
import pandas
# from utils.general import xyxy2xywh
import serial
from serial import Serial



arduino = serial.Serial(port = 'COM4', baudrate = 9600, timeout = .1)


# --------------------- distance ---------------------

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

frame_width = 1280
frame_height = 720
cap.set(3, frame_width)
cap.set(4, frame_height)


# _model = torch.hub.load('WongKinYiu/yolov7', 'custom') # _model is the model itself
model = torch.hub.load('/home/mongol-tori/Documents/yolov5', 'custom', path=r'/home/mongol-tori/Documents/yolov5/best.pt', source='local', force_reload=True)
# model = torch.load('yolov7\runs\train\exp2\weights\best.pt', force_reload=True, trust_repo=True)

# model = torch.hub.load('E:/yolov7/', 'custom', path='E:\yolov7\runs\train\exp2\weights\best.pt', source='local', force_reload=True)

while cap.isOpened():
    ret, frame = cap.read()
    
    
    # -------------------- divide the frame into 3x3 grid --------------------
    
    line_width = 2
    color = (255, 0, 0)
    
    line1_start = (frame_width//3, 0)
    line1_end = (frame_width//3, frame_height)
    cv2.line(frame, line1_start, line1_end, color, line_width)

    line2_start = (2*frame_width//3, 0)
    line2_end = (2*frame_width//3, frame_height)
    cv2.line(frame, line2_start, line2_end, color, line_width)

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


    distance = Distance_finder(
                Focal_length_found, Known_width, w)


    print(distance, label)
    
    
    # ------------------ Serial Commands ------------------
    
    
    if (distance > 10):
        arduino.write(b'f')
    
    if(distance <= 10):
        arduino.write(b'd')
        if(label=='1'):
            arduino.write(b'r')
        
        elif(label=='0'):
            arduino.write(b'l')
            
    if label is None:
        arduino.write(b'n')
        
        

    
    
    
    
    


    cv2.imshow('YOLO', np.squeeze(results.render()))

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()