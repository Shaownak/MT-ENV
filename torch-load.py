import cv2
import torch
import numpy as np

cap = cv2.VideoCapture(0)

_model = torch.hub.load('WongKinYiu/yolov7', 'custom') # _model is the model itself
# model = torch.hub.load('WongKinYiu/yolov7', 'custom', path=r'yolov7\runs\train\exp2\weights\best.pt', force_reload=True)
model = torch.load('yolov7\runs\train\exp2\weights\best.pt', force_reload=True, trust_repo=True)

# model = torch.hub.load('E:/yolov7/', 'custom', path='E:\yolov7\runs\train\exp2\weights\best.pt', source='local', force_reload=True)

while cap.isOpened():
    ret, frame = cap.read()

    # Make detections
    results = model(frame)

    cv2.imshow('YOLO', np.squeeze(results.render()))

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
