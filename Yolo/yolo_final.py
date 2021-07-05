import argparse

import cv2
import numpy as np

whT = 416
confThreshold = 0.2
nmsThreshold = 0.4# lower it is less no. of boxes

img = cv2.imread('/home/greesh/Yolo/camera.jpg')
classfile = '/home/greesh/Yolo/classes.names'
classnames = []
with open(classfile,'rt') as f:
    classnames =  f.read().rstrip('\n').split('\n')
print(classnames)

modelconfig = '/home/greesh/Yolo/yolov4-tiny_custom_drone.cfg'
modelweights = '/home/greesh/Yolo/yolov4-tiny_custom_drone_last.weights'

net = cv2.dnn.readNetFromDarknet(modelconfig,modelweights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

blob = cv2.dnn.blobFromImage(img,1/255,(whT,whT),[0,0,0],1,crop=False)
net.setInput(blob)

layernames = net.getLayerNames()
print(layernames)
# print(net.getUnconnectedOutLayers())
outputnames = [layernames[i[0]-1] for i in net.getUnconnectedOutLayers()]
print(outputnames)

outputs = net.forward(outputnames)
print(outputs[0].shape)
print(outputs[1].shape)

print(outputs[0][0])


def findobjects(outputs,img):
    hT,wT,cT = img.shape
    bbox = []
    classids = []
    confs = []

    for output in outputs:
        for det in output:
            scores = det[5:]
            classid = np.argmax(scores)
            confidence = scores[classid]
            if(confidence > confThreshold):
                w,h = int(det[2]*wT ),int(det[3]*hT)
                x,y = int((det[0]*wT)-w/2), int((det[1]*hT)-h/2)
                bbox.append([x,y,w,h])
                classids.append(classid)
                confs.append(float(confidence))

    print(len(bbox))

    #nonmax supression
    indices = cv2.dnn.NMSBoxes(bbox,confs,confThreshold,nms_threshold=nmsThreshold)
    print(indices)
    for i in  indices:
        i = i[0]
        box = bbox[i]
        x,y,w,h = box[0],box[1],box[2],box[3]
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,255),2)
        cv2.putText(img,f'{classnames[classids[i]].upper()},{int(confs[i]*100)}%',(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),1)





while(True):
    img = cv2.imread('/home/greesh/Yolo/camera.jpg')

    try:
        findobjects(outputs,img)
        cv2.imshow("img", img)
        cv2.waitKey(1)
    except AttributeError:
        pass

