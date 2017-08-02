#!/usr/bin/env python
# import the necessary packages
import numpy as np
import cv2
import cv2.cv as cv
from video import create_capture
from common import clock, draw_str
import time
w = 640
h = 480
import serial
ser = serial.Serial('COM13', 9600)
time.sleep(2)
prev_right_eye_x=240
prev_left_eye_x=90
help_message = '''
USAGE: facedetect.py [--cascade <cascade_fn>] [--nested-cascade <cascade_fn>] [<video_source>]
'''

def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags = cv.CV_HAAR_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects

def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        
def union(a,b):
    x = min(a[0], b[0])
    y = min(a[1], b[1])
    w = max(a[2], b[2])
    h = max(a[3], b[3]) 
    return x, y, w, h
    
def intersection(a,b):
    x = max(a[0], b[0])
    y = max(a[1], b[1])
    w = min(a[2], b[2])
    h = min(a[3], b[3]) 
    if w<0 or h<0: return (0,0,0,0) # or (0,0,0,0) ?
    return x, y, w, h


def non_max_suppression(boxes_1,boxes_2):
    
    # grab the coordinates of the bounding boxes
    x1 = boxes_1[0]
    y1 = boxes_1[1]
    x2 = boxes_1[2]
    y2 = boxes_1[3]
    
    x3 = boxes_2[0]
    y3 = boxes_2[1]
    x4 = boxes_2[2]
    y4 = boxes_2[3]
    # compute the area of the bounding boxes 
    area1 = (x2 - x1 + 1) * (y2 - y1 + 1)
    area2 = (x4 - x3 + 1) * (y4 - y3 + 1)
    
    # compute the ratio of overlap between the bounding boxes  
    overlap = (area1*1.0) / area2
    if 0.7 < overlap < 1.3:
        return boxes_1
    else:
        return boxes_2
     

if __name__ == '__main__':
    import sys, getopt
    print help_message

    args, video_src = getopt.getopt(sys.argv[1:], '', ['cascade=', 'nested-cascade='])
    try: video_src = video_src[0]
    except: video_src = 0
    args = dict(args)
    cascade_fn = args.get('--cascade', "../../data/haarcascades/haarcascade_frontalface_alt.xml")
    nested_fn  = args.get('--nested-cascade', "../../data/haarcascades/haarcascade_eye.xml")
    #detect face
    cascade = cv2.CascadeClassifier(cascade_fn)
    #detect eyes in the face
    nested = cv2.CascadeClassifier(nested_fn)

    cam = create_capture(video_src, fallback='synth:bg=../cpp/lena.jpg:noise=0.05')
    u=0
    boxes1=[]
    boxes3=[]
    boxes5=[]
    while True:
        subrects1=[]
        ret, img = cam.read()
    
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        frame = cv2.equalizeHist(frame)

        t = clock()
        rects = detect(frame, cascade)
        vis = img.copy()
        
        windowClose = np.ones((5,5),np.uint8)
        windowOpen = np.ones((3,3),np.uint8)   
        windowErode = np.ones((2,2),np.uint8)

        draw_rects(vis, rects, (0, 255, 0))
        for x, y, w, h in rects:
            cent=[]
            
            boxes2=rects[0]
            if u!=0:
                box=non_max_suppression(boxes1,boxes2)
                x,y,w,h=box
                
            roi = frame[y:h, x:w]
            vis_roi = vis[y:h, x:w]
            subrects = detect(roi.copy(), nested)
            new_rect=[]
            
            if len(subrects)>0:
                subrects1=subrects.tolist()
            
            if len(subrects1)>2:
                i=0
                k=0
                rect1=subrects1[0]
                while(i<len(subrects1)):
                    rect2=subrects1[i]
                    
                    if ((intersection(rect1,rect2))!=(0,0,0,0)):
                        rect1=(union(rect1,rect2))
                        new_rect.append(rect1)
                        
                        if i<len(subrects1)-1:
                            if ((intersection(rect1,subrects1[i+1]))!=(0,0,0,0)):
                                new_rect.remove(rect1)
                            else:
                                if k>0:
                                    new_rect.remove(new_rect[k])
                        if i==len(subrects1)-1:
                            new_rect.remove(new_rect[k])
                            
                    else:
                        new_rect.append(rect2)
                        k=new_rect.index(rect2)
                        rect1=rect2
                   
                    i+=1
            else:
                import copy
                new_rect=copy.copy(subrects1)
            
            #detecting whether the subrects is 'left' eye or 'right' eye
            pupilFrame = roi
            pupilO = roi
            if len(new_rect)==2:
                draw_rects(vis_roi, new_rect, (255, 0, 0))
                new_rect=np.asarray(new_rect)
                z=0
                for x1,y1,w1,h1 in new_rect:
                    
                    if (x+x1)>(x+min(new_rect[:,0])):
                        eye = 'right'
                        if u!=0:
                            boxes4=new_rect[z]
                            z+=1
                            boxx=non_max_suppression(boxes3,boxes4)
                            x1,y1,w1,h1=boxx
                            boxes3=boxes4
                        else:
                            boxes3=new_rect[z]
                            z+=1
                    else:
                        eye = 'left'
                        if u!=0:
                            boxes6=new_rect[z]
                            z+=1
                            boxx1=non_max_suppression(boxes5,boxes6)
                            x1,y1,w1,h1=boxx1
                            boxes5=boxes6
                        else:
                            boxes5=new_rect[z]
                            z+=1
                    cv2.rectangle(roi, (x1,y1), ((w1),(h1)), (0,0,255),1)
                    cv2.line(roi, (x1,y1), ((w1,h1)), (0,0,255),1)
                    cv2.line(roi, (w1,y1), ((x1,h1)), (0,0,255),1)
                    pupilFrame = cv2.equalizeHist(roi[(0.75*y1+0.25*h1):h1, x1:w1])
                    pupilO = pupilFrame
                    ret, pupilFrame = cv2.threshold(pupilFrame,55,255,cv2.THRESH_BINARY)	
                    pupilFrame = cv2.morphologyEx(pupilFrame, cv2.MORPH_CLOSE, windowClose)
                    pupilFrame = cv2.morphologyEx(pupilFrame, cv2.MORPH_ERODE, windowErode)
                    pupilFrame = cv2.morphologyEx(pupilFrame, cv2.MORPH_OPEN, windowOpen)

                    #so above we do image processing to get the pupil..
		    #now we find the biggest blob and get the centriod
                    threshold = cv2.inRange(pupilFrame,250,255)
                    cv2.imshow('thresh',threshold)
                    contours, hierarchy = cv2.findContours(threshold,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

                    #if there are 3 or more blobs, delete the biggest and delete the left most for the right eye
		    #if there are 2 blob, take the second largest
		    #if there are 1 or less blobs, do nothing
                    
                    if len(contours) >=2:
                        #find biggest blob
                        maxArea = 0
                        MAindex = 0	    #to get the unwanted frame   
                        distanceX = []	    #delete the left most (for right eye)
                        currentIndex = 0 
                        for cnt in contours:
                            area = cv2.contourArea(cnt)
                            if area!=0.0:
                                center = cv2.moments(cnt)
                                cx,cy = int(center['m10']/center['m00']), int(center['m01']/center['m00'])
                                distanceX.append(cx)	
                                if area > maxArea:
                                    maxArea = area
                                    MAindex = currentIndex
                                currentIndex = currentIndex + 1
                        del contours[MAindex]
                        del distanceX[MAindex]

                    if len(contours) >=2:	#delete the left most blob for right eye
                        if eye == 'right':
                            edgeOfEye = distanceX.index(min(distanceX))
                        else:
                            edgeOfEye = distanceX.index(min(distanceX))
                        del contours[edgeOfEye]
                        del distanceX[edgeOfEye]
                        
                    if len(contours) >= 1:	#get largest blob
                        maxArea = 0
                        for cnt in contours:
                            area = cv2.contourArea(cnt)
                            if area > maxArea:
                                maxArea = area
                                largeBlob = cnt

                    #tracking pupil movement and sending command to arduino using Serial to control motors            
                    if len(largeBlob) > 0:	
                        center = cv2.moments(largeBlob)
                        cx,cy = int(center['m10']/center['m00']), int(center['m01']/center['m00'])
                        cv2.circle(pupilO,(cx,cy),5,255,-1)
                        cent.append([(cx+x1),(cy+y1+((h1-y1)*.25))])
                        if eye=='right':
                            right_eye_x=(x+cx+x1)
                            if ((right_eye_x-prev_right_eye_x)>50):
                                print "moved left"
                                ser.write("L")
                                time.sleep(0.5)
                            elif ((right_eye_x-prev_right_eye_x)<-50):
                                print "moved right"  
                                ser.write("R")
                                time.sleep(0.5)
                            else:
                                print "stayed" 
                                ser.write("F")
                                time.sleep(0.5)
                            prev_right_eye_x=right_eye_x
                        else:
                            left_eye_x=(x+cx+x1)
                            if ((left_eye_x-prev_left_eye_x)>50):
                                print "moved left"
                                ser.write("L")
                                time.sleep(0.5)
                            elif ((left_eye_x-prev_left_eye_x)<-50):
                                print "moved right"
                                ser.write("R")
                                time.sleep(0.5)
                            else:
                                print "stayed" 
                                ser.write("F")
                                time.sleep(0.5)
                            prev_left_eye_x=left_eye_x
            
            cv2.imshow('frame',pupilO)    
            dt = clock() - t
    
            draw_str(vis, (20, 20), 'time: %.1f ms' % (dt*1000))
            
            for dx,dy in cent:
                cv2.circle(vis,(x+dx,y+int(dy)),5,255,-1)
                print dx,' ' ,int(dy)
                
            u+=1
            boxes1=rects[0]
            cv2.imshow('facedetect', vis)
            if 0xFF & cv2.waitKey(5) == 27:
                break
    cv2.destroyAllWindows()
