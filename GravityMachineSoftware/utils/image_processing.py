# -*- coding: utf-8 -*-
"""
Created on Mon May  7 19:44:40 2018

@author: Francois and Deepak
"""

import numpy as np
import cv2
from scipy.ndimage.filters import laplace

#color is a vector HSV whose size is 3


def default_lower_HSV(color):
    c=[0,100,100]
    c[0]=np.max([color[0]-10,0])
    c[1]=np.max([color[1]-40,0])
    c[2]=np.max([color[2]-40,0])
    return np.array(c,dtype="uint8")

def default_upper_HSV(color):
    c=[0,255,255]
    c[0]=np.min([color[0]+10,178])
    c[1]=np.min([color[1]+40,255])
    c[2]=np.min([color[2]+40,255])
    return np.array(c,dtype="uint8")

def threshold_image(image_BGR,LOWER,UPPER):
    image_HSV=cv2.cvtColor(image_BGR,cv2.COLOR_BGR2HSV)
    imgMask = cv2.inRange(image_HSV, LOWER, UPPER)  #The tracked object will be in white
    imgMask = cv2.erode(imgMask, None, iterations=2) # Do a series of erosions and dilations on the thresholded image to reduce smaller blobs
    imgMask = cv2.dilate(imgMask, None, iterations=2)
    
    return imgMask

def threshold_image_gray(image_gray, LOWER, UPPER):
    imgMask = np.array((image_gray >= LOWER) & (image_gray <= UPPER), dtype='uint8')
    
    # imgMask = cv2.inRange(cv2.UMat(image_gray), LOWER, UPPER)  #The tracked object will be in white
    imgMask = cv2.erode(imgMask, None, iterations=2) # Do a series of erosions and dilations on the thresholded image to reduce smaller blobs
    imgMask = cv2.dilate(imgMask, None, iterations=2)
    
    return imgMask



def bgr2gray(image_BGR):
    return cv2.cvtColor(image_BGR,cv2.COLOR_BGR2GRAY)

def crop(image,center,imSize): #center is the vector [x,y]
    imH,imW,*rest=image.shape  #image.shape:[nb of row -->height,nb of column --> Width]
    xmin = max(10,center[0] - int(imSize))
    xmax = min(imW-10,center[0] + int(imSize))
    ymin = max(10,center[1] - int(imSize))
    ymax = min(imH-10,center[1] + int(imSize))
    return np.array([[xmin,ymin],[xmax,ymax]]),np.array(image[ymin:ymax,xmin:xmax])

def get_bbox(cnt):
    return cv2.boundingRect(cnt)


def find_centroid_enhanced(image,last_centroid):
    #find contour takes image with 8 bit int and only one channel
    #find contour looks for white object on a black back ground
    # This looks for all contours in the thresholded image and then finds the centroid that maximizes a tracking metric
    # Tracking metric : current centroid area/(1 + dist_to_prev_centroid**2)
    contours = cv2.findContours(image, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2]
    centroid=False
    isCentroidFound=False
    if len(contours)>0:
        all_centroid=[]
        dist=[]
        for cnt in contours:
            M = cv2.moments(cnt)
            if M['m00']!=0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                centroid=np.array([cx,cy])
                isCentroidFound=True
                all_centroid.append(centroid)
                dist.append([cv2.contourArea(cnt)/(1+(centroid-last_centroid)**2)])

    if isCentroidFound:
        ind=dist.index(max(dist))
        centroid=all_centroid[ind]

    return isCentroidFound,centroid

def find_centroid_enhanced_Rect(image,last_centroid):
    #find contour takes image with 8 bit int and only one channel
    #find contour looks for white object on a black back ground
    # This looks for all contours in the thresholded image and then finds the centroid that maximizes a tracking metric
    # Tracking metric : current centroid area/(1 + dist_to_prev_centroid**2)
    contours = cv2.findContours(image, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2]
    centroid=False
    isCentroidFound=False
    rect = False
    if len(contours)>0:
        all_centroid=[]
        dist=[]
        for cnt in contours:
            M = cv2.moments(cnt)
            if M['m00']!=0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                centroid=np.array([cx,cy])
                isCentroidFound=True
                all_centroid.append(centroid)
                dist.append([cv2.contourArea(cnt)/(1+(centroid-last_centroid)**2)])

    if isCentroidFound:
        ind=dist.index(max(dist))
        centroid=all_centroid[ind]
        cnt = contours[ind]
        xmin,ymin,width,height = cv2.boundingRect(cnt)
        xmin = max(0,xmin)
        ymin = max(0,ymin)
        width = min(width, imW - int(cx))
        height = min(height, imH - int(cy))
        rect = (xmin, ymin, width, height)


    return isCentroidFound,centroid, rect

def find_centroid_basic(image):
    #find contour takes image with 8 bit int and only one channel
    #find contour looks for white object on a black back ground
    # This finds the centroid with the maximum area in the current frame
    contours = cv2.findContours(image, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2]
    centroid=False
    isCentroidFound=False
    if len(contours)>0:
        cnt = max(contours, key=cv2.contourArea)
        M = cv2.moments(cnt)
        if M['m00']!=0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            centroid=np.array([cx,cy])
            isCentroidFound=True
    return isCentroidFound,centroid

def find_centroid_basic_Rect(image):
    #find contour takes image with 8 bit int and only one channel
    #find contour looks for white object on a black back ground
    # This finds the centroid with the maximum area in the current frame and alsio the bounding rectangle. - DK 2018_12_12
    imH,imW = image.shape
    contours = cv2.findContours(image, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2]
    centroid=False
    isCentroidFound=False
    rect = False
    if len(contours)>0:
        # Find contour with max area
        cnt = max(contours, key=cv2.contourArea)
        M = cv2.moments(cnt)

        if M['m00']!=0:
            # Centroid coordinates
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            centroid=np.array([cx,cy])
            isCentroidFound=True

             # Find the bounding rectangle
            xmin,ymin,width,height = cv2.boundingRect(cnt)
            xmin = max(0,xmin)
            ymin = max(0,ymin)
            width = min(width, imW - xmin)
            height = min(height, imH - ymin)
            rect = (xmin, ymin, width, height)

    return isCentroidFound,centroid, rect

def get_image_center_width(image):
    ImShape=image.shape
    ImH,ImW=ImShape[0],ImShape[1]
    return np.array([ImW*0.5,ImH*0.5]),ImW

def get_image_top_center_width(image):
    ImShape=image.shape
    ImH,ImW=ImShape[0],ImShape[1]
    return np.array([ImW*0.5,0.25*ImH]),ImW


def YTracking_Objective_Function(image, color):
    #variance methode
    if(image.size is not 0):
        if(color):
            image = bgr2gray(image)
        mean,std=cv2.meanStdDev(image)
        return std[0][0]**2
    else:
        return 0

#test part
if __name__ == "__main__":
    # Load an color image in grayscale
    rouge=np.array([[[255,0,0]]],dtype="uint8")
    vert=np.array([[[0,255,0]]],dtype="uint8")
    bleu=np.array([[[0,0,255]]],dtype="uint8")

    rouge_HSV=cv2.cvtColor(rouge,cv2.COLOR_RGB2HSV)[0][0]
    vert_HSV=cv2.cvtColor(vert,cv2.COLOR_RGB2HSV)[0][0]
    bleu_HSV=cv2.cvtColor(bleu,cv2.COLOR_RGB2HSV)[0][0]
    
    img = cv2.imread('C:/Users/Francois/Documents/11-Stage_3A/6-Code_Python/ConsoleWheel/test/rouge.jpg')
    print(img)
    img2=cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    
    couleur = bleu_HSV
    LOWER = default_lower_HSV(couleur)
    UPPER = default_upper_HSV(couleur)
    
    img3=threshold_image(img2,LOWER,UPPER)
    cv2.imshow('image',img3)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#for more than one tracked object
'''
def find_centroid_many(image,contour_area_min,contour_area_max):
    contours = cv2.findContours(image, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2]
    count=0
    last_centroids=[]
    for j in range(len(contours)):
        cnt = contours[j]
        if cv2.contourArea(contours[j])>contour_area_min and cv2.contourArea(contours[j])<contour_area_max :
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            last_centroids.append([cx,cy])
            count+=1
    return last_centroids,count
'''

