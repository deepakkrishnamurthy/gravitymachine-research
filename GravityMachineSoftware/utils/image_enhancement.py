# -*- coding: utf-8 -*-
"""
Created on Fri May  4 15:37:21 2018

@author: Francois
"""
import cv2
import numpy as np

def Contrast_Brightness(image,contrast,brightness):
    alpha=(contrast+50)/50
    
    beta=brightness
    print('number of channel',len(cv2.split(image)))
    b, g, r = cv2.split(image)


    sum1=np.sum(cv2.reduce(b,0,cv2.REDUCE_SUM,dtype=cv2.CV_32S)+cv2.reduce(g,0,cv2.REDUCE_SUM,dtype=cv2.CV_32S)+cv2.reduce(r,0,cv2.REDUCE_SUM,dtype=cv2.CV_32S))
    
    cv2.multiply(b, alpha, b)
    cv2.multiply(r, alpha, r)
    cv2.multiply(g, alpha, g)
    
    sum2=np.sum(cv2.reduce(b,0,cv2.REDUCE_SUM,dtype=cv2.CV_32S)+cv2.reduce(g,0,cv2.REDUCE_SUM,dtype=cv2.CV_32S)+cv2.reduce(r,0,cv2.REDUCE_SUM,dtype=cv2.CV_32S))
    
    
    beta2=(sum1-sum2)/image.size

   
    cv2.add(b, beta+beta2,b)
    cv2.add(r, beta+beta2,r)
    cv2.add(g, beta+beta2,g)
    
    image = cv2.merge((b, g, r))
    return image

def Saturation(image_rgb,saturation):
    img_hsv=cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
    h, s, v = cv2.split(img_hsv)
    cv2.add(s, saturation,s)
    img_hsv = cv2.merge((h, s, v))
    image_rgb=cv2.cvtColor(img_hsv, cv2.COLOR_HSV2RGB)
    return image_rgb



def Apply_Clahe(img):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    return clahe.apply(img)
    
    
    