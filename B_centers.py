# -*- coding: utf-8 -*-
"""
Created on Sun Feb 27 12:51:28 2022

@author: Pamodya Peiris

Input: color image after 30 seconds
Output: list of center of the identified red circles
Description: Extract the R image and threshold, dilate to get the full circle
             HoughCircles to identify the centers of the image
"""


import numpy as np
import cv2
from skimage.morphology import disk, dilation #, erosion

def Circles(I):
    #read image
    # image_folder = "C://University//Winter 2022//EE175B//images//"
    # I = cv2.imread(image_folder+"B_color_balls.png", cv2.IMREAD_COLOR)
    
    #split image
    B, G, R = cv2.split(I)
    
    #binarize image
    (thresh, im_bw) = cv2.threshold(R, 200, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    #remove noise
    # s = disk(3, dtype=np.uint8)
    # red = erosion(im_bw, selem=s)
    
    s = disk(3, dtype=np.uint8)
    red = dilation(im_bw, selem=s)
    
    #draw hough circles
    img = red
    # circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT_ALT,1,1,
                                # param1=100,param2=0.8,minRadius=7,maxRadius=0)
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT_ALT,1,1,
                                param1=100,param2=0.87,minRadius=16,maxRadius=50)
    try:
        circles = np.uint16(np.around(circles))
    except TypeError:
        circles = list()
        print("no circles")
        
    return circles




            