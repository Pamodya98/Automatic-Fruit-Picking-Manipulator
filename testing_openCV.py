# -*- coding: utf-8 -*-
"""
Created on Fri Mar 11 14:26:39 2022

@author: Pamodya Peiris

Description: Testing OpenCV on detecting the centers of the balls
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import time
from B_centers import Circles
#from B_xyz import XYZ
from xyz_test import XYZ
from B_inverse_kinematics import InverseKinematics
from skimage.morphology import disk, dilation #, erosion
# from B_serial import SerialConnection

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    

#start time
start_time = time.time()
count = 0

# Start streaming
profile = pipeline.start(config)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)


try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))
        
        #save image after 30 seconds
        if ((time.time() - start_time) > 10):
            #cv2.imwrite('C:/University/Winter 2022/EE175B/testing/opencv/image'+str(i)+'.png', color_image)
            #print('writing: ', i)
            #find the centers of the red balls
            #circles = Circles(color_image)
            I = color_image
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
            circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT_ALT,1,5,
                                        param1=150,param2=0.88,minRadius=16,maxRadius=50)
            try:
                circles = np.uint16(np.around(circles))
            except TypeError:
                circles = list()
                #print("no circles")
            
            t=10
            try: 
                cimg = color_image
                for i in circles[0,:]:
                    # draw the outer circle
                    color = list(np.random.choice(range(256), size=3))
                    cv2.circle(cimg,(i[0],i[1]),i[2],(int(color[0]),int(color[1]),int(color[2])),2)
                    # draw the center of the circle
                    cv2.circle(cimg,(i[0],i[1]),2,(255,255,255),3)
                    
                images = np.hstack((cimg, depth_colormap))
                
            except TypeError:
                print("no circles found")

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        key = cv2.waitKey(1)
        #how do I stop the streaming
        if key == 27:
            break        

finally:

    # Stop streaming
    pipeline.stop()
    
cv2.destroyAllWindows()

