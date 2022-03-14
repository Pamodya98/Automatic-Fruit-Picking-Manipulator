# -*- coding: utf-8 -*-
"""
Created on Sun Feb 27 12:02:57 2022

@author: Pamodya Peiris

streaming code
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

"""

import pyrealsense2 as rs
import numpy as np
import cv2
import time
from B_centers import Circles
#from B_xyz import XYZ
from xyz_test import XYZ
from B_inverse_kinematics import InverseKinematics
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
        #print('time: ', (time.time() - start_time)%60)
        t = 30
        if count == 0 and ((time.time() - start_time) % t < 0.05):
            cv2.imwrite('C:/University/Winter 2022/EE175B/images/B_color_balls.png', color_image)
            print('writing')
            #find the centers of the red balls
            circles = Circles(color_image)
                
            #maybe sort/remove duplicate
            
            
            #get x y z for camera frame
            try:
                x = circles[0][0][0]
                y = circles[0][0][1]
            except IndexError:
                print("no circles found")
                
            try:
                depth = depth_frame.get_distance(x,y)   
            except NameError:
                print("no circles found depth")
            interinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics() 
            xzy = rs.rs2_deproject_pixel_to_point(interinsics, [x,y], depth)
            
            #get cordinates from the robot world frame
            position = XYZ(xzy)
            
            #get the inverse kinematics
            servos = InverseKinematics(position)
            print(servos)
            
            #save angle to file to sent it to arduino
            # data = np.asarray(angles)
            # np.savetxt('theta.txt', data)
            # print('saving file')
            
            #serial connection to arduino
            #SerialConnection(servos)
            
            #wait for 30 seconds
            #time.sleep(120)
            #count += 1
            t = 120
        


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