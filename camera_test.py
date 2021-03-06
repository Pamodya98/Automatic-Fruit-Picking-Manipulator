# -*- coding: utf-8 -*-
"""
Created on Wed Nov 17 14:35:50 2021

@author: Pamodya Peiris
"""


## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

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

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    

# Start streaming
profile = pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
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
        
        #save image
        cv2.imwrite('C:/University/Winter 2022/EE175B/images/color_balls1.png', color_image)
        # cv2.imwrite('C:/University/Winter 2022/EE175B/images/depth_balls.png', depth_colormap)
        
        # depth_image_3d = np.dstack(
        #     (depth_image, depth_image, depth_image)
        # )
        # cv2.imwrite("C:/University/Winter 2022/EE175B/images/depth.png", depth_image)
        
        x = 361
        y = 357
        depth = depth_frame.get_distance(x,y)


        
        # depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        # color_intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        
        interinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics() 
        xyz = rs.rs2_deproject_pixel_to_point(interinsics, [x,y], depth)
        


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