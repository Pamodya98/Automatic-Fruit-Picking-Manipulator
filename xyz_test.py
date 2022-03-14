# -*- coding: utf-8 -*-
"""
Created on Sun Feb 27 21:31:00 2022

@author: Pamodya Peiris
Input: position of center from the camera frame
Output: position of center from the world frame
Description: camera angle is Rotated by angle and Transposed 
             to the robot world frame
"""

from math import sin, cos, asin

def XYZ(xzy):
    print('xyz', xzy)
    x = xzy[0] 
    y = xzy[2] - 0.46
    z = abs(0.105 - xzy[1])
    position = [0,0,0]
    
    angle = asin(34.1/46)
    
    addz = 0.105
    
    if x < 0 :
        addy = 0
    else:
        addy = -0.035
    
    ry = 0.137 - y * cos(angle) + x * sin(angle) + addy
    rx = - y * sin(angle) - x * cos(angle)
    rz = 0.03 # + zz
    
    position = [rx, ry, rz]
    
    return position

# if __name__ == "__main__":
#     p = [0.038683801889419556, 0.08016824722290039, 0.3400000035762787]
#     position =  XYZ(p)
#     print(position)