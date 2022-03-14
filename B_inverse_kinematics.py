# -*- coding: utf-8 -*-
"""
Created on Mon Feb 21 15:56:50 2022

@author: Pamodya Peiris
Input: The x y z position of the red ball according to the robot frame
Output: angles for servo 3-6 respectively theta 4-1
Description: 
    Servo 6: angle between y and x
    Servo 3 is a range: when the ball is within a radias of - 
    the corresponding angles are -
    Servo 4 and 5 are calculated using the cosine rule

"""


from math import atan, sqrt, acos, pi, cos

def InverseKinematics(point):
    x = point[0]
    y = point[1]
    z = point[2] + 0.03
    
    #robot links in meters
    link1 = 0.096 #0.07
    link2 = 0.105 #0.125
    link3 = 0.089 #0.095
    link4 = 0.135 #offset for the gripper
    
    
    #to get to the position - theta1
    if x==0:
        theta1 = 90   
    elif y==0:
        if x > 0:
            theta1 = 0
        elif x < 0:
            theta1 = 180
    else:
        theta1rad = atan(y/x)
        theta1 = theta1rad * 180 / pi
        
        if x < 0:
            theta1 += 180
    
    #length to x, y
    a = sqrt(x**2 + y**2) - 0.01
    # print(a)
    b = link1 - z
    c = sqrt(a**2 + b**2)
    
    
    #theta4 has to be determined first to find the rest
    # theta4 = [170:100] corresponts to a = [12cm:28cm]
    #14cm - 150  26cm-110
    theta4 = 110 - ((110 - 150) * (0.26 - a)) / (0.26 - 0.14)
    #theta4 = 105 - ((105 - 160) * (0.27 - a)) / (0.27 - 0.14)
    theta4rad = theta4 * pi / 180
    
    # for p
    p = (link3**2) + (link4**2) - (2*link3*link4*cos(3*pi/2 - theta4rad))
    p = sqrt(p)
    # print(p)
    
    #alpha
    alpha = atan(b/a)
    # print(alpha)
    
    #theta2
    t2 = ((link2**2) + (c**2) - (p**2)) / (2*link2*c)
    # print(t2)    
    theta2rad = acos(t2) - alpha
    theta2 = theta2rad * 180 / pi
    
    #for theta3
    b3 = ((link2**2) + (p**2) - (c**2)) / (2*link2*p)
    # print(b3)
    beta = acos(b3) - pi/2
    
    r3 = ((link3**2) + (p**2) - (link4**2)) / (2*link3*p)
    # print(r3)
    gamma = acos(r3)
    
    theta3rad = pi - beta - gamma
    theta3 = theta3rad * 180 / pi
    
    if theta3 > 180.0:
        theta3 = 180
    
    
    #theta5 - always has to be 90 to properly grab it in the project
    #theta5 = 90
    
    #theta6 - open/close given from seconds
    #theta6 = 0
    

    #thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
    #print(thetas)
    
    servos = [theta4, theta3, theta2, theta1]
    
    return servos

# if __name__ == "__main__":
#     points = [0.17154818824184487, 0.15164693387059577, 0.02]
#     #add a 1mm to the position
#     joints = InverseKinematics(points)
#     print(joints)