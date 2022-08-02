#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cmath import pi
import numpy as np
import rospy
import time
import numpy as np
import math

from geometry_msgs.msg import Twist
from unitree_legged_msgs.msg import HighState
from std_msgs.msg import String

import Lidar
import UI
# import Command

foward_weight_Val = 1.0
rotational_weight_Val = 1.0
location_Mat = np.zeros((3,2))
height = 0.34
x_current = 0
y_current = 0
linear_x = 0.0
linear_y = 0.0
angular_z = 0.0
r = 0.0
p = 0.0
y = 0.0
direction = ''
# direction_previous = ''
direction_count = 0

no_goal = 0
Goals = [[0.0, 0.0]]
Goal_num = len(Goals)
Goal_Count = 0
x_Goal_now = Goals[0][0]
y_Goal_now = Goals[0][1]
y_Goal_final = Goals[Goal_num-1][1]
x_Goal_final = Goals[Goal_num-1][0]
Goal_final = Goals[Goal_num-1]

fast_rotation = 0

next_x = 0
next_y = 0
path_num_previous = 0

def get_theta(x, y):
    # r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(x, y)*180/pi
    '''if theta < 0:
        theta = 360 + theta'''
    return theta

def align_coordinates(y,x,theta):
    radian = theta*math.pi/180
    y_changed = y*math.cos(radian) - x*math.sin(radian)
    x_changed = y*math.sin(radian) + x*math.cos(radian)
    return y_changed, x_changed

def quaternion_to_euler(quarternion):

    x = quarternion[0]
    y = quarternion[1]
    z = quarternion[2]
    w = quarternion[3]

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.degrees(math.atan2(t0, t1))
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.degrees(math.asin(t2))
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.degrees(math.atan2(t3, t4))
    
    return roll_x, pitch_y, yaw_z # in radians

def get_state(states):
    global x_current, y_current
    global linear_y, linear_x, angular_z
    global r, p, y

    x_current = states.forwardPosition
    y_current = states.sidePosition
    quarternion = states.imu.quaternion
    r, p, y = quaternion_to_euler(quarternion)
   

def which_quadrant(y,x):
    if y*x >= 0:
        if y >= 0:
            return 1
        else:
            return 3
    else:
        if y >= 0:
            return 2
        else:
            return 4

def drection_controller():
    global x_current, y_current
    global next_x, next_y
    global direction_previous
    returning_direction = ""

    angle = get_theta(-(next_x - Lidar.half), next_y - Lidar.half)

    if angle == 90:
        returning_direction = "G"
    elif angle == -90:
        returning_direction = "B"
    elif angle == 0:
        returning_direction = "R_S"
    elif angle == 180:
        returning_direction = "L_S"
    else:
        if abs(angle) < 90:
            returning_direction = "R"
        elif abs(angle) > 90:
            returning_direction = "L"
    
    direction_previous = returning_direction
    return returning_direction

def is_goal():
    global y_current, x_current
    global y_Goal_now, x_Goal_now
    if abs(y_Goal_now - y_current) < 0.1 and abs(x_current - x_Goal_now) < 0.1:
        return "Y"
    else:
        return "N"

def goal_updater(count):
    global Goals
    global x_Goal_now, y_Goal_now
    if count < len(Goals):
        y_Goal_now = Goals[count][1]
        x_Goal_now = Goals[count][0]

def A1_go(x,y,z):
    A1_msg = Twist()

    A1_msg.linear.x = x
    A1_msg.linear.y = y
    A1_msg.angular.z = z

    return A1_msg

def control_loop():
    global height
    global Goal_Count
    global linear_x, linear_y, angular_z
    global direction, direction_count
    global A1_msg, velocity, is_Auto
    rospy.init_node('controller')
    pub = rospy.Publisher('/unitree/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(50) 

    A1_msg = Twist()
    A1_msg.linear.x = 0
    A1_msg.linear.y = 0
    A1_msg.angular.z = 0

    velocity = 0

    is_Auto = 0

    rospy.Subscriber("/unitree/state_high", HighState, get_state, queue_size=1)
    while not rospy.is_shutdown():
        goal_updater(Goal_Count)
        UI.current_state.setText("<Current>" + '\n' + "x=  " + str(round(x_current, 3)) + '\n' + "y=  " + str(round(y_current, 3)) + '\n' + 'yaw=  ' + str(round(y, 3)) + '\n' + 'velocity=  ' + str(round(velocity,3)))
        UI.goal_state.setText("<Goal>" + '\n' + 'x=  ' + str(round(x_Goal_now, 3)) + '\n' + 'y=  ' + str(round(y_Goal_now, 3)))

        direction_previous = direction

        direction = drection_controller()
        if direction_previous is not direction:
            direction_count = 0 
        else:
            direction_count += 1
        if direction_count > 80:
            fast_rotation = 1
        else:
            fast_rotation = 0

        if is_Auto == 1: 
            if direction == "L":
                if fast_rotation == 0:
                    A1_msg = A1_go(0, 0, velocity/20)
                elif fast_rotation == 1:
                    A1_msg = A1_go(0, 0, velocity*0.8)
                # pub.publish(A1_msg)
            elif direction == "R":
                if fast_rotation == 0:
                    A1_msg = A1_go(0, 0, -velocity/20)
                elif fast_rotation == 1:
                    A1_msg = A1_go(0, 0, -velocity*0.8)
                # pub.publish(A1_msg)
            elif direction == "G":
                A1_msg = A1_go(velocity,0,0)
                # pub.publish(A1_msg)
            elif direction == "B":
                A1_msg = A1_go(-velocity,0,0)
                # pub.publish(A1_msg)
            elif direction == "R_S":
                A1_msg = A1_go(0, -velocity, 0)
                # pub.publish(A1_msg)
            elif direction == "L_S":
                A1_msg = A1_go(0, velocity,0)
            else:
                pass
            # pub.publish(A1_msg)
            if is_goal() == "Y":
                # if Goal_Count < Goal_num:
                #     Goal_Count += 1
                # if Goal_Count == Goal_num:
                #     A1_msg = A1_go(0,0,0)
                #     pub.publish(A1_msg)
                # else:
                #     goal_updater(Goal_Count)
                A1_msg = A1_go(0,0,0)
            pub.publish(A1_msg)
        elif is_Auto == 0:
            pub.publish(A1_msg)
        rate.sleep()
