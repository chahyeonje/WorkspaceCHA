#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import rospy

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

import Path
import Control
import Main

height = 0.20
size = 41
resolution = 0.05 # m
half = int(size/2)
threshold = 20
frame_count = 0
is_go = 0

def set_threshold(map):
    for i in range(0,size):
        for j in range(0,size):
            if map[i][j] < threshold:
                map[i][j] = 0
    return map

def real_to_map(x,y):
  x_return = -(int(x/resolution) - half)
  if x_return > half:
    x_return += 1
  y_return = -(int(y/resolution) - half) 
  if y_return > half:
    y_return += 1
  return x_return, y_return

def block_right(map):
    for i in range(0, int(half/4)):
        map[half - 2 + i][half + i + 2] = 55555
        map[half - 2 + i][half + i + 1] = 55555
    return map

def block_left(map):
    for i in range(0, int(half/4)):
        map[half - 2 + i][half - i - 2] = 55555
        map[half - 2 + i][half - i - 1] = 55555
    return map

def robot_size_clear(map):
    for i in range(-5, 8): # vertical
        for j in range(-3,4):
            map[half + i][half + j] = 0
    return map

def marker(map):
  for i in range(0, size):
    for j in range(0, size):
      if 11111 > map[i][j] > 0 and map[i][j] != 55555: # obstacle maping
        for k in range(-7, 5): # vertical
          for l in range(-3, 4): 
            if (i+k < size and j+l <size) and (i+k > 0 and j+l > 0) and map[i+k][j+l] == 0:
              map[i+k][j+l] = 11111
  # for the empty sides
  for i in range(0, size):
    map[0][i] = 0 # tops
    map[size - 1][i] = 0 # bottom
    map[i][0] = 0 # left
    map[i][size - 1]= 0 # right
  return map

def write_map(data):
    global is_go, frame_count, map
    map = np.zeros((41, 41), dtype=np.int)
    pcd_data = list(pc2.read_points(data, skip_nans=True, field_names = ("x", "y", "z")))
    for point in pcd_data:
        z = point[2]
        if abs(z) < height and abs(point[0]) < 1.0 and abs(point[1]) < 1.0 :
            map[+int(point[0]/resolution) - half][(int(point[1]/resolution) + half)] += 1
    map = set_threshold(map)
    map = robot_size_clear(map)
    map = marker(map)
    if (Control.direction == 'L' or Control.direction == "L_S"):
        map = block_right(map)
    if (Control.direction == 'R' or Control.direction == "R_S"):
        map = block_left(map)
    Main.lock.acquire()
    Path.map_Path = map.copy()
    Main.lock.release()
    # print(time.time())

def main():
    rospy.Subscriber("/lslidar_point_cloud", PointCloud2, write_map, queue_size=1)
    rospy.spin()