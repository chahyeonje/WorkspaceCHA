# -*- coding: utf-8 -*-
# -*- coding: euc-kr -*-

import imp
import math
import numpy as np
import time
import threading

import Control
import Lidar
import UI
import Main
import Gps

map_count = 0

next_x_previous = 0
next_y_previous = 0

map_visual = np.zeros((41, 41), dtype=np.int)
map_Path = np.zeros((41, 41), dtype=np.int)

x_by_gps = 0.0
y_by_gps = 0.0

visual_lock = threading.Lock()

from warnings import warn
import heapq

class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __repr__(self):
      return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
      return self.f < other.f
    
    # defining greater than for purposes of heap queue
    def __gt__(self, other):
      return self.f > other.f

def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar(maze, start, end, allow_diagonal_movement = True):

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Heapify the open_list and Add the start node
    heapq.heapify(open_list) 
    heapq.heappush(open_list, start_node)

    # Adding a stop condition
    outer_iterations = 0
    max_iterations = (len(maze[0]) * len(maze) // 3)

    # what squares do we search
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)
    if allow_diagonal_movement:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)

    # Loop until you find the end
    while len(open_list) > 0:
        outer_iterations += 1

        if outer_iterations > max_iterations:
          # if we hit this point return the path such as it is
          # it will not contain the destination
          warn("giving up on pathfinding too many iterations")
          return[]
          #  return return_path(current_node)       
        
        # Get the current node
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            return return_path(current_node)

        # Generate children
        children = []
        
        for new_position in adjacent_squares: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            if child in open_list: 
                idx = open_list.index(child) 
                if child.g < open_list[idx].g:
                    # update the node in the open list
                    open_list[idx].g = child.g
                    open_list[idx].f = child.f
                    open_list[idx].h = child.h
            else:
                # Add the child to the open list
                heapq.heappush(open_list, child)

    warn("Couldn't get a path to destination")
    return None

def align_coordinates(x,y,theta): # to x axis
    radian = theta*math.pi/180
    x_changed = x*math.cos(radian) - y*math.sin(radian)
    y_changed = x*math.sin(radian) + y*math.cos(radian)
    return x_changed, y_changed

def real_to_map(x,y, half, resolution):
  x_return = -(int(x/resolution) - half)
  if x_return > half:
    x_return += 1
  y_return = -(int(y/resolution) - half) 
  if y_return > half:
    y_return += 1
  return x_return, y_return

def main():
  global map_count
  global next_x_previous, next_y_previous
  global map_Path, map_visual
  global x_by_gps, y_by_gps
  # gps = Gps.Gps()
  # gps.setupGps(35.2323, 129.0824)
  # gps.setupGps(35.2430, 129.0824)
  # time.sleep(0.99)
  k = 0
  while 1:
    start = (Lidar.half, Lidar.half) # center = (50,50)
    x_goal, y_goal = align_coordinates(Control.x_Goal_now - Control.x_current, Control.y_Goal_now - Control.y_current, -(Control.y))
    # if k == 500:
    #   x_by_gps, y_by_gps = gps.getDistanceCurrentToGoal()
    #   k = 0
    # k = k + 1
    # print(k, x_by_gps, y_by_gps)
    # x_goal, y_goal = align_coordinates(x_by_gps/30, -y_by_gps/30, -(Control.y))
    x_goal, y_goal = real_to_map(x_goal, y_goal, Lidar.half, Lidar.resolution)
    if x_goal < 0:
      x_goal = 0
    if y_goal < 0:
      y_goal = 0
    if x_goal > (Lidar.size - 1):
      x_goal = (Lidar.size - 1)
    if y_goal > (Lidar.size - 1):
      y_goal = (Lidar.size - 1)
    end = (x_goal, y_goal)

    Main.lock.acquire()
    map = map_Path.copy()
    Main.lock.release()
    if map[Lidar.half][Lidar.half] == 0:
      path = astar(map, (start[0], start[1]), (end[0], end[1]))
    else:
      continue

    # if type(path) is not type(None): 
    if len(path) > 1 :
      Control.next_x = path[1][0]
      Control.next_y = path[1][1]
      for i in range(0,len(path)-1):
        x = path[i][0]
        y = path[i][1]
        map[x][y] = 44444
      x = path[len(path)-1][0]
      y = path[len(path)-1][1]
      map[x][y] = 33333
    else:
      pass
    map_str = ''
    for i in range(0, Lidar.size):
      for j in range(0, Lidar.size):
          if i == Lidar.half and j == Lidar.half:
              map_str += '▦ '
          elif 11111 > map[i][j] > 0:   
              map_str += '■ ' # obstacle
          elif map[i][j] == 0: 
              map_str += '□ ' # no obstacle
          elif map[i][j] == 11111: 
              map_str += '▣ ' # can't go
          elif map[i][j] == 44444:
              map_str += '▦ ' # path
          elif map[i][j] == 33333:
              map_str += '◙ ' # goal
          elif map[i][j] == 55555:
              map_str += '▦ ' # direction
      map_str += '\n'
    time.sleep(0.00001)
    UI.map_ui.setText(str(map_str))

    visual_lock.acquire()
    map_visual = map.copy()
    visual_lock.release()
    
