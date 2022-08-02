#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from std_msgs.msg import String
import sys

from PyQt5 import QtWidgets
from PyQt5 import QtGui
from PyQt5 import QtCore

import Path
import Control
# import Command

def UI_Thread():
    global main_Window, map_ui, current_state, goal_state

    def change_velocity():
        val = velocity_slider.value()
        Control.velocity = val/100
        velocity_label.setText("Velocity : " + str(Control.velocity))

    def linear_go_pressed():
        Control.A1_msg.linear.x = Control.velocity

    def linear_go_released():
        Control.A1_msg.linear.x = 0

    def linear_back_pressed():
        Control.A1_msg.linear.x = -Control.velocity

    def linear_back_released():
        Control.A1_msg.linear.x = 0

    def linear_left_pressed():
        Control.A1_msg.linear.y = Control.velocity

    def linear_left_released():
        Control.A1_msg.linear.y = 0

    def linear_right_pressed():
        Control.A1_msg.linear.y = -Control.velocity

    def linear_right_released():
        Control.A1_msg.linear.y = 0

    def angular_left_pressed():
        Control.A1_msg.angular.z = Control.velocity

    def angular_left_released():
        Control.A1_msg.angular.z = 0

    def angular_right_pressed():
        Control.A1_msg.angular.z = -Control.velocity

    def angular_right_released():
        Control.A1_msg.angular.z = 0

    def goal_x_up():
        Control.Goals[0][0] += 1.0
    
    def goal_x_up_detail():
        Control.Goals[0][0] += 0.1

    def goal_x_down():
        Control.Goals[0][0] += -1.0
    
    def goal_x_down_detail():
        Control.Goals[0][0] += -0.1

    def goal_y_up():
        Control.Goals[0][1] += 1.0
    
    def goal_y_up_detail():
        Control.Goals[0][1] += 0.1

    def goal_y_down():
        Control.Goals[0][1] += -1.0
    
    def goal_y_down_detail():
        Control.Goals[0][1] += -0.1

    def auto_manual():
        Control.is_Auto = (Control.is_Auto + 1) % 2
        # if(Control.is_Auto == 0):
        #     print("Manual mode")
        # elif(Control.is_Auto == 1):
        #     print("Auto mode")

    def stop():
        Control.A1_msg.linear.x = 0.0
        Control.A1_msg.linear.y = 0.0
        Control.A1_msg.angular.z = 0.0
        Control.is_Auto = 0


    window_width =  1200 #1295
    window_height = 580

    app = QtWidgets.QApplication(sys.argv) 

    main_Window = QtWidgets.QLabel()
    main_Window.setWindowTitle("A1_Controller")
    main_Window.resize(window_width,window_height)
    # main_Window.setStyleSheet("color: black;""background-color: #000000")
    main_Window.show()

    map_ui = QtWidgets.QLabel(main_Window)
    map_ui.setFont(QtGui.QFont('Arial Black',8))
    map_ui.setGeometry(10, 0, 700, 570)
    map_ui.show()

    velocity_label = QtWidgets.QLabel(main_Window)
    velocity_label.setFont(QtGui.QFont('Arial Black',15))
    velocity_label.move(600, 10)
    velocity_label.resize(250, 25)
    # velocity_label.setText(str(Control.velocity))
    velocity_label.show()

    velocity_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, main_Window)
    velocity_slider.move(600, 30)
    velocity_slider.resize(250, 25)
    velocity_slider.setRange(0, 25)
    velocity_slider.valueChanged.connect(change_velocity)
    velocity_slider.show()

    linear_x_p = QtWidgets.QPushButton("X_Go", main_Window)
    linear_x_p.move(600, 50)
    linear_x_p.resize(250, 25)
    linear_x_p.pressed.connect(linear_go_pressed)
    linear_x_p.released.connect(linear_go_released)
    linear_x_p.show()

    linear_x_n = QtWidgets.QPushButton("X_Back", main_Window)
    linear_x_n.move(600, 80)
    linear_x_n.resize(250, 25)
    linear_x_n.pressed.connect(linear_back_pressed)
    linear_x_n.released.connect(linear_back_released)
    linear_x_n.show()

    linear_y_p = QtWidgets.QPushButton("liear_Y_L", main_Window)
    linear_y_p.move(600, 110)
    linear_y_p.resize(250, 25)
    linear_y_p.pressed.connect(linear_left_pressed)
    linear_y_p.released.connect(linear_left_released)
    linear_y_p.show()

    linear_y_n = QtWidgets.QPushButton("liear_Y_R", main_Window)
    linear_y_n.move(600, 140)
    linear_y_n.resize(250, 25)
    linear_y_n.pressed.connect(linear_right_pressed)
    linear_y_n.released.connect(linear_right_released)
    linear_y_n.show()

    angular_z_p = QtWidgets.QPushButton("angular_Z_L", main_Window)
    angular_z_p.move(600, 170)
    angular_z_p.resize(250, 25)
    angular_z_p.pressed.connect(angular_left_pressed)
    angular_z_p.released.connect(angular_left_released)
    angular_z_p.show()

    angular_z_n = QtWidgets.QPushButton("angular_Z_R", main_Window)
    angular_z_n.move(600, 200)
    angular_z_n.resize(250, 25)
    angular_z_n.pressed.connect(angular_right_pressed)
    angular_z_n.released.connect(angular_right_released)
    angular_z_n.show()

    x_up = QtWidgets.QPushButton("X_Up", main_Window)
    x_up.move(600, 250)
    x_up.resize(250, 25)
    x_up.pressed.connect(goal_x_up)
    x_up.show()

    x_up_detail = QtWidgets.QPushButton("X_Up_detail", main_Window)
    x_up_detail.move(600, 280)
    x_up_detail.resize(250, 25)
    x_up_detail.pressed.connect(goal_x_up_detail)
    x_up_detail.show()

    x_down = QtWidgets.QPushButton("X_Down", main_Window)
    x_down.move(600, 310)
    x_down.resize(250, 25)
    x_down.pressed.connect(goal_x_down)
    x_down.show()

    x_down_detail = QtWidgets.QPushButton("X_Down_detail", main_Window)
    x_down_detail.move(600, 340)
    x_down_detail.resize(250, 25)
    x_down_detail.pressed.connect(goal_x_down_detail)
    x_down_detail.show()

    y_up = QtWidgets.QPushButton("Y_Up", main_Window)
    y_up.move(600, 370)
    y_up.resize(250, 25)
    y_up.pressed.connect(goal_y_up)
    y_up.show()
    y_up.show()

    y_up_detail = QtWidgets.QPushButton("Y_Up_detail", main_Window)
    y_up_detail.move(600, 400)
    y_up_detail.resize(250, 25)
    y_up_detail.pressed.connect(goal_y_up_detail)
    y_up_detail.show()

    y_down = QtWidgets.QPushButton("Y_Down", main_Window)
    y_down.move(600, 430)
    y_down.resize(250, 25)
    y_down.pressed.connect(goal_y_down)
    y_down.show()

    y_down_detail = QtWidgets.QPushButton("Y_Down_detail", main_Window)
    y_down_detail.move(600, 460)
    y_down_detail.resize(250, 25)
    y_down_detail.pressed.connect(goal_y_down_detail)
    y_down_detail.show()

    auto_button = QtWidgets.QPushButton("Auto/No", main_Window)
    auto_button.move(600, 510)
    auto_button.resize(250, 25)
    auto_button.pressed.connect(auto_manual)
    auto_button.show()

    stop_button = QtWidgets.QPushButton("Stop", main_Window)
    stop_button.move(600, 540)
    stop_button.resize(250, 25)
    stop_button.pressed.connect(stop)
    stop_button.show()

    state_ref = 900

    current_state = QtWidgets.QLabel(main_Window)
    current_state.setFont(QtGui.QFont('Arial Black',15))
    current_state.move(state_ref, 50)
    current_state.resize(250, 150)
    current_state.show()

    goal_state = QtWidgets.QLabel(main_Window)
    goal_state.setFont(QtGui.QFont('Arial Black',15))
    goal_state.move(state_ref, 200)
    goal_state.resize(250, 100)
    goal_state.show()

    sys.exit(app.exec_())

# def UI_Thread():
#     # rospy.init_node('controller')

#     # pub = rospy.Publisher('map', String, queue_size=1)
#     # rate = rospy.Rate(5) 
#     while 1:
#         os.system('clear')
#         # Path.visual_lock.acquire()
#         map = Path.map_visual.copy()
#         # Path.visual_lock.release()
#         map_str = ''
#         for i in range( Lidar.half-13, Lidar.half+14):
#             for j in range( Lidar.half-13, Lidar.half+14):
#         # for i in range(0, Lidar.size):
#         #   for j in range(0, Lidar.size):
#                 if i == Lidar.half and j == Lidar.half:
#                     map_str += '▦ '
#                 elif 11111 > map[i][j] > 0:   
#                     map_str += '■ ' # obstacle
#                 elif map[i][j] == 0: 
#                     map_str += '□ ' # no obstacle
#                 elif map[i][j] == 11111: 
#                     map_str += '▣ ' # can't go
#                 elif map[i][j] == 44444:
#                     map_str += '▦ ' # path
#                 elif map[i][j] == 33333:
#                     map_str += '◙ ' # goal
#                 elif map[i][j] == 55555:
#                     map_str += '▦ ' # direction
#             map_str += '\n'
#         print(map_str)
#         # pub.publish(map_str)
#         # rate.sleep()
