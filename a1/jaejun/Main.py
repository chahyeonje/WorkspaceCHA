#!/usr/bin/env python
# -*- coding: utf-8 -*-

#####################################################
## Jaejun ## CAMEL ## PNU EE ## 
## First released ## 2022/05/26 ##
## Last modified ## 2022/05/26 ## 
#####################################################

import rospy
import threading
import time

import Lidar
import Path
import Control
import UI
import Command
import Map

lock = threading.Lock()

def value_setting():
    # Set A1's Speed
    Command.velocity = 0.0

if __name__ == '__main__':
    try:
        value_setting()

        lidar_thread = threading.Thread(target=Lidar.main)
        lidar_thread.daemon = True
        lidar_thread.start()

        visual_thread = threading.Thread(target=UI.UI_Thread)
        visual_thread.daemon = True
        visual_thread.start()

        time.sleep(0.2)

        planning_thread = threading.Thread(target=Path.main)
        planning_thread.daemon = True
        planning_thread.start()

        # main Thread
        Control.control_loop()

    except rospy.ROSInterruptException:
        pass