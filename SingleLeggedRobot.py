from CAMELRaisimLib import Robot
import numpy as np
import math

"""
[4 frames]    0: "fixed" 1: "linear_guide" 2: "hip_pitch" 3: "knee_pitch"
[4 bodies]    0: "base" 1: "hip" 2: "upper_leg" 3:"lower_leg"
[Generalized Coordinate]  0: "linear_guide" 1: "hip_pitch" 2: "knee_pitch"
"""
class SingleLeggedRobot(Robot.Robot):

    # override
    def initialize(self):
        initialPosition = np.array([0, math.pi / 4.0, - math.pi / 2.0])
        initialVelocity = np.array([0, 0, 0])
        self.setState(initialPosition, initialVelocity)
        
    def getHipQ(self):
        return self.getGeneralizedCoordinate()[1]   # hip joint angular position , 1: hip_revolute_joint

    def getKneeQD(self):
        return self.getGeneralizedVelocity()[2]   # knee joint angular position
    
    def getKneeQ(self):
        return self.getGeneralizedCoordinate()[2]   # knee joint angular position, 2: knee_revolute_joint

    def getHipQD(self):
        return self.getGeneralizedVelocity()[1]   # hip joint angular position

    

