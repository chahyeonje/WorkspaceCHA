from CAMELRaisimLib import Robot
import numpy as np
import math
import os

"""
path of the robot's urdf file and name of the robot should be set by user.
urdfPath : Path and name of 'robot.urdf' file.
name     : Name of robot

[4 frames]    0: "fixed" 1: "linear_guide" 2: "hip_pitch" 3: "knee_pitch" 4: "ankle_pitch"
[4 bodies]    0: "base" 1: "hip" 2: "upper_leg" 3: "lower_leg" 4: "foot"
[Generalized Coordinate]  0: "linear_guide" 1: "hip_pitch" 2: "knee_pitch" 3: "ankle_pitch"
"""
class SingleLegged3DOFRobot(Robot):

    def __init__(self, sim):
        urdfPath = os.path.dirname(os.path.realpath(__file__))+"/rsc/camel_single_leg_3dof.urdf"
        name = 'single_leg'
        super().__init__(sim, urdfPath, name)
        self.initialize()
        self.mass = 5.0
        self.upperLegLength = 0.3
        self.lowerLegLength = 0.3
        self.footLength = 0.1

    # override
    def initialize(self):
        initialPosition = np.array([0.42426, -math.pi * 0.25, math.pi * 0.5, -math.pi * 0.75])
        initialVelocity = np.array([0.0, 0.0, 0.0, 0.0])
        self.setState(initialPosition, initialVelocity)
        
    def getQ(self):
        return self.getGeneralizedCoordinate()

    def getQD(self):
        return self.getGeneralizedVelocity()
    
    def getHipPositionZ(self):
        return self.getQ()[0]

    def getHipVelocityZ(self):
        return self.getQD()[0]

    def getHipAngularPosition(self):
        return self.getQ()[1]

    def getKneeAngularPosition(self):
        return self.getQ()[2]

    def getAnkleAngularPosition(self):
        return self.getQ()[3]

    def getHipAngularVelocity(self):
        return self.getQD()[1]

    def getKneeAngularVelocity(self):
        return self.getQD()[2]
    
    def getAnkleAngularVelocity(self):
        return self.getQD()[3]

    def getMass(self):
        return self.mass

    def getUpperLegLength(self):
        return self.upperLegLength
    
    def getLowerLegLength(self):
        return self.lowerLegLength

    def getFootLength(self):
        return self.footLength