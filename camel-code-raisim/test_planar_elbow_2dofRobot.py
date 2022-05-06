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
class test_planar_elbow_2dofRobot(Robot):

    def __init__(self, sim):
        urdfPath = os.path.dirname(os.path.realpath(__file__))+"/rsc/test_planar_elbow_2dof.urdf"
        name = 'single_leg'
        super().__init__(sim, urdfPath, name)
        self.initialize()
        self.mass = 5.0
        self.Arm1Length = 0.5
        self.Arm2Length = 0.5

    # override
    def initialize(self):
        initialPosition = np.array([-math.pi/2, 0])
        initialVelocity = np.array([0, 0])
        self.setState(initialPosition, initialVelocity)
        
    def getQ(self):
        return self.getGeneralizedCoordinate()

    def getQD(self):
        return self.getGeneralizedVelocity()
    
    def getArm1Position(self):
        return self.getQ()[0]

    def getArm1Velocity(self):
        return self.getQD()[0]

    def getArm2Position(self):
        return self.getQ()[1]

    def getArm2Velocity(self):
        return self.getQD()[1]

    def getMass(self):
        return self.mass

    def getArm1Length(self):
        return self.Arm1Length
    
    def getArm2Length(self):
        return self.Arm2Length
