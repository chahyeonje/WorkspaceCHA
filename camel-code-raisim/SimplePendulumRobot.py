from CAMELRaisimLib import Robot
import numpy as np
import math
import os

"""
path of the robot's urdf file and name of the robot should be set by user.
urdfPath : Path and name of 'robot.urdf' file.
name     : Name of robot

[ frames]                 0: "fixed"     1: "top_pitch"
[ bodies]                 0: "base"      1: "wire"
[Generalized Coordinate]  0: "top_pitch"
"""
class SimplePendulumRobot(Robot):
    
    def __init__(self, sim):
        urdfPath = os.path.dirname(os.path.realpath(__file__))+"/rsc/camel_simple_pendulum.urdf"
        name = 'cutePendulum'
        super().__init__(sim, urdfPath, name)
        self.initialize()
        self.mass = 5.0
        self.wireLength = 0.575

    # override
    def initialize(self):
        initialPosition = np.array([math.pi * 0.0])
        initialVelocity = np.array([0.0])
        self.setState(initialPosition, initialVelocity)
        
    def getQ(self):
        return self.getGeneralizedCoordinate()[0]       # 0: top_pitch

    def getQD(self):
        return self.getGeneralizedVelocity()[0]

    def getMass(self):
        return self.mass

    def getWireLength(self):
        return self.wireLength