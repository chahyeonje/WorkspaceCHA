import numpy as np
from CAMELController import MPCController
from CAMELOptimizer import GradientDescentSolver
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectory1D

class SimplePendulumMPCController(MPCController):
    def __init__(self, robot):
        super().__init__(robot)
        self.GDSolver = GradientDescentSolver()
        self.setSolver(self.GDSolver)
        self.setMPCHorizon(10)
        
        self.trajectoryGenerator = ThirdOrderPolynomialTrajectory1D()
        self.trajectoryGenerator.updateTrajectory(currentPosition=self.robot.getQ(), goalPosition= 2.57, currentTime= self.robot.getTime(), timeDuration=10.0)

        self.setTorqueLimit(50)
    
    # override
    def doControl(self):
        return super().doControl()

    # override
    def setTrajectory(self, trajectorySequence):
        return super().setTrajectory(trajectorySequence)

    # override
    def updateState(self):
        self.position = self.robot.getQ()
        self.velocity = self.robot.getQD()        
    
    # override
    def computeControlInput(self):
        return super().computeControlInput()

    # override
    def setControlInput(self):
        if (self.torqueLimit < self.torque):
            self.robot.setGeneralizedForce(np.array([self.torqueLimit]))
        elif(-self.torqueLimit > self.torque):
            self.robot.setGeneralizedForce(np.array([-self.torqueLimit]))
        else:
            self.robot.setGeneralizedForce(np.array([self.torque]))        

    def setTorqueLimit(self, torqueLimit):
        self.torqueLimit = torqueLimit

    # override
    def setMPCHorizon(self, MPCHorizon):
        return super().setMPCHorizon(MPCHorizon)
    
    # override
    def setSolver(self, solver):
        return super().setSolver(solver)