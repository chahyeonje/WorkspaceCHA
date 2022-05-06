from CAMELController import PDController
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectoryNDim
import numpy as np
import math

class DoublePendulumPDController(PDController):

    def __init__(self, robot):
        super().__init__(robot)
        self.n = 2
        self.setPDGain(PGain=100.0, DGain=10.0)
        self.trajectoryGenerator = ThirdOrderPolynomialTrajectoryNDim()
        self.updateState()
        self.trajectoryGenerator.updateTrajectory(self.n, self.position, [math.pi * 0.125, math.pi * 0.25], self.robot.getTime(), 5.0)
        self.setTorqueLimit([10.0,10.0])

    # override
    def doControl(self):
        self.updateState()
        self.setTrajectory(desiredPosition=self.trajectoryGenerator.getPostionTrajectory(self.robot.getTime()), desiredVelocity=self.trajectoryGenerator.getVelocityTrajectory(self.robot.getTime()))
        # print(self.velocity)
        self.computeControlInput()
        self.setControlInput()

    # override
    def setTrajectory(self, desiredPosition, desiredVelocity):
        return super().setTrajectory(desiredPosition, desiredVelocity)

    # override
    def updateState(self):
        self.position = self.robot.getQ()
        self.velocity = self.robot.getQD()           

    # override
    def computeControlInput(self):
        self.positionError = self.desiredPosition - self.position
        self.differentialError = self.desiredVelocity - self.velocity
        self.torque = self.PGain * self.positionError + self.DGain * self.differentialError
        # self.torque = [10.0, -10.0]
    # override
    def setControlInput(self):
        for i in range(1,self.n):
            if (self.torqueLimit[i] < self.torque[i]):
                self.torque[i] = self.torqueLimit[i]
            elif(-1*self.torqueLimit[i] > self.torque[i]):
                self.torque[i] = -1*self.torqueLimit[i]
        self.robot.setGeneralizedForce(self.torque)
        print(self.torque)



        # if (self.torqueLimit < self.torque):
        #     self.robot.setGeneralizedForce(np.array(self.torqueLimit))
        # elif(-1*self.torqueLimit > self.torque):
        #     self.robot.setGeneralizedForce(np.array(-self.torqueLimit))
        # else:
        #     self.robot.setGeneralizedForce(np.array(self.torque))        

    def setTorqueLimit(self, torqueLimit):
        self.torqueLimit = torqueLimit

    def getPosition(self):
        return self.position

    def getVelocity(self):
        return self.velocity

    def getDesiredPosition(self):
        return self.desiredPosition
    
    def getDesiredVelocity(self):
        return self.desiredVelocity

    def getInputTorque(self):
        return self.torque
