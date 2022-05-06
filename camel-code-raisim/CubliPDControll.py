from CAMELController import PDController
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectory1D
import numpy as np

class CubliPDController(PDController):

    def __init__(self, robot):
        super().__init__(robot)
        self.setPDGain(PGain=100.0, DGain=10.0)
        self.trajectoryGenerator = ThirdOrderPolynomialTrajectory1D()
        self.updateState()
        self.trajectoryGenerator.updateTrajectory(self.position, 0.0, self.robot.getTime(), 20.0)
        self.setTorqueLimit(10.0)

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
        print(self.torque)
    # override
    def setControlInput(self):
        if (self.torqueLimit < self.torque):
            self.robot.setGeneralizedForce(np.array([0, self.torqueLimit]))
        elif(-self.torqueLimit > self.torque):
            self.robot.setGeneralizedForce(np.array([0, -self.torqueLimit]))
        else:
            self.robot.setGeneralizedForce(np.array([0, self.torque]))        

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
