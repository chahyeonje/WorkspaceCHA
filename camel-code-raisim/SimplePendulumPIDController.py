from CAMELController import PIDController
import numpy as np

class SimplePendulumPIDController(PIDController):
    def __init__(self, robot):
        super().__init__(robot)
        self.setPIDGain(150, 1, 30)
        self.setTrajectory(desiredPosition=-1.57, desiredVelocity=0.0)
        self.setTorqueLimit(50)
        
    # override
    def doControl(self):
        self.updateState()
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
        self.cumulativeError += self.positionError
        self.differentialError = self.desiredVelocity - self.velocity
        self.torque = self.PGain * self.positionError + self.IGain * self.cumulativeError + self.DGain * self.differentialError 
        
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

    def getPosition(self):
        return self.position

    def getVelocity(self):
        return self.velocity

    def getDesiredPosition(self):
        return self.desiredPosition
    
    def getDesiredVelocity(self):
        return self.desiredVelocity
