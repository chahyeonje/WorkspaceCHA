import math
import numpy as np
from CAMELController import InverseDynamicsController
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectoryNDim

class DoublePendulumIDController(InverseDynamicsController):
    def __init__(self, robot):
        super().__init__(robot)
        self.n = 2
        self.setPDGain(PGain = 200.0, DGain= 20.0)
        self.trajectoryGenerator = ThirdOrderPolynomialTrajectoryNDim()
        self.updateState()
        self.trajectoryGenerator.updateTrajectory(self.n, self.position, [math.pi * 0.125, math.pi * 0.25], self.robot.getTime(), 5.0)
        self.setTorqueLimit([10.0,10.0])

    def setPDGain(self, PGain, DGain):
        return super().setPDGain(PGain, DGain)
    
    # override
    def doControl(self):
        self.updateState()
        self.setTrajectory(desiredPosition=self.trajectoryGenerator.getPostionTrajectory(self.robot.getTime()), desiredVelocity=self.trajectoryGenerator.getVelocityTrajectory(self.robot.getTime()), desiredAcceleration=self.trajectoryGenerator.getAccelerationTrajectory(self.robot.getTime()))
        self.computeControlInput()
        self.setControlInput()
    
    def setTrajectory(self, desiredPosition, desiredVelocity, desiredAcceleration):
        return super().setTrajectory(desiredPosition, desiredVelocity, desiredAcceleration)

    # override
    def updateState(self):
        self.position = self.robot.getQ()
        self.velocity = self.robot.getQD()        
        self.updateMassMatrix()
        self.updateGravityTerm()
        self.updateCoriolisMatrix()
    
    # override
    def updateMassMatrix(self):
        self.massMatrix = np.array([[(3+2*math.cos(self.position[1]))*self.robot.getMass()*self.robot.getWireLength()**2, (1+math.cos(self.position[1]))*self.robot.getMass()*self.robot.getWireLength()**2],
                                    [(1+math.cos(self.position[1]))*self.robot.getMass()*self.robot.getWireLength()**2, self.robot.getMass()*self.robot.getWireLength()**2]])

    def updateCoriolisMatrix(self):
        self.coriolisMatrix = np.array([[0, -1*self.robot.getMass()*self.robot.getWireLength()**2*(2*self.velocity[0]+self.velocity[1])*math.sin(self.position[1])],
                                        [self.robot.getMass()*self.robot.getWireLength()**2*self.velocity[0]*math.sin(self.position[1]),0]])

    def updateGravityTerm(self):
        self.gravityTerm = self.robot.getMass() * -9.81 * self.robot.getWireLength() * np.array([2*math.sin(self.position[0]) + math.sin(self.position[0]+self.position[1]), math.sin(self.position[0]+self.position[1])])
    
    # override
    def computeControlInput(self):
        self.positionError = self.desiredPosition - self.position
        self.velocityError = self.desiredVelocity - self.velocity
        self.torque = self.massMatrix.dot(self.desiredAcceleration + self.PGain * self.positionError + self.DGain * self.velocityError) + self.coriolisMatrix.dot(self.desiredVelocity + self.PGain * self.positionError + self.DGain * self.velocityError) - self.gravityTerm
        print("torque : ", self.torque)
        

    # override
    def setControlInput(self):
        for i in range(1,self.n):
            if (self.torqueLimit[i] < self.torque[i]):
                self.torque[i] = self.torqueLimit[i]
            elif(-1*self.torqueLimit[i] > self.torque[i]):
                self.torque[i] = -1*self.torqueLimit[i]
        self.robot.setGeneralizedForce(self.torque)
        print(self.torque)       

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
    