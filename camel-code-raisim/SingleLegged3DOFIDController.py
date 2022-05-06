import math
import random
from CAMELController import InverseDynamicsController
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectory1D
import numpy as np

"""
3dof single legged robot inverse dynamics controller
    : Modeled in point mass. 
        First, find Force by inverse dynamics of model
        (force = mass*(desired_zdd +Kp*positionError + Kd*velocityError) - mass*gravity)
        Second, find torque by Jacobian * Force
        (Jacobian is from kinematic constraints)

Problems(Solved!)
    : compared to full body dynamics equation (not modeled one.)
      I needed to adjust mass value(in Robot class) in order to make proper initial force

=> it is solved by get full Jacobian.


TODO : In current system, ankle joint has an independent PD controller.
       Because I think foot needs planar contact region with ground.
       Ask S.S.Dr.Lee about control law of foot.
"""

class SingleLegged3DOFIDController(InverseDynamicsController):
    def __init__(self, robot):
        super().__init__(robot)
        self.iteration = 0
        self.gravity = -9.81
        self.dof = 4                                    # 1 passive joint is in index - 0 (base prismatic joint)
        self.force = 0
        self.torque = np.zeros(self.dof)          
        self.setTorqueLimit(50.0)
        self.positionError = 0
        self.differentialError = 0
        self.trajectoryGenerator = ThirdOrderPolynomialTrajectory1D()
        self.goalPosition = 0.61
        self.trajectoryGenerator.updateTrajectory(currentPosition=self.robot.getHipPositionZ(), goalPosition= self.goalPosition, currentTime= self.robot.getTime(), timeDuration=2.0)
    
    def setPDGain(self, PGain, DGain):
        return super().setPDGain(PGain, DGain)
    
    # override
    def doControl(self):
        self.iteration += 1
        self.checkUpdateTrajectory()
        self.updateState()
        self.setTrajectory(desiredPosition=self.trajectoryGenerator.getPostionTrajectory(self.robot.getTime()), desiredVelocity=self.trajectoryGenerator.getVelocityTrajectory(self.robot.getTime()), desiredAcceleration=self.trajectoryGenerator.getAccelerationTrajectory(self.robot.getTime()))
        self.computeControlInput()
        self.setControlInput()

    def checkUpdateTrajectory(self):
        if((self.iteration % 400) == 0):
            self.currentPosition = self.goalPosition
            self.goalPosition = (random.random()-0.5)*0.2 + 0.42426
            self.trajectoryGenerator.updateTrajectory(currentPosition=self.currentPosition, goalPosition=self.goalPosition, currentTime= self.robot.getTime(), timeDuration=2.0)
    
    # override
    def setTrajectory(self, desiredPosition, desiredVelocity, desiredAcceleration):
        return super().setTrajectory(desiredPosition, desiredVelocity, desiredAcceleration)
    
    # override
    def updateState(self):
        self.position = self.robot.getHipPositionZ()
        self.velocity = self.robot.getHipVelocityZ()
        self.theta1 = self.robot.getHipAngularPosition()
        self.theta2 = self.robot.getKneeAngularPosition()
        self.theta3 = self.robot.getAnkleAngularPosition()
        self.updateMassMatrix()
        self.updateJacobian()
        self.updateGravityTerm()

    # override
    def updateMassMatrix(self):
        self.massMatrix = self.robot.mass
    
    def updateGravityTerm(self):
        self.gravityTerm = self.robot.getMass() * self.gravity

    def updateJacobian(self):
        self.dz_dth1 = -self.robot.getUpperLegLength()*math.sin(self.theta1) -self.robot.getLowerLegLength()*math.sin(self.theta1+self.theta2) -self.robot.getFootLength()*math.sin(self.theta1+self.theta2+self.theta3)
        self.dz_dth2 = -self.robot.getLowerLegLength()*math.sin(self.theta1+self.theta2)-self.robot.getFootLength()*math.sin(self.theta1+self.theta2+self.theta3)
        self.dz_dth3 = -self.robot.getFootLength()*math.sin(self.theta1+self.theta2+self.theta3)


    def computeForceToTorque(self):
        self.torque[0] = 0.0
        self.torque[1] = self.dz_dth1 * self.force
        self.torque[2] = self.dz_dth2 * self.force
        self.torque[3] = self.dz_dth3 * self.force

    def computeControlInput(self):
        self.positionError = self.desiredPosition - self.position
        self.velocityError = self.desiredVelocity - self.velocity
        self.force = self.massMatrix * (self.desiredAcceleration + self.PGain * self.positionError + self.DGain * self.velocityError) - self.gravityTerm
        self.computeForceToTorque()

    # override
    def setControlInput(self):
        for i in range(self.dof):
            if (self.torqueLimit < self.torque[i]):
                self.torque[i] = self.torqueLimit
            elif(-self.torqueLimit > self.torque[i]):
                self.torque[i] = -self.torqueLimit    
        self.robot.setGeneralizedForce(self.torque)
        print("force", self.force)
        print("torque", self.torque)

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
    