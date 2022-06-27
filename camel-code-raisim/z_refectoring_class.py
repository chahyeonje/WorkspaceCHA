import numpy as np
import math
class twoDofArm:
    def __init__(self, num1, num2, num3, num4):
      self.link1=num1
      self.link2=num2
      self.mass1=num3
      self.mass2=num4
      
    def stateUpdate(self, angleOfJoint, velOfJoint, desiredVel):
      self.angleOfFisrtJoint = angleOfJoint[0]
      self.angleOfsecondJoint = angleOfJoint[1]
      self.velOfFirstJoint = velOfJoint[0]
      self.velOfSecondJoint = velOfJoint[1]
      self.velMat = np.array([self.velOfFirstJoint,self.velOfSecondJoint])
      self.desiredVel = desiredVel
      self.massMatrix
      self.coriolisMatrix

    def massMatrix(self):
      self.massMat=np.zeros((2,2))
      self.massMat[0,0] = self.mass1*self.link1**2 + self.mass2*(self.link1**2+self.link2**2+2*self.link1*self.link2*math.cos(self.angleOfsecondJoint))
      self.massMat[0,1] = self.mass2*(self.link2**2+self.link1*self.link2*math.cos(self.angleOfsecondJoint))
      self.massMat[1,0] = self.mass2*(self.link2**2+self.link1*self.link2*math.cos(self.angleOfsecondJoint))
      self.massMat[1,1] = self.mass2*self.link2**2
      return self.massMat

    def coriolisMatrix(self):
      self.coriolisMat=np.zeros(2)
      self.coriolisMat[0] = -self.mass2*self.link1*self.link2*math.sin(2*self.angleOfFisrtJoint*self.angleOfsecondJoint+self.angleOfsecondJoint**2)
      self.coriolisMat[1] = self.mass2*self.link1*self.link2*(self.angleOfFisrtJoint**2)*math.sin(self.angleOfsecondJoint)
      return self.coriolisMat
#collsision parameter
    def currentMomenta(self):
      self.currentm = np.dot(self.massMat, self.velMat)
      return self.currentm

    def desiredMomenta(self):
      self.desiredm = np.dot(self.massMat, self.desiredVel)
      return self.desiredm

    def residualVector(self):
      self.residual = self.currentm - self.desiredm
      return self.residual

    def magnitudeResidualVector(self):
      return np.linalg.norm(self.residual)
