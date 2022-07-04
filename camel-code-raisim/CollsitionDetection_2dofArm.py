import numpy as np
import math
class twoDofArm:
    def __init__(self, link1, link2, mass1, mass2, diag_k):#Get parameter of 2dof robot, and diagonal gain matrix 'K'
      self.link1=link1
      self.link2=link2
      self.mass1=mass1
      self.mass2=mass2
      self.diag_k = diag_k
      self.massMat=np.zeros((2,2))
      self.coriolisMat=np.zeros(2)
      self.colMassMat = np.zeros((2,2))

    def stateUpdate(self, angleOfJoint, velOfJoint, desiredVel):#Get generalized coordinate, velocity and desired velocity
      self.angleOfFisrtJoint = angleOfJoint[0]
      self.angleOfsecondJoint = angleOfJoint[1]
      self.velOfFirstJoint = velOfJoint[0]
      self.velOfSecondJoint = velOfJoint[1]
      self.velMat = np.array([self.velOfFirstJoint,self.velOfSecondJoint])
      self.desiredVel = desiredVel
      self.massMatrix
      self.coriolisMatrix

    def massMatrix(self):#Calculate mass matrix
      self.massMat[0,0] = self.mass1*self.link1**2 + self.mass2*(self.link1**2+self.link2**2+2*self.link1*self.link2*math.cos(self.angleOfsecondJoint))
      self.massMat[0,1] = self.mass2*(self.link2**2+self.link1*self.link2*math.cos(self.angleOfsecondJoint))
      self.massMat[1,0] = self.mass2*(self.link2**2+self.link1*self.link2*math.cos(self.angleOfsecondJoint))
      self.massMat[1,1] = self.mass2*self.link2**2
      return self.massMat

    def coriolisMatrix(self):#Calculate coriolis matrix
      self.coriolisMat[0] = -self.mass2*self.link1*self.link2*math.sin(2*self.angleOfFisrtJoint*self.angleOfsecondJoint+self.angleOfsecondJoint**2)
      self.coriolisMat[1] = self.mass2*self.link1*self.link2*(self.angleOfFisrtJoint**2)*math.sin(self.angleOfsecondJoint)
      return self.coriolisMat

###collsision parameter###
    def currentMomenta(self):
      self.currentm = np.dot(self.massMatrix(), self.velMat)
      return self.currentm

    def desiredMomenta(self):
      self.desiredm = np.dot(self.massMatrix(), self.desiredVel)
      return self.desiredm

    def residualVector(self):
      self.momentError = self.currentMomenta() - self.desiredMomenta()
      self.residual = np.dot(self.diag_k,self.momentError)
      return self.residual

    def magnitudeResidualVector(self):
      return np.linalg.norm(self.residualVector())


###for hybrid/motion control("Sensorless robot collision detection ~", A.D.Luca), need to be corrected.
    def T_collision(self):
      self.t_collision = np.array([self.residualVector()[1]/self.magnitudeResidualVector(),-self.residualVector()[0]/self.magnitudeResidualVector()])
      return self.t_collision

    def s_collision(self):
      return np.dot(self.T_collision(),self.velMat)

    def colMassMatrix(self):
      self.colMassMat[0,0] = np.dot(self.massMatrix(),self.T_collision())[0]
      self.colMassMat[0,1] = -self.residualVector()[0]
      self.colMassMat[1,0] = np.dot(self.massMatrix(),self.T_collision())[1]
      self.colMassMat[1,1] = -self.residualVector()[1]
      return self.colMassMat