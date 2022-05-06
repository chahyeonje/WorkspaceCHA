import math
import os
#from this import d
#from this import d
#from typing_extensions import Self
import numpy as np
import sys

sys.path.append('C:/Users/camelcha/raisimLib/install/bin') # path to the raisimpy
import raisimpy as raisim
import time
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectory1D
raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
### basic lines for initializing raisim

def delay(delayTime):   # should be changed later.
      startTime = time.time_ns()
      while(True):
         currentTime = time.time_ns()
         if((currentTime - startTime)*1e-9 > delayTime):
            break

# make raisim world
world = raisim.World()
world.setTimeStep(0.001)
server = raisim.RaisimServer(world)
ground = world.addGround()

# load robot.urdf file
planarElbow_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/rsc/test_planar_elbow_2dof.urdf"
planarElbow = world.addArticulatedSystem(planarElbow_urdf_file)   # robot class

planarElbow.setName("planarElbow")
planarElbow.setGeneralizedCoordinate(np.array([0, 0]))

# lauch server
server.launchServer(8080)

#link parameter
t=0.0
i=0
m1=0.193
m2=0.073
l1=0.1492
l2=0.381
I1=0.0015
I2=0.0001949
lc1=0.1032
lc2=0.084

q1=planarElbow.getGeneralizedCoordinate()[0]
q2=planarElbow.getGeneralizedCoordinate()[1]
dq1=planarElbow.getGeneralizedVelocity()[0]
dq2=planarElbow.getGeneralizedVelocity()[1]

massMat = np.ones((2,2))
massMat[0,0] = m1*lc1**2 + m2*(l2**2+lc2**2+2*l1*l2*math.cos(q2))+I1+I2
massMat[0,1] = m2*(lc2**2+l1*l2*math.cos(q2))+I2
massMat[1,0] = m2*(lc2**2+l1*l2*math.cos(q2))+I2
massMat[1,1] = m2*lc2**2+I2

coriolisMat = np.ones((2,2))
h = -m2*l1*l2*math.sin(q2)
coriolisMat[0,0] = h*dq2
coriolisMat[0,1] = h*dq2+h*dq1
coriolisMat[1,0] = -h*dq1
coriolisMat[1,1] = 0

torque = np.zeros(2)

desiredPosition = np.array([0.0,0.0])
desiredVelocity = np.array([0.0,0.0])
desiredAcc = np.array([0.0,0.0])

q1_traj = ThirdOrderPolynomialTrajectory1D()
q1_traj.updateTrajectory(currentPosition=0,goalPosition=math.pi/2,currentTime=0,timeDuration=1)
q2_traj = ThirdOrderPolynomialTrajectory1D()
q2_traj.updateTrajectory(currentPosition=0,goalPosition=0,currentTime=0,timeDuration=1)



obj = world.addCylinder(0.2, 0.3, 9999999)
obj.setPosition(0.0, 0.4, 0.2)

# kp=100
# kd=10
time.sleep(2)
world.integrate1()
while(True):
   delay(0.001)

   q1=planarElbow.getGeneralizedCoordinate()[0]
   q2=planarElbow.getGeneralizedCoordinate()[1]
   q=np.zeros((2,1))
   q[0]=q1
   q[1]=q2

   dq=np.zeros((2,1))
   dq[0]=planarElbow.getGeneralizedVelocity()[0]
   dq[1]=planarElbow.getGeneralizedVelocity()[1]

   desiredPosition[0] = q1_traj.getPostionTrajectory(world.getWorldTime())
   desiredPosition[1] = q2_traj.getPostionTrajectory(world.getWorldTime())
   desiredVelocity[0] = q1_traj.getVelocityTrajectory(world.getWorldTime())
   desiredVelocity[1] = q2_traj.getVelocityTrajectory(world.getWorldTime())
   desiredAcc[0] = q1_traj.getAccelerationTrajectory(world.getWorldTime())
   desiredAcc[1] = q2_traj.getAccelerationTrajectory(world.getWorldTime())

   if world.getWorldTime() > 1:
      desiredPosition[0] = math.pi/2
      desiredPosition[1] = 0
      desiredVelocity = np.array([0.0,0.0])

   massMat[0,0] = m1*lc1**2 + m2*(l2**2+lc2**2+2*l1*l2*math.cos(desiredPosition[1]))+I1+I2
   massMat[0,1] = m2*(lc2**2+l1*l2*math.cos(desiredPosition[1]))+I2
   massMat[1,0] = m2*(lc2**2+l1*l2*math.cos(desiredPosition[1]))+I2
   massMat[1,1] = m2*lc2**2+I2

   h = -m2*l1*l2*math.sin(desiredPosition[1])
   coriolisMat[0,0] = h*desiredVelocity[1]
   coriolisMat[0,1] = h*desiredVelocity[1]+h*desiredVelocity[0]
   coriolisMat[1,0] = -h*desiredVelocity[0]
   coriolisMat[1,1] = 0

   temptorque = np.dot(massMat,desiredAcc) + np.dot(coriolisMat,desiredVelocity)
   planarElbow.setGeneralizedForce(temptorque)
   world.integrate()