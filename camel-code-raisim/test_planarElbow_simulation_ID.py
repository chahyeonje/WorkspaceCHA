from dataclasses import asdict
from turtle import fd
import pathOS
import math
import os
import numpy as np
import sys
sys.path.append(pathOS.pathos) # path to the raisimpy
import raisimpy as raisim
import time
from CAMELTrajectoryGenerator import Sinusoidal, ThirdOrderPolynomialTrajectory1D
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
massMat[0,0] = m1*lc1**2 + m2*(l1**2+l2**2+2*l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))
massMat[0,1] = m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))
massMat[1,0] = m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))
massMat[1,1] = m2*l2**2

coriolisMat = np.array([0.0,0.0])
h = -m2*l1*l2*math.sin(q2)
coriolisMat[0] = -m2*l1*l2*math.sin(dq2)*(2*dq1*dq2+dq2**2)
coriolisMat[1] = m2*l1*l2*math.sin(q2)*dq1**2

torque = np.zeros(2)

desiredPosition = np.array([0.0,0.0])
desiredVelocity = np.array([0.0,0.0])
desiredAcc = np.array([0.0,0.0])

q1_traj = ThirdOrderPolynomialTrajectory1D()
q1_traj.updateTrajectory(currentPosition=0,goalPosition=math.pi/2,currentTime=0,timeDuration=1)
q2_traj = ThirdOrderPolynomialTrajectory1D()
q2_traj.updateTrajectory(currentPosition=0.0,goalPosition=0.0,currentTime=0,timeDuration=1)

###aftercolltion
s_traj = Sinusoidal()
s_traj.updateTrajectory(currentPosition=q1 ,goalPosition=math.pi/2,currentTime=0,timeDuration=1)
residualVector = np.zeros(2)
desiredMomenta = np.zeros(2)
currentMomenta = np.zeros(2)
diag_K = np.array([[50,0],[0,50]])
T_collsion = np.zeros(2)
s_collsion = 0
colMassMat = np.zeros((2,2))
###


q=np.zeros(2)
dq=np.zeros(2)
obj = world.addCylinder(0.2, 0.3, 9999999)
obj.setPosition(0.0, 0.4, 0.2)



time.sleep(2)
world.integrate()
while(True):
   delay(0.001)
   
   q=np.array([planarElbow.getGeneralizedCoordinate()[0],planarElbow.getGeneralizedCoordinate()[1]])
   dq=np.array([planarElbow.getGeneralizedVelocity()[0],planarElbow.getGeneralizedVelocity()[1]])

   desiredPosition[0] = q1_traj.getPostionTrajectory(world.getWorldTime())
   desiredPosition[1] = q2_traj.getPostionTrajectory(world.getWorldTime())
   desiredVelocity[0] = q1_traj.getVelocityTrajectory(world.getWorldTime())
   desiredVelocity[1] = q2_traj.getVelocityTrajectory(world.getWorldTime())
   desiredAcc[0] = q1_traj.getAccelerationTrajectory(world.getWorldTime())
   desiredAcc[1] = q2_traj.getAccelerationTrajectory(world.getWorldTime())

   if world.getWorldTime() > 1:
      #desiredPosition = -desiredPosition
      desiredVelocity = -desiredVelocity
      desiredAcc = -desiredAcc

   massMat[0,0] = m1*lc1**2 + m2*(l1**2+l2**2+2*l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))
   massMat[0,1] = m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))
   massMat[1,0] = m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))
   massMat[1,1] = m2*l2**2

   h = -m2*l1*l2*math.sin(planarElbow.getGeneralizedCoordinate()[1])
   coriolisMat[0] = -m2*l1*l2*math.sin(2*dq1*dq2+dq2**2)
   coriolisMat[1] = m2*l1*l2*(dq1**2)*math.sin(q2)

   desiredMomenta = np.dot(massMat,desiredVelocity)
   

###after collsion
   magnitudeResidualVector = np.linalg.norm(residualVector)
   T_collision = np.array([residualVector[1]/magnitudeResidualVector,-residualVector[0]/magnitudeResidualVector])
   s_collision = np.dot(T_collsion,planarElbow.getGeneralizedVelocity())
   us = s_traj.getAccelerationTrajectory(world.getWorldTime()) + 2*(s_traj.getVelocityTrajectory(world.getWorldTime()) - s_collision)
   uf = 0.1 + (0.01 - 1)*(0.01 - magnitudeResidualVector)
   colMassMat[0,0] = np.dot(massMat,T_collision)[0]
   colMassMat[1,0] = np.dot(massMat,T_collision)[1]
   colMassMat[0,1] = -residualVector[0]
   colMassMat[1,1] = -residualVector[1]
###

   temptorque = np.dot(massMat,desiredAcc + 10*(desiredPosition-planarElbow.getGeneralizedCoordinate()) + 20*(desiredVelocity-planarElbow.getGeneralizedVelocity())) + coriolisMat
   torque[0] = temptorque[0]*15.0
   torque[1] = temptorque[1]*13.0

   if(magnitudeResidualVector > 2):
      while(True):
         delay(0.001)

         massMat[0,0] = m1*lc1**2 + m2*(l1**2+l2**2+2*l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))
         massMat[0,1] = m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))
         massMat[1,0] = m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))
         massMat[1,1] = m2*l2**2

         h = -m2*l1*l2*math.sin(planarElbow.getGeneralizedCoordinate()[1])
         coriolisMat[0] = -m2*l1*l2*math.sin(2*dq1*dq2+dq2**2)
         coriolisMat[1] = m2*l1*l2*(dq1**2)*math.sin(q2)

         desiredMomenta = np.dot(massMat,desiredVelocity)
         currentMomenta = np.dot(massMat,planarElbow.getGeneralizedVelocity())
         residualVector = np.dot(diag_K,currentMomenta - desiredMomenta)

         magnitudeResidualVector = np.linalg.norm(residualVector)
         T_collision = np.array([residualVector[1]/magnitudeResidualVector,-residualVector[0]/magnitudeResidualVector])
         s_collision = np.dot(T_collsion,planarElbow.getGeneralizedVelocity())
         us = s_traj.getAccelerationTrajectory(world.getWorldTime()) + 2*(s_traj.getVelocityTrajectory(world.getWorldTime()) - s_collision)
         uf = 0.1 + (0.01 - 1)*(0.01 - magnitudeResidualVector)
         colMassMat[0,0] = np.dot(massMat,T_collision)[0]
         colMassMat[1,0] = np.dot(massMat,T_collision)[1]
         colMassMat[0,1] = -residualVector[0]
         colMassMat[1,1] = -residualVector[1]

         temptorque = np.dot(colMassMat,[us,uf])
         torque[0] = temptorque[0]
         torque[1] = 0.74*temptorque[1]

         # position = planarElbow.getGeneralizedCoordinate()
         # velocity = planarElbow.getGeneralizedVelocity()
         # desiredPosition[0] = math.pi/2#q1_traj.getPostionTrajectory(world.getWorldTime())
         # desiredPosition[1] = 0 #q2_traj.getPostionTrajectory(world.getWorldTime())
         # desiredVelocity[0] = 0 #q1_traj.getVelocityTrajectory(world.getWorldTime())
         # desiredVelocity[1] = 0 #q2_traj.getVelocityTrajectory(world.getWorldTime())
         # tempTorque = 100 * (desiredPosition - position) + 10 * (desiredVelocity - velocity)
         # torque[0] = tempTorque[0]
         # torque[1] = 0.1*tempTorque[1]


         planarElbow.setGeneralizedForce(torque)

         num1 = world.getWorldTime() * 1000
         int_a = int(num1)
         if(int_a%100==0):
            print("col=>","currneMomenta : ",currentMomenta," desiredMomenta : ",desiredMomenta ," residualVector : ",residualVector," magnitude : ",magnitudeResidualVector,"\n")

         world.integrate()

   planarElbow.setGeneralizedForce(torque)
   currentMomenta = np.dot(massMat,planarElbow.getGeneralizedVelocity())
   residualVector = np.dot(diag_K,currentMomenta - desiredMomenta)

   num1 = world.getWorldTime() * 100
   int_a = int(num1)
   if(int_a%10==0):
      #print(temptorque[0]," ",temptorque[1],"\n")   
      #print(residualVector, "\n")
      print("currneMomenta : ",currentMomenta," desiredMomenta : ",desiredMomenta ," residualVector : ",residualVector," magnitude : ",magnitudeResidualVector,"\n")

   world.integrate()
