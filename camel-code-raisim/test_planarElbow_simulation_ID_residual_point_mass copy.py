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
planarElbow_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/rsc/test_planar_elbow_2dof_point_mass.urdf"
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
#

q1=planarElbow.getGeneralizedCoordinate()[0]
q2=planarElbow.getGeneralizedCoordinate()[1]
dq1=planarElbow.getGeneralizedVelocity()[0]
dq2=planarElbow.getGeneralizedVelocity()[1]

massMat = np.ones((2,2))
coriolisMat = np.array([0.0,0.0])
torque = np.zeros(2)

desiredPosition = np.array([0.0,0.0])
desiredVelocity = np.array([0.0,0.0])
desiredAcc = np.array([0.0,0.0])

q1_traj = ThirdOrderPolynomialTrajectory1D()
q1_traj.updateTrajectory(currentPosition=0,goalPosition=math.pi/2,currentTime=0,timeDuration=3)
q2_traj = ThirdOrderPolynomialTrajectory1D()
q2_traj.updateTrajectory(currentPosition=0.0,goalPosition=0.0,currentTime=0,timeDuration=3)




q=np.zeros(2)
dq=np.zeros(2)
obj = world.addCylinder(0.2, 0.3, 9999999)
obj.setPosition(0.0, 0.4, 0.2)

s_traj = Sinusoidal()
s_traj.updateTrajectory(currentPosition=planarElbow.getGeneralizedCoordinate()[0] ,goalPosition=math.pi/2,currentTime=world.getWorldTime(),timeDuration=3)
residualVector = np.zeros(2)
desiredMomenta = np.zeros(2)
currentMomenta = np.zeros(2)
magnitudeResidualVector = np.linalg.norm(residualVector)
diag_K = np.array([[47,0],[0,36.5]])
T_collsion = np.zeros(2)
s_collsion = 0
colMassMat = np.zeros((2,2))
alpha = np.zeros(2)
in_integral = np.zeros(2)
time.sleep(2)
world.integrate()
while(True):
   delay(0.001)
   
   q               = np.array([planarElbow.getGeneralizedCoordinate()[0],planarElbow.getGeneralizedCoordinate()[1]])
   dq              = np.array([planarElbow.getGeneralizedVelocity()[0],planarElbow.getGeneralizedVelocity()[1]])

   desiredPosition = np.array([q1_traj.getPostionTrajectory(world.getWorldTime()),q2_traj.getPostionTrajectory(world.getWorldTime())])
   desiredVelocity = np.array([q1_traj.getVelocityTrajectory(world.getWorldTime()),q2_traj.getVelocityTrajectory(world.getWorldTime())])
   desiredAcc      = np.array([q1_traj.getAccelerationTrajectory(world.getWorldTime()),q2_traj.getAccelerationTrajectory(world.getWorldTime())])

   massMat         = np.array([[m1*l1**2 + m2*(l1**2+l2**2+2*l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1])),   m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))],
                               [m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1])),                      m2*l2**2]])
   
   
   coriolisMat     = np.array([-m2*l1*l2*math.sin(planarElbow.getGeneralizedCoordinate()[1])*(2*planarElbow.getGeneralizedVelocity()[0]*planarElbow.getGeneralizedVelocity()[1]+planarElbow.getGeneralizedVelocity()[1]**2), 
                                m2*l1*l2*(planarElbow.getGeneralizedVelocity()[0]**2)*math.sin(planarElbow.getGeneralizedCoordinate()[1])])

   currentMomenta = np.dot(massMat,planarElbow.getGeneralizedVelocity())
   
   residualVector = np.dot(diag_K,in_integral)
   

   desiredMomenta  = np.dot(massMat,desiredVelocity)


   temptorque = np.dot(massMat,desiredAcc) + coriolisMat
   torque[0] = temptorque[0]
   torque[1] = temptorque[1]
   residualVector = np.dot(diag_K,currentMomenta - desiredMomenta)
   magnitudeResidualVector = np.linalg.norm(residualVector)
   # alpha[0] = 0
   # bbbb = np.dot([[-2*m2*l1*l2*math.sin(planarElbow.getGeneralizedCoordinate()[1]),-m2*l1*l2*math.sin(planarElbow.getGeneralizedCoordinate()[1])],[-m2*l1*l2*math.sin(planarElbow.getGeneralizedCoordinate()[1]),0]],planarElbow.getGeneralizedVelocity())
   # alpha[1] = -(1/2)*np.dot(planarElbow.getGeneralizedVelocity(),bbbb)
   # in_integral = currentMomenta - (torque - alpha + residualVector)

   planarElbow.setGeneralizedForce(torque)
##### calculated of residualvector
   
   
   
#####
   

   ###aftercolltion
   # if(magnitudeResidualVector > 2):
   #    s_traj.updateTrajectory(currentPosition=planarElbow.getGeneralizedCoordinate()[0] ,goalPosition=math.pi/2,currentTime=world.getWorldTime(),timeDuration=1)
   #    while(True):
         

   #       massMat         = np.array([[m1*l1**2 + m2*(l1**2+l2**2+2*l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1])),   m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))],
   #                                   [m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1])),                      m2*l2**2]])
   
   #       coriolisMat     = np.array([-m2*l1*l2*math.sin(2*planarElbow.getGeneralizedVelocity()[0]*planarElbow.getGeneralizedVelocity()[1]+planarElbow.getGeneralizedVelocity()[1]**2), 
   #                                    m2*l1*l2*(planarElbow.getGeneralizedVelocity()[0]**2)*math.sin(planarElbow.getGeneralizedCoordinate()[1])])

   #       # desiredPosition = np.array([s_traj.getPostionTrajectory(world.getWorldTime()),q2_traj.getPostionTrajectory(world.getWorldTime())])
   #       # desiredVelocity = np.array([s_traj.getVelocityTrajectory(world.getWorldTime()),q2_traj.getVelocityTrajectory(world.getWorldTime())])
   #       desiredMomenta  = np.dot(massMat,desiredVelocity)
         
   #       T_collision = np.array([residualVector[1]/magnitudeResidualVector,-residualVector[0]/magnitudeResidualVector])
   #       s_collision = np.dot(T_collsion,planarElbow.getGeneralizedVelocity())
   #       us = s_traj.getAccelerationTrajectory(world.getWorldTime()) + 2*(s_traj.getVelocityTrajectory(world.getWorldTime()) - s_collision)
   #       uf = 0.1 + (0.01 - 1)*(0.01 - magnitudeResidualVector)
   #       colMassMat[0,0] = np.dot(massMat,T_collision)[0]
   #       colMassMat[1,0] = np.dot(massMat,T_collision)[1]
   #       colMassMat[0,1] = -residualVector[0]
   #       colMassMat[1,1] = -residualVector[1]

   #       temptorque = np.dot(colMassMat,[us,uf])+ coriolisMat
   #       torque = np.array([temptorque[0],temptorque[1]])
   #       planarElbow.setGeneralizedForce(torque)

   #       currentMomenta = np.dot(massMat,planarElbow.getGeneralizedVelocity())
   #       residualVector = np.dot(diag_K,currentMomenta - desiredMomenta)
   #       magnitudeResidualVector = np.linalg.norm(residualVector)

   #       num1 = world.getWorldTime() * 1000
   #       int_a = int(num1)
   #       if(int_a%100==0):
   #          print("col=>","currneMomenta : ",currentMomenta," desiredMomenta : ",desiredMomenta ," residualVector : ",residualVector," magnitude : ",magnitudeResidualVector,"\n")

   #       world.integrate()
   #       delay(0.001)


   num1 = world.getWorldTime() * 100
   int_a = int(num1)
   if(int_a%10==0):
      print("currneMomenta : ",currentMomenta," desiredMomenta : ",desiredMomenta ," residualVector : ",residualVector," magnitude : ",magnitudeResidualVector,"torque[0]:",torque[0],"torque[1]:",torque[1],"cos :",math.cos(planarElbow.getGeneralizedCoordinate()[1]),"\n")

   world.integrate()
