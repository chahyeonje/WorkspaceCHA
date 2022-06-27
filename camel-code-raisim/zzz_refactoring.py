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

massMat = np.ones((2,2))
coriolisMat = np.array([0.0,0.0])
torque = np.zeros(2)

desiredPosition = np.array([0.0,0.0])
desiredVelocity = np.array([0.0,0.0])
desiredAcc = np.array([0.0,0.0])

q1_traj = ThirdOrderPolynomialTrajectory1D()
q1_traj.updateTrajectory(currentPosition=0,goalPosition=math.pi/2,currentTime=0,timeDuration=1)
q2_traj = ThirdOrderPolynomialTrajectory1D()
q2_traj.updateTrajectory(currentPosition=0.0,goalPosition=0.0,currentTime=0,timeDuration=1)

obj = world.addCylinder(0.2, 0.3, 9999999)
obj.setPosition(0.0, 0.4, 0.2)

s_traj = Sinusoidal()
s_traj.updateTrajectory(currentPosition=planarElbow.getGeneralizedCoordinate()[0] ,goalPosition=math.pi/2,currentTime=world.getWorldTime(),timeDuration=1)
residualVector = np.zeros(2)
desiredMomenta = np.zeros(2)
currentMomenta = np.zeros(2)
magnitudeResidualVector = np.linalg.norm(residualVector)
T_collsion = np.zeros(2)
s_collsion = 0
colMassMat = np.zeros((2,2))
diag_K = np.array([[38,0],[0,23]])


def Sd_t(time):
   return 0.2*math.sin(0.2*math.pi*time)

def Sd_t_prime(time):
   return 0.04*math.pi*math.cos(0.2*math.pi*time)

tttppp = np.zeros(2)
tttvvv = np.zeros(2)

time.sleep(2)
world.integrate()
while(True):
   desiredPosition = np.array([q1_traj.getPostionTrajectory(world.getWorldTime()),q2_traj.getPostionTrajectory(world.getWorldTime())])
   desiredVelocity = np.array([q1_traj.getVelocityTrajectory(world.getWorldTime()),q2_traj.getVelocityTrajectory(world.getWorldTime())])
   desiredAcc      = np.array([q1_traj.getAccelerationTrajectory(world.getWorldTime()),q2_traj.getAccelerationTrajectory(world.getWorldTime())])

   massMat         = np.array([[m1*l1**2 + m2*(l1**2+l2**2+2*l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1])),   m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))],
                               [m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1])),                      m2*l2**2]])
   
   coriolisMat     = np.array([-m2*l1*l2*math.sin(2*planarElbow.getGeneralizedVelocity()[0]*planarElbow.getGeneralizedVelocity()[1]+planarElbow.getGeneralizedVelocity()[1]**2), 
                                m2*l1*l2*(planarElbow.getGeneralizedVelocity()[0]**2)*math.sin(planarElbow.getGeneralizedCoordinate()[1])])

   desiredMomenta  = np.dot(massMat,desiredVelocity)

   temptorque = np.dot(massMat,desiredAcc + 10*(desiredPosition-planarElbow.getGeneralizedCoordinate()) + 20*(desiredVelocity-planarElbow.getGeneralizedVelocity())) + coriolisMat
   torque = np.array([temptorque[0],temptorque[1]])
   planarElbow.setGeneralizedForce(torque)
   currentMomenta = np.dot(massMat,planarElbow.getGeneralizedVelocity())
   residualVector = np.dot(diag_K,currentMomenta - desiredMomenta)
   magnitudeResidualVector = np.linalg.norm(residualVector)
   ###aftercolltion
   if(magnitudeResidualVector > 1):
      while(True):
         massMat         = np.array([[m1*l1**2 + m2*(l1**2+l2**2+2*l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1])),   m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))],
                                     [m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1])),                      m2*l2**2]])
   
         coriolisMat     = np.array([-m2*l1*l2*math.sin(2*planarElbow.getGeneralizedVelocity()[0]*planarElbow.getGeneralizedVelocity()[1]+planarElbow.getGeneralizedVelocity()[1]**2), 
                                      m2*l1*l2*(planarElbow.getGeneralizedVelocity()[0]**2)*math.sin(planarElbow.getGeneralizedCoordinate()[1])])

         desiredMomenta  = np.dot(massMat,desiredVelocity)
         
         T_collision = np.array([residualVector[1]/magnitudeResidualVector,-residualVector[0]/magnitudeResidualVector])
         s_collision = np.dot(T_collsion,planarElbow.getGeneralizedVelocity())
         
         colMassMat[0,0] = np.dot(massMat,T_collision)[0]
         colMassMat[1,0] = np.dot(massMat,T_collision)[1]
         colMassMat[0,1] = -residualVector[0]
         colMassMat[1,1] = -residualVector[1]


         us = Sd_t_prime(world.getWorldTime()) + 2*(Sd_t(world.getWorldTime()) - s_collision)
         uf = 0.1 + (0.01 - 1)*(0.01 - magnitudeResidualVector)
         
         temptorque = np.dot(colMassMat,[us,uf])+ coriolisMat
         torque = np.array([temptorque[0]+10*(math.pi/2-planarElbow.getGeneralizedCoordinate()[0])+2*(planarElbow.getGeneralizedVelocity()[1]),temptorque[1]])
         planarElbow.setGeneralizedForce(torque)

         currentMomenta = np.dot(massMat,planarElbow.getGeneralizedVelocity())
         residualVector = np.dot(diag_K,currentMomenta - desiredMomenta)
         magnitudeResidualVector = np.linalg.norm(residualVector)

         tttppp = planarElbow.getGeneralizedCoordinate()
         tttvvv = planarElbow.getGeneralizedVelocity()

         num1 = world.getWorldTime() * 1000
         int_a = int(num1)
         if(int_a%100==0):
            print("col=>","worldTime : ",world.getWorldTime(),"currneMomenta : ",currentMomenta," desiredMomenta : ",desiredMomenta ," residualVector : ",residualVector," magnitude : ",magnitudeResidualVector,"\n")

         world.integrate()
         delay(0.001)


   num1 = world.getWorldTime() * 1000
   int_a = int(num1)
   if(int_a%100==0):
      print("worldTime : ",world.getWorldTime(),"currneMomenta : ",currentMomenta," desiredMomenta : ",desiredMomenta ," residualVector : ",residualVector," magnitude : ",magnitudeResidualVector,"\n")

   world.integrate()
   delay(0.001)