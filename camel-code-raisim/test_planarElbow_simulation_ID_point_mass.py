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
from CollsitionDetection_2dofArm import twoDofArm
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
planarElbow_parameter = twoDofArm(0.1492,0.381,0.193,0.073,[[38,0],[0,23]])
#
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

def Sd_t(time):
   return 0.2*math.sin(0.2*math.pi*time)
def Sd_t_prime(time):
   return 0.04*math.pi*math.cos(0.2*math.pi*time)

time.sleep(2)
world.integrate()
while(True):
   desiredPosition = np.array([q1_traj.getPostionTrajectory(world.getWorldTime()),q2_traj.getPostionTrajectory(world.getWorldTime())])
   desiredVelocity = np.array([q1_traj.getVelocityTrajectory(world.getWorldTime()),q2_traj.getVelocityTrajectory(world.getWorldTime())])
   desiredAcc      = np.array([q1_traj.getAccelerationTrajectory(world.getWorldTime()),q2_traj.getAccelerationTrajectory(world.getWorldTime())])
   planarElbow_parameter.stateUpdate(planarElbow.getGeneralizedCoordinate(),planarElbow.getGeneralizedVelocity(),desiredVelocity)
   ###aftercolltion
   if(planarElbow_parameter.magnitudeResidualVector() > 1):
      while(True):
         us = Sd_t_prime(world.getWorldTime()) + 2*(Sd_t(world.getWorldTime()) - planarElbow_parameter.s_collision())
         uf = 0.1 + (0.01 - 1)*(0.01 - planarElbow_parameter.magnitudeResidualVector())

         temptorque = np.dot(planarElbow_parameter.colMassMatrix(),[us,uf]) + planarElbow_parameter.coriolisMatrix()
         torque = np.array([temptorque[0]+10*(math.pi/2-planarElbow.getGeneralizedCoordinate()[0])+2*(planarElbow.getGeneralizedVelocity()[1]),temptorque[1]])
         planarElbow.setGeneralizedForce(torque)

         num1 = world.getWorldTime() * 1000
         int_a = int(num1)
         if(int_a%100==0):
            print("col=>","worldTime : ",world.getWorldTime(),"currneMomenta : ",planarElbow_parameter.currentMomenta()," desiredMomenta : ",planarElbow_parameter.desiredMomenta() ," residualVector : ",planarElbow_parameter.residualVector()," magnitude : ",planarElbow_parameter.magnitudeResidualVector(),"\n")

         world.integrate()
         planarElbow_parameter.stateUpdate(planarElbow.getGeneralizedCoordinate(),planarElbow.getGeneralizedVelocity(),desiredVelocity)
         delay(0.001)
         
   temptorque = np.dot(planarElbow_parameter.massMatrix(),desiredAcc + 10*(desiredPosition-planarElbow.getGeneralizedCoordinate()) + 20*(desiredVelocity-planarElbow.getGeneralizedVelocity())) + planarElbow_parameter.coriolisMatrix()
   torque = np.array([temptorque[0],temptorque[1]])
   planarElbow.setGeneralizedForce(torque)
   num1 = world.getWorldTime() * 1000
   int_a = int(num1)
   if(int_a%100==0):
      print("worldTime : ",world.getWorldTime(),"currneMomenta : ",planarElbow_parameter.currentMomenta()," desiredMomenta : ",planarElbow_parameter.desiredMomenta() ," residualVector : ",planarElbow_parameter.residualVector()," magnitude : ",planarElbow_parameter.magnitudeResidualVector(),"\n")
   world.integrate()
   delay(0.001)