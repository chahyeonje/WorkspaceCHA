import pathOS
import math
import os
import numpy as np
import sys
sys.path.append(pathOS.pathos) # path to the raisimpy
import raisimpy as raisim
import time
raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
from CAMELTrajectoryGenerator import Sinusoidal, ThirdOrderPolynomialTrajectory1D
### basic lines for initializing raisim

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



torque = np.zeros(2)
Kp = 10.0
Kd = 1.0

desiredPosition = np.array([0.0,0.0])
desiredVelocity = np.array([0.0,0.0])
q1_traj = ThirdOrderPolynomialTrajectory1D()
q1_traj.updateTrajectory(currentPosition=0,goalPosition=math.pi/2,currentTime=0,timeDuration=1)
q2_traj = ThirdOrderPolynomialTrajectory1D()
q2_traj.updateTrajectory(currentPosition=0,goalPosition=0,currentTime=0,timeDuration=1)
###

#link parameter
m1=0.193
m2=0.073
l1=0.1492
l2=0.381
#
diag_K = np.array([[38,0],[0,22]])
magnitudeResidualVector = 0
time_fix = 0
# obj = world.addCylinder(0.2, 0.3, 9999)
# obj.setPosition(0.0, 0.4, 0.2)

time.sleep(2)
#integrate() = integrate1() + integrate2()
world.integrate1()

def delay(delayTime):   # should be changed later.
      startTime = time.time_ns()
      while(True):
         currentTime = time.time_ns()
         if((currentTime - startTime)*1e-9 > delayTime):
            break
i=0

while(True):
   delay(0.001)

   position = planarElbow.getGeneralizedCoordinate()
   velocity = planarElbow.getGeneralizedVelocity()

   if world.getWorldTime() >= 1:
      desiredPosition = np.array([math.pi/2,0])
      desiredVelocity = np.array([q1_traj.getVelocityTrajectory(0.999),q2_traj.getVelocityTrajectory(0.999)])
   
   else :
      desiredPosition = np.array([q1_traj.getPostionTrajectory(world.getWorldTime()),q2_traj.getPostionTrajectory(world.getWorldTime())])
      desiredVelocity = np.array([q1_traj.getVelocityTrajectory(world.getWorldTime()),q2_traj.getVelocityTrajectory(world.getWorldTime())])
   

   massMat         = np.array([[m1*l1**2 + m2*(l1**2+l2**2+2*l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1])),   m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1]))],
                               [m2*(l2**2+l1*l2*math.cos(planarElbow.getGeneralizedCoordinate()[1])),                      m2*l2**2]])
   
   
   coriolisMat     = np.array([-m2*l1*l2*math.sin(2*planarElbow.getGeneralizedVelocity()[0]*planarElbow.getGeneralizedVelocity()[1]+planarElbow.getGeneralizedVelocity()[1]**2), 
                                m2*l1*l2*(planarElbow.getGeneralizedVelocity()[0]**2)*math.sin(planarElbow.getGeneralizedCoordinate()[1])])

   desiredMomenta  = np.dot(massMat,desiredVelocity)
   currentMomenta = np.dot(massMat,planarElbow.getGeneralizedVelocity())
   residualVector = np.dot(diag_K,currentMomenta - desiredMomenta)
   magnitudeResidualVector = np.linalg.norm(residualVector)

   tempTorque = Kp * (desiredPosition - position) + Kd * (desiredVelocity - velocity)
   torque = tempTorque
   planarElbow.setGeneralizedForce(torque)

   num1 = world.getWorldTime() * 1000
   int_a = int(num1)
   if(int_a%10==0):
      print("col=>","currneMomenta : ",currentMomenta," desiredMomenta : ",desiredMomenta ," residualVector : ",residualVector," magnitude : ",magnitudeResidualVector,"\n")

   world.integrate()