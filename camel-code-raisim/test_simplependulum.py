import pathOS
import math
import os
import numpy as np
import sys
sys.path.append(pathOS.pathos) # path to the raisimpy
import raisimpy as raisim
import time
raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
from CAMELTrajectoryGenerator import ThirdOrderPolynomialTrajectory1D
# ### basic lines for initializing raisim
# from CAMELRaisimLib import Simulation
# make raisim world
world = raisim.World()
world.setTimeStep(0.001)
server = raisim.RaisimServer(world)
ground = world.addGround()

# load robot.urdf file
simplepedulum_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/rsc/camel_simple_pendulum.urdf"
simplepedulum = world.addArticulatedSystem(simplepedulum_urdf_file)   # robot class
simplepedulum.setName("simplepedulum")
simplepedulum.setGeneralizedCoordinate(np.array([0, 0]))

# lauch server
server.launchServer(8080)

q1_traj = ThirdOrderPolynomialTrajectory1D()
q1_traj.updateTrajectory(currentPosition=0,goalPosition=math.pi/2,currentTime=0,timeDuration=1)

torque = np.zeros(2)
Kp = 250.0
Kd = 25.0

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

   position = simplepedulum.getGeneralizedCoordinate()
   velocity = simplepedulum.getGeneralizedVelocity()

   desiredPosition = q1_traj.getPostionTrajectory(world.getWorldTime())
   desiredVelocity = q1_traj.getVelocityTrajectory(world.getWorldTime())

   if world.getWorldTime() > q1_traj.timeDuration:
      desiredPosition = math.pi/2
      desiredVelocity = 0

   ztime=world.getWorldTime()
   print(ztime)
   tempTorque = Kp * (desiredPosition - position) + Kd * (desiredVelocity - velocity)
   torque = tempTorque
   simplepedulum.setGeneralizedForce(torque)
   # print("position :", position)
   # print("velocity :", velocity)
   world.integrate()
   

