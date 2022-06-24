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
Kp = 100.0
Kd = 10.0

###
desiredPosition = np.array([0.0,0.0])
desiredVelocity = np.array([0.0,0.0])
q1_traj = ThirdOrderPolynomialTrajectory1D()
q1_traj.updateTrajectory(currentPosition=0,goalPosition=math.pi/2,currentTime=0,timeDuration=10)
q2_traj = ThirdOrderPolynomialTrajectory1D()
q2_traj.updateTrajectory(currentPosition=0,goalPosition=0,currentTime=0,timeDuration=10)
###

# obj = world.addCylinder(0.2, 0.3, 9999)
# obj.setPosition(0.0, 0.4, 0.2)
def delay(delayTime):   # should be changed later.
      startTime = time.time_ns()
      while(True):
         currentTime = time.time_ns()
         if((currentTime - startTime)*1e-9 > delayTime):
            break
          
time.sleep(2)
#integrate() = integrate1() + integrate2()
world.integrate()


i=0
while(True):
    delay(0.001)

    position = planarElbow.getGeneralizedCoordinate()
    velocity = planarElbow.getGeneralizedVelocity()
    desiredPosition[0] = q1_traj.getPostionTrajectory(world.getWorldTime())
    desiredPosition[1] = q2_traj.getPostionTrajectory(world.getWorldTime())
    desiredVelocity[0] = q1_traj.getVelocityTrajectory(world.getWorldTime())
    desiredVelocity[1] = q2_traj.getVelocityTrajectory(world.getWorldTime())
    # if world.getWorldTime() > 1:
    #   desiredPosition[0] = math.pi/2
    #   desiredPosition[1] = 0
    #   desiredVelocity = np.array([0.0,0.0])
    #   desiredAcc = np.array([0.0, 0.0])

    tempTorque = Kp * (desiredPosition - position) + Kd * (desiredVelocity - velocity)
    torque[0] = tempTorque[0]
    torque[1] = tempTorque[1]
    planarElbow.setGeneralizedForce(torque)
    world.integrate()

