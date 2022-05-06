import SingleLeggedRobot
from CAMELRaisimLib import Simulation

urdfPath = "C:/Users/Jaehoon/raisimLib/rsc/camel/camel_single_leg_urdf.urdf"
name = 'cuteLeg'
dT = 0.005
sim = Simulation(dT)
robot = SingleLeggedRobot.SingleLeggedRobot(sim, urdfPath, name)



robot.initialize()
print(robot.getFrames())
print(robot.getBodyNames())
print(robot.getFrameIdx("hip_pitch"))
print(robot.getFrameIdx("knee_pitch"))
print(robot.getFrameOrientation("hip_pitch"))
print(robot.getFrameOrientation("knee_pitch"))
print(robot.getGeneralizedCoordinate())
print(robot.getHipQ())
print(robot.getKneeQ())

while(True):
    print("current time :", sim.getTime())
    sim.integrate()
    print(robot.getHipQ())    
    print(robot.getHipQD())
    print(robot.getKneeQ())
    print(robot.getKneeQD())

    
    
        


sim.killServer()
