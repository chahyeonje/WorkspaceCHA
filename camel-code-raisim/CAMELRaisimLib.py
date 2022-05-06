import sys
import time
import threading
from abc import abstractmethod

import pyqtgraph as pg

from PySide6.QtWidgets import *
from PySide6.QtCore import Slot

"""
!IMPORTANT!

Path of bin and license should be set by user. 

"""

binPath = 'C:/Users/camelcha/raisimLib/install/bin'
licensePath = "C:/Users/camelcha/.raisim/raisim.activation"
sys.path.append(binPath)
import raisimpy as raisim
raisim.World.setLicenseFile(licensePath)

class CAMELThread (threading.Thread):
   def __init__(self, sim):
      threading.Thread.__init__(self)
      self.sim = sim
      self.ui = UI()
      self.iteration = 0
      self.simDurationTime = 0

   def run(self):
      while(True):
         if(self.ui.getIsButtonPressed()):
            if(self.sim.getSimulationDuration() > self.simDurationTime):
               self.simDurationTime += self.sim.getDT()
               self.iteration += 1
               if(self.sim.isFastSimulation):
                  self.fastSimulation()
               else:
                  self.realTimeSimulation()
                  
               if(self.sim.isDataPlot):   
                  self.sim.getPlot().setData()
               
            else:
               if(self.sim.isDataPlot):
                  self.ui.plot(self.sim.getPlot().t,self.sim.getPlot().data1,self.sim.getPlot().t,self.sim.getPlot().data2)
                  
               
               print("simulation time : ", self.sim.getTime())
               self.ui.setIsButtonPressed(False)
               self.simDurationTime = 0
         
      
   def delay(self, delayTime):   # should be changed later.
      startTime = time.time_ns()
      while(True):
         currentTime = time.time_ns()
         if((currentTime - startTime)*1e-9 > delayTime):
            break

   def fastSimulation(self):
      # print("current time :", self.sim.getTime())
      self.sim.controller.doControl()
      self.sim.integrate()

   def realTimeSimulation(self):
      self.delay(self.sim.getDT() / 2)
      # print("current time :", self.sim.getTime())
      self.sim.controller.doControl()
      self.sim.integrate()


class Simulation:

    def __init__(self):
        self.world = raisim.World()
        self.server = raisim.RaisimServer(self.world)
        self.ground = self.world.addGround()

    def integrate(self):
        self.world.integrate()

    def integration1(self):
        self.world.integrate1()

    def integration2(self):
        self.world.integrate2()

    def initializeServer(self):
        self.server.launchServer(8080)

    def killServer(self):
        self.server.killServer()

    def setRobot(self, robot):
        self.robot = robot

    def setController(self, controller):
        self.controller = controller

    def setPlot(self, plot):
        self.plot = plot

    def setDT(self, dT):
        self.world.setTimeStep(dT)

    def setSimulationDuration(self, duration):
        self.simulationDuration = duration

    def setFastSimulation(self, booleanValue):
        self.isFastSimulation = booleanValue

    def setDataPlot(self, booleanValue):
        self.isDataPlot = booleanValue

    def getRobot(self):
        return self.robot

    def getController(self):
        return self.controller

    def getPlot(self):
        return self.plot

    def getDT(self):
        return self.world.getTimeStep()

    def getSimulationDuration(self):
        return self.simulationDuration

    def getTime(self):
        return self.world.getWorldTime()


class Robot:

    def __init__(self, sim, urdfPath, name):
        self.sim = sim
        self.robot = sim.world.addArticulatedSystem(urdfPath)
        self.robot.setName(name)

    @abstractmethod
    def initialize(self):
        pass

    def setState(self, position, velocity):
        self.robot.setState(position, velocity)

    def getGeneralizedCoordinate(self):
        return self.robot.getGeneralizedCoordinate()

    def getGeneralizedVelocity(self):
        return self.robot.getGeneralizedVelocity()        

    def getContacts(self):
        return self.robot.getContacts()

    def getFrameIdx(self, frameName):
        return self.robot.getFrameIdxByName(frameName)
    
    def getFramePosition(self, frameName):
        return self.robot.getFramePosition(self.getFrameIdx(frameName))

    def getFrameOrientation(self, frameName):
        return self.robot.getFrameOrientation(self.getFrameIdx(frameName))

    def getFrameAngularVelocity(self, frameName):
        return self.robot.getFrameAngularVelocity(self.getFrameIdx(frameName))

    def getBodyNames(self):
        return self.robot.getBodyNames()

    def getFrames(self):
        return self.robot.getFrames()

    def setGeneralizedForce(self, force):
        return self.robot.setGeneralizedForce(force)

    def getTime(self):
        return self.sim.getTime()


class Controller:

    def __init__(self, robot):
        self.robot = robot
    
    
    @abstractmethod
    def doControl(self):
        pass

    @abstractmethod
    def setTrajectory(self):
        pass

    @abstractmethod
    def updateState(self):
        pass

    @abstractmethod
    def computeControlInput(self):
        pass

    @abstractmethod
    def setControlInput(self):
        pass
    

class Plot:
    def __init__(self, simulation):
        self.sim = simulation
        self.controller =simulation.getController()
        self.totalPlotTime = self.sim.getSimulationDuration()

    @abstractmethod
    def getData(self):
        pass


class UI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulation Controller")
        self.initializeGraph()
        self.initializeButton()
        self.initializeWindow()
        self.show()

    def initializeButton(self):
        self.button = QPushButton(text = "Run", parent=self)
        self.button.clicked.connect(self.buttonPressPlay)
        self.button.move(400,0)
        self.isButtonPressed = False

    def initializeGraph(self):
        self.graphWidget = pg.PlotWidget(parent=self)
        self.graphWidget.setBackground('w')
        self.setCentralWidget(self.graphWidget)

    def initializeWindow(self):
        self.setMinimumHeight(300)
        self.setMinimumWidth(500)
        styles = {'color':'b', 'font-size':'10px'}
        self.graphWidget.setLabel('left', 'rad', **styles)
        self.graphWidget.setLabel('bottom', 'time [s]', **styles)

    @Slot()
    def buttonPressPlay(self):
        self.isButtonPressed = True
        print("Play button is pressed")
    
    def getIsButtonPressed(self):
        return self.isButtonPressed
    
    def setIsButtonPressed(self, booleanValue):
        self.isButtonPressed = booleanValue

    def plot(self,x,y):
        pen = pg.mkPen(color=(255, 0, 0))
        self.graphWidget.plot(x, y, pen=pen)

    def plot(self,x1,y1,x2,y2):
        pen1 = pg.mkPen(color=(0, 0, 255))
        self.graphWidget.plot(x1, y1, pen=pen1)        

        pen2 = pg.mkPen(color=(255, 0, 0))
        self.graphWidget.plot(x2, y2, pen=pen2)        

    def setPlotTitle(self, title):
        self.graphWidget.setTitle(title)

    # def setPlotLegend(self):
    #     pen1 = pg.mkPen(color=(0, 0, 255))
    #     self.graphWidget.plot([0 , 0], [0, 0], pen=pen1)        
    #     pen2 = pg.mkPen(color=(255, 0, 0))
    #     self.graphWidget.plot([0, 0], [0, 0], pen=pen2)      