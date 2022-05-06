import sys
from CAMELRaisimLib import Simulation, CAMELThread
from PySide6.QtWidgets import QApplication

from CubliRobot import CubliRobot
from CubliPDControll_UnitTrajactory import CubliPDController
from CubliPlot import CubliPlot

class CubliSimulation(Simulation):
    def __init__(self):
        super().__init__()
        self.setDT(0.005)
        self.setSimulationDuration(duration = 10.0)
        self.setFastSimulation(False)
        self.setDataPlot(True)
        self.initializeServer()

        # set robot class
        self.robot = CubliRobot(self)
        
        # set controller 
        self.PDcontroller = CubliPDController(robot=self.robot)
        self.PDcontroller .setPDGain(-5,-5)

        self.setController(self.PDcontroller)
        # set plot
        self.plot = CubliPlot(self)
        self.setPlot(self.plot)

        # set thread
        self.thread = CAMELThread(self)
    
    def run(self):
        self.thread.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    CubliSimulation().run()
    app.exec()
