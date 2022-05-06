import sys
from CAMELRaisimLib import Simulation, CAMELThread
from PySide6.QtWidgets import QApplication

from DoublePendulumRobot import DoublePendulumRobot
from DoublePendulumPDController import DoublePendulumPDController
from DoublePendulumIDController import DoublePendulumIDController
from DoublePendulumPlot import DoublePendulumPlot

class DoublePendulumSimulation(Simulation):
    def __init__(self):
        super().__init__()
        self.setDT(0.005)
        self.setSimulationDuration(duration = 5.0)
        self.setFastSimulation(False)
        self.setDataPlot(True)
        self.initializeServer()

        # set robot class
        self.robot = DoublePendulumRobot(self)
        
        # set controller 
        self.PDcontroller = DoublePendulumPDController(self.robot)
        self.PDcontroller.setPDGain(300,30)
        self.IDcontroller = DoublePendulumIDController(self.robot)

        self.setController(self.PDcontroller)
        
        # set plot
        self.plot = DoublePendulumPlot(self)
        self.setPlot(self.plot)

        # set thread
        self.thread = CAMELThread(self)

    def run(self):
        self.thread.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    DoublePendulumSimulation().run()
    app.exec()
