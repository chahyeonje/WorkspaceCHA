import sys
from CAMELRaisimLib import Simulation, CAMELThread
from PySide6.QtWidgets import QApplication

from SingleLegged3DOFRobot import SingleLegged3DOFRobot
from SingleLegged3DOFIDController import SingleLegged3DOFIDController
from SingleLegged3DOFPlot   import SingleLegged3DOFPlot

class SingleLegged3DOFSimulation(Simulation):
    def __init__(self):
        super().__init__()
        self.setDT(0.005)
        self.setSimulationDuration(duration = 2.0) 
        self.setFastSimulation(True)
        self.setDataPlot(True)
        self.initializeServer()
        
        # set robot class
        self.robot = SingleLegged3DOFRobot(self)
        
        # set controller 
        self.IDcontroller =  SingleLegged3DOFIDController(self.robot)
        self.IDcontroller.setPDGain(200.0, 20.0)
        self.setController(self.IDcontroller)
        
        ## set plot
        self.plot = SingleLegged3DOFPlot(self)
        self.setPlot(self.plot)

        # set thread
        self.thread = CAMELThread(self)

    def run(self):
        self.thread.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    SingleLegged3DOFSimulation().run()
    app.exec()
