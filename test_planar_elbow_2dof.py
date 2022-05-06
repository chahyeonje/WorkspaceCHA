import sys
from CAMELRaisimLib import Simulation, CAMELThread
from PySide6.QtWidgets import QApplication

from test_planar_elbow_2dofRobot import test_planar_elbow_2dofRobot
from test_planar_elbow_2dof_IDController import test_planar_elbow_2dof_IDController
from test_planar_elbow_2dof_PDController import test_planar_elbow_2dof_PDController
from SingleLegged3DOFPlot   import SingleLegged3DOFPlot

class test_planar_elbow_2dof(Simulation):
    def __init__(self):
        super().__init__()
        self.setDT(0.005)
        self.setSimulationDuration(duration = 2.0) 
        self.setFastSimulation(True)
        self.setDataPlot(True)
        self.initializeServer()
        
        # set robot class
        self.robot = test_planar_elbow_2dofRobot(self)
        
        # set controller 
        self.IDcontroller = test_planar_elbow_2dof_IDController(self.robot)
        self.IDcontroller.setPDGain(200.0, 20.0)
        self.PDcontroller = test_planar_elbow_2dof_PDController(self.robot)
        self.PDcontroller.setPDGain(200.0, 20.0)

        self.setController(self.PDcontroller)
        
        ## set plot
        self.plot = SingleLegged3DOFPlot(self)
        self.setPlot(self.plot)

        # set thread
        self.thread = CAMELThread(self)

    def run(self):
        self.thread.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    test_planar_elbow_2dof().run()
    app.exec()
