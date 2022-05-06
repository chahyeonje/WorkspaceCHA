from CAMELRaisimLib import Plot
import numpy as np

class CubliPlot(Plot):
    def __init__(self, simulation):
        super().__init__(simulation)
        self.totalDataNum = 2

        self.t = np.array([])
        self.data1 = np.array([])
        self.data2 = np.array([])
        
    # override
    def setData(self):
        self.t = np.append(self.t, [self.sim.getTime()])
        self.data1 = np.append(self.data1, [self.controller.getDesiredPosition()])
        self.data2 = np.append(self.data2, [self.controller.getPosition()])

        # self.data2 = np.append(self.data2, [self.controller.getInputTorque()])

        # self.data1 = np.append(self.data1, [self.controller.getDesiredVelocity()])
        # self.data2 = np.append(self.data2, [self.controller.getVelocity()])
