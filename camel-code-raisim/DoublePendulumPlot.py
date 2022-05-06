from CAMELRaisimLib import Plot
import numpy as np

class DoublePendulumPlot(Plot):
    def __init__(self, simulation):
        super().__init__(simulation)
        self.totalDataNum = 2

        self.t = np.array([])
        self.data1 = np.array([])
        self.data2 = np.array([])
        
    # override
    def setData(self):
        self.t = np.append(self.t, [self.sim.getTime()])
        self.data1 = np.append(self.data1, [self.controller.getDesiredPosition()[1]])
        self.data2 = np.append(self.data2, [self.controller.getPosition()[1]])

        # self.data1 = np.append(self.data1, [self.controller.getDesiredVelocity()[0]])
        # self.data2 = np.append(self.data2, [self.controller.getVelocity()[0]])

        # self.data1 = np.append(self.data1, [self.controller.getInputTorque()[0]])
        # self.data2 = np.append(self.data2, [self.controller.getInputTorque()[1]])

