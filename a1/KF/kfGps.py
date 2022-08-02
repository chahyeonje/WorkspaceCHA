import numpy as np

class KF:
    def __init__(self):
        self.dt
        self.startLat
        self.startLng
        self.goalLat
        self.goalLng
        self.yaw
        self.currentVelocity
        self.currentVelocityToN
        self.currentVelocityToW
        self.systemMatrix
        self.measurementMatrix

    def setup(self):
        self.systemMatrix = np.array([[1,0,self.dt,0],[0,1,0,self.dt],[0,0,1,0]])
        pass

    def getVelocity(self):
        return self.currentVelocity, self.currentVelocityToN, self.currentVelocityToW

    def getYaw(self):
        pass

    def timeUpdate(self):
        pass

    def measurementUpdate(self): #10초에 1회
        pass
    
    def doFiltering(self):
        pass
