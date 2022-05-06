from abc import abstractmethod

from CAMELRaisimLib import Controller

class PDController(Controller):

    def __init__(self, robot):
        super().__init__(robot)
        self.positionError = 0
        self.differentialError = 0
        
    def setPDGain(self, PGain, DGain):
        self.PGain = PGain
        self.DGain = DGain
    
    def doControl(self):
        return super().doControl()

    def setTrajectory(self, desiredPosition, desiredVelocity):
        self.desiredPosition = desiredPosition
        self.desiredVelocity = desiredVelocity

    def updateState(self):
        return super().updateState()

    def computeControlInput(self):
        return super().computeControlInput()

    def setControlInput(self):
        return super().setControlInput()


class PIDController(PDController):
    def __init__(self, robot):
        super().__init__(robot)
        self.cumulativeError = 0
    
    def setPDGain(self, PGain, DGain):
        return super().setPDGain(PGain, DGain)
    
    def setPIDGain(self, PGain, IGain, DGain):
        self.setPDGain(PGain, DGain)
        self.IGain = IGain

    def doControl(self):
        return super().doControl()
    
    def setTrajectory(self, desiredPosition, desiredVelocity):
        return super().setTrajectory(desiredPosition, desiredVelocity)
    
    def updateState(self):
        return super().updateState()

    def computeControlInput(self):
        return super().computeControlInput()
     
    def setControlInput(self):
        return super().setControlInput()
    

class InverseDynamicsController(Controller):
    def __init__(self, robot):
        super().__init__(robot)
        self.positionError = 0
        self.differentialError = 0

    def setPDGain(self, PGain, DGain):
        self.PGain = PGain
        self.DGain = DGain

    def doControl(self):
        return super().doControl()

    def setTrajectory(self, desiredPosition, desiredVelocity, desiredAcceleration):
        self.desiredPosition = desiredPosition
        self.desiredVelocity = desiredVelocity
        self.desiredAcceleration = desiredAcceleration

    def updateState(self):
        return super().updateState()

    @abstractmethod
    def updateMassMatrix(self):
        pass

    def computeControlInput(self):
        return super().computeControlInput()
    
    def setControlInput(self):
        return super().setControlInput()
    
    
        

class MPCController(Controller):
    def __init__(self, robot):
        super().__init__(robot)
        
    def doControl(self):
        return super().doControl()
    
    def setTrajectory(self, trajectorySequence):
        self.trajectorySequence = trajectorySequence
    
    def updateState(self):
        return super().updateState()
    
    def computeControlInput(self):
        return super().computeControlInput()

    def setControlInput(self):
        return super().setControlInput()
    
    def setMPCHorizon(self, MPCHorizon):
        self.MPCHorizon = MPCHorizon

    def setSolver(self, solver):
        self.solver = solver
