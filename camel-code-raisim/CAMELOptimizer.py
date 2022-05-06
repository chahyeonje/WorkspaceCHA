import numpy as np

class GradientDescentSolver:
    def __init__(self):
        self.stepSize = 0.01
        self.maximumIteration = 1000
        self.terminateCondition = 1e-6
        self.delta = 1e-6
        self.iteration = 0
        self.terminateFlag = False

    def setStepSize(self, stepSize):
        self.stepSize = stepSize

    def setMaximumIteration(self, maximumIteration):
        self.maximumIteration = maximumIteration

    def setTerminateCondition(self, terminateCondition):
        self.terminateCondition = terminateCondition

    def setObjectiveFunction(self, objectiveFunction):
        self.objectiveFunction = objectiveFunction

    def setInitialPoint(self, initialPoint):
        self.x = initialPoint
        self.dim = np.size(initialPoint)
        self.gradient = np.zeros(self.dim)
    
    def computeGradient(self):
        functionValue = self.objectiveFunction(self.x)
        for i in range(self.dim):
            temp_x = self.x.copy()
            temp_x[i] += self.delta 
            self.gradient[i] = (self.objectiveFunction(temp_x) - functionValue) / self.delta
        self.RMSgradient = (self.gradient.dot(self.gradient) / self.dim) ** (1/2)

    def updateVariables(self):
        self.x = self.x.copy() - self.stepSize * self.gradient
    
    def checkTerminateCondition(self):
        if(self.iteration == self.maximumIteration):
            self.terminateReason = "maximum iteration"
            self.terminateFlag = True
        elif(self.RMSgradient < self.terminateCondition):
            self.terminateReason = "terminate conditon"
            self.terminateFlag = True

    def solve(self):
        for i in range(self.maximumIteration):
            self.iteration += 1
            self.computeGradient()
            self.updateVariables()
            self.checkTerminateCondition()
            if(self.terminateFlag):
                print(self.terminateReason)
                break
            

    
    
