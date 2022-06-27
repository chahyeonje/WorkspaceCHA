from zz_class import FourCal
from z_refectoring_class import twoDofArm


testParameter = twoDofArm(1,20,3,14)
testParameter.massMatrix

# testParameter.stateUpdate(10,10,20,20,[1,1])
testParameter.stateUpdate([10,10],[20,20],[1,1])

print(testParameter.massMatrix())
print(testParameter.currentMomenta())

# testParameter.printTest()

# a=FourCal()
# b=FourCal()
# a.SetData(4,1)
# b.SetData(8,3)

# print("a.Add = ", a.Add())
# print("\na.Sub = ", a.Sub())
# print("\na.RSub = ", a.RSub())

# print("\nb.Add = ", b.Add())
# print("\nb.Sub = ", b.Sub())
# print("\nb.RSub = ", b.RSub())

