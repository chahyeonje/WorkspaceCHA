import Gps

testGps = Gps.Gps()

# while True:
#     try:
#         testGps.getGps()
#         testGps.recordOutput('testtest')
#         print(testGps.getGps().timestamp, testGps.getGps().lat, testGps.getGps().lng)
#     except:
#         continue

testGps.startGpsPosition(35.0000, 129.0000)
testGps.goalGpsPosition(35.0001, 129.0001)
distanceN, distanceE = testGps.getDistanceStartToGoal()
print("distanceN : ", distanceN, "distanceE : ", distanceE)

testGps.currentGpsPosition(35.00005,129.00005)
distanceN, distanceE = testGps.getDistanceCurrentToGoal()
print("distanceN : ", distanceN, "distanceE : ", distanceE)
