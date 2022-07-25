import Gps

testGps = Gps.Gps()
startlat,startlng = testGps.getGps()
testGps.startGpsPosition(startlat, startlng)
print("----startposition. lat: ", startlat," lng: ", startlng ,"----")
testGps.goalGpsPosition(35.2323, 129.0824)
distanceN, distanceE = testGps.getDistanceStartToGoal()
print("북쪽으로 남은 거리 : ", distanceN, "동쪽으로 남은 거리 : ", distanceE)

while True:
    try:
        lat, lng = testGps.getGps()
        testGps.currentGpsPosition(lat,lng)
        distanceN, distanceE = testGps.getDistanceCurrentToGoal()
        print("북쪽으로 남은 거리 : ", distanceN, "동쪽으로 남은 거리 : ", distanceE)
        print("현재 위치: ",lat,lng)
        print(" ")
    except:
        continue






