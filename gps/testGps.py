import Gps

testGps = Gps.Gps()

testGps.setupGps(35.2323,129.0824)

distanceN, distanceE = testGps.getDistanceStartToGoal()
print("----startposition. lat: ", testGps.startLat," lng: ", testGps.startLng ,"----")
print("북쪽으로 남은 거리 : ", distanceN, "동쪽으로 남은 거리 : ", distanceE)
k=0
while True:
    try:
        testGps.showState()
        #testGps.recordOutput('0725')
        # lat, lng = testGps.getGps()
        # print(k, lat, lng)
        # k=k+1

        
    except:
        continue






