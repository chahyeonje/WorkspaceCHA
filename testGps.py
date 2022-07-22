import Gps

testGps = Gps.Gps()

while True:
    try:
        testGps.getGps()
        testGps.recordOutput('testtest')
        print(testGps.getGps().timestamp, testGps.getGps().lat, testGps.getGps().lng)
    except:
        continue
