import serial
import string
import pynmea2
import time
class Gps:
    def __init__(self):
        self.port = '/dev/ttyUSB0'
        self.ser = serial.Serial(self.port, baudrate=9600)
        self.firstRun = True
        
    def recordOutput(self,title):
        f = open(title,'a')
        f.write(f'{self.timestamp}, {self.lat}, {self.lng}')
        f.write('\n')
        f.close()

    def getGps(self):
        self.data = self.ser.readline()
        if str(self.data[0:6], 'utf-8')=="$GNGGA" :
            self.msg = pynmea2.parse(str(self.data, 'utf-8'))
            self.timestamp = str(self.msg.timestamp)[0:8]
            self.lat = self.msg.latitude 
            self.lng = self.msg.longitude
            return  self.lat, self.lng

        else :
            pass

    def setupGps(self, goallat, goallng):
        lat, lng = self.getGps()
        self.startGpsPosition(lat, lng)
        self.goalGpsPosition(goallat, goallng)

    def startGpsPosition(self, latitude, longitude):
        self.startLat, self.startLng = latitude,longitude 

    def currentGpsPosition(self):
        self.latIn, self.lngIn = self.getGps()
        if(self.firstRun == True):
            prevLat = self.startLat
            prevLng = self.startLng
            self.firstRun = False
        else : 
            pass
        
        if(self.latIn == 0):
            self.currentLat = prevLat
            self.currentLng = prevLng
        else:
            self.currentLat = self.latIn
            self.currentLng = self.lngIn
            prevLat = self.latIn
            prevLng = self.lngIn
        
        return self.currentLat, self.currentLng

    def goalGpsPosition(self, latitude, longitude):
        self.goalLat, self.goalLng = latitude, longitude

    def getDistanceStartToGoal(self):
        self.distanceToNorth = (self.goalLat - self.startLat)*100000*1.08
        self.distanceToEast = (self.goalLng - self.startLng)*100000*0.98
        return self.distanceToNorth, self.distanceToEast

    def getDistanceCurrentToGoal(self): #현재 위치에서 목적지까지 북,동 방향으로 각각 몇 미터 남았는지 알려줌
        self.flat, self.flng = self.currentGpsPosition()
        self.distanceToNorth = (self.goalLat - self.flat)*100000*1.08 #단위 : m
        self.distanceToEast = (self.goalLng - self.flng)*100000*0.98 #단위 : m
        return self.distanceToNorth, self.distanceToEast

    def showState(self):
        distanceN, distanceE = self.getDistanceCurrentToGoal()
        print("북쪽으로 남은 거리 : ", distanceN, "동쪽으로 남은 거리 : ", distanceE)
        print("현재 위치: ",self.flat,self.flng)
        print(" ")
