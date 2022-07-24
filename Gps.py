import serial
import string
import pynmea2
import time
class Gps:
    def __init__(self):
        self.port = '/dev/ttyUSB0'
        self.ser = serial.Serial(self.port, baudrate=9600)
        
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

        else :
            time.sleep(1)
            self.data = self.ser.readline()
            self.msg = self.msg = pynmea2.parse(str(self.data, 'utf-8'))
            self.timestamp = str(self.msg.timestamp)[0:8]
            self.lat = self.msg.latitude 
            self.lng = self.msg.longitude
        return self.lat, self.lng

    def startGpsPosition(self, latitude, longitude):
        self.startLat, self.startLng = latitude,longitude 

    def currentGpsPosition(self, latitude, longitude):
        self.currentLat, self.currentLng = latitude, longitude

    def goalGpsPosition(self, latitude, longitude):
        self.goalLat, self.goalLng = latitude, longitude

    def getDistanceStartToGoal(self):
        self.distanceToNorth = (self.goalLat - self.startLat)*100000*1.08
        self.distanceToEast = (self.goalLng - self.startLng)*100000*0.98
        return self.distanceToNorth, self.distanceToEast

    def getDistanceCurrentToGoal(self):
        self.distanceToNorth = (self.goalLat - self.currentLat)*100000*1.08
        self.distanceToEast = (self.goalLng - self.currentLng)*100000*0.98
        return self.distanceToNorth, self.distanceToEast