import serial
import string
import pynmea2

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
            print(self.timestamp, self.lat, self.lng)
