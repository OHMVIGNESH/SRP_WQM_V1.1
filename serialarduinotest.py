import serial 
import time

arduino = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)


while True:
    try:
        data = arduino.readline()
        if data:
            print(data)
    except:
        arduino.close() 