import serial
import time
import random

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
time.sleep(0.1)

while True:
    num = random.randint(0, 9)
    print(num)
    arduino.write(str(num).encode('utf-8'))
    arduino.flush()
    print(arduino.readline().decode('utf-8').rstrip())
    time.sleep(2)
