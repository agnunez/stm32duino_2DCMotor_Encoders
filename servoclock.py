#!/usr/bin/python
import serial, time
def deg2pos(deg):
  return int(-deg/36.*320)
ser = serial.Serial('/dev/ttyUSB0',115200)
for i in range(0,61):
  ser.write('1 %s' % deg2pos(i*6))
  print(i)
  time.sleep(1)
ser.write('1 0')
ser.close()
