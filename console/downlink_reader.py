import serial
import time
import sys
import struct

sio = serial.Serial("/dev/tty.usbmodem1411",9600)

while 1:
  line = sio.readline()
  if "Got" in line:
    data = sio.readline().rstrip()
    if len(data) != 8:
      print "woops:"
      print data
    else:
      vals = struct.unpack('hhhh', data)
      print vals
    sys.stdout.flush()

