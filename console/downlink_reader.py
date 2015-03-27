import serial
import time
import sys
import struct

## Again: build the controller before running this!
sys.path.append("./pylib")
sys.path.append("../controller/build/nanopb/generator/proto")
import downlink_pb2

sio = serial.Serial("/dev/tty.SerialAdaptor-SPPDev",19200)

while 1:
  c = 0
  while c < 10:
    ind = sio.read(1)
    if ind == 'X':
      c += 1
    else:
      c = 0

  framelen = struct.unpack('!H', sio.read(2))[0]
  body = sio.read(framelen)
  f = downlink_pb2.Frame()
  f.ParseFromString(body)
  print f
