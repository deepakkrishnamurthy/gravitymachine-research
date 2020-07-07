import serial
import time
import struct

# num_bytes = 10
num_packets = 1000

ser = serial.Serial('/dev/tty.usbmodem1411', 12000000)
# cmd_str = '*'*num_bytes
cmd_str = '000'
cmd = bytearray(cmd_str.encode())

t0 = time.time()
for i in range(num_packets):
	ser.write(cmd)
	phase_2byte = ser.read(2)
	# phase = struct.unpack('>Bi',phase_2byte)
	phase = phase_2byte[0]*256 + phase_2byte[1]
	print(phase)
	#ser.read(2)

def setSweepFrequency(ser,liquidLensSweepFrequency):
  liquidLensSweepFrequency_2bytes = liquidLensSweepFrequency*100
  cmd = bytearray(b'100')
  cmd[1] = int(liquidLensSweepFrequency_2bytes) % 256
  cmd[2] = int(liquidLensSweepFrequency_2bytes) >> 8

def setVAmplitude(ser,liquidLensVAmplitude): #liquidLensVAmplitude<20
  liquidLensVAmplitude_2bytes = (liquidLensVAmplitude/20)*65536;
  cmd = bytearray(b'100')
  cmd[1] = int(liquidLensVAmplitude_2bytes) % 256
  cmd[2] = int(liquidLensVAmplitude_2bytes) >> 8

def setVOffset(ser,liquidLensVOffset):
  liquidLensVOffset_2bytes = ((liquidLensVOffset-34)/20)*65536;
  cmd = bytearray(b'100')
  cmd[1] = int(liquidLensVOffset_2bytes) % 256
  cmd[2] = int(liquidLensVOffset_2bytes) >> 8


delta_t = time.time() - t0
print(delta_t)
ser.close()
