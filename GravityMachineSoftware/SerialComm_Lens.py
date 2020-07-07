# Serial communication protocol to control optotune liquid lens
import serial 	
import platform
import sys
import time
import numpy as np
import warnings
import serial.tools.list_ports






class lens_comm:

	def __init__(self):

		self.serialconn=None

		# Auto-detect the lens-driver

		for p in serial.tools.list_ports.comports():
			print(p)

		lens_ports = [
				p.device
				for p in serial.tools.list_ports.comports()
				if 'Optotune' in p.description]
		
		if not lens_ports:
			raise IOError("No Optotune lens found")
		else:
			print('Optotune lens driver found!')
		
	   
	
		self.serialconn = serial.Serial(lens_ports[0],115200)
			
		self.serialconn.close()
		self.serialconn.open()
		time.sleep(2.0)
		print('Serial Connection Open')

		self.handshake()

		self.mode = "Sinusoid"
		
	def sendData(self, data):
		self.serialconn.write(data)

	# def recData(self, nBytes):
		
	# 	data = []

	# 	for i in range(nBytes):
	# 		data.append(ord(self.serialconn.read()))

		
	# 	# Join the byte array to create a string
	# 	data_str = ''.join(chr(i) for i in data)
	# 	return data_str

	def recData(self, nBytes):
		
		data = bytearray(nBytes)

		for i in range(nBytes):
			data[i] = ord(self.serialconn.read())

		return data



	def handshake(self):

		start_cmd = bytearray("Start", 'utf-8')

		self.sendData(start_cmd)

		rec_data = self.recData(7)
		print(rec_data)


		if(rec_data == bytearray(b'Ready\r\n')):
			print('Handshaking complete')

	def split_int_4byte(self, number):
		byte0 = (number >> 24) & 0xFF
		byte1 = (number >> 16) & 0xFF
		byte2 = (number >> 8) & 0xFF
		byte3 = number & 0xFF
		
		# Highest byte is byte 0, byte3 is lowest
		return byte0, byte1, byte2, byte3

	def split_int_2byte(self,number):
		# HIGH Byte, LOW Byte
		return int(number) >> 8, int(number)% 256

	def split_signed_int_2byte(self,number):
		if abs(number) > 32767:
			number = np.sign(number)*32767

		if number!=abs(number):
			number=65536+number

		# HIGH Byte, LOW Byte
		return int(number) >> 8, int(number)% 256


	def setMode(self, mode):
		self.mode = mode

	def set_currs(self, lower_curr, upper_curr):
		self.lower_curr = lower_curr
		self.upper_curr = upper_curr

	def setFreq(self, freq):
		self.freq = freq



	def sendMode(self):

		cmd = bytearray(4) # Data array 

		cmd[0], cmd[1] = ord('M'),ord('w')
		cmd[3] = ord('A')


		if self.mode == "Sinusoid":
			cmd[2] = ord('S')
		elif self.mode == "Square":
			cmd[2] = ord('Q')
		elif self.mode == "DC":
			cmd[2] = ord('D')
		elif self.mode == "Tringular":
			cmd[2] = ord('T')

		# print('Sent data without crc bytes: {}'.format(cmd))

		cmd_crc = self.addcrcCheckSum(cmd)

		# print('Sent data with crc bytes: {}'.format(cmd_crc))

		self.sendData(cmd_crc)

		rec_data = self.recData(7)

		rec_string = bytearray(3)

		rec_string[0], rec_string[1], rec_string[2] = cmd_crc[0],cmd_crc[2], cmd_crc[3]

		check = self.calculate_crc(rec_data[:5])
		if(rec_data[:3] == rec_string and check==0 ):
			print('Mode set successfully to : {}'.format(self.mode))



	def sendProperty(self, prop, value):

		cmd = bytearray(8)
		# Property and Write Flags
		cmd[0], cmd[1] = ord('P'),ord('w')
		# Channel A flag
		cmd[3] = ord('A')

		if(prop=="UpperCurr"):
			cmd[2] = ord('U')
			# High to low bytes
			cmd[4], cmd[5] = self.split_signed_int_2byte(value)
			# Add zeros as stuffing bytes if changing the current
			cmd[6], cmd[7] = 0, 0
		elif(prop=="LowerCurr"):
			cmd[2] = ord('L')
			# High to low bytes
			cmd[4], cmd[5] = self.split_signed_int_2byte(value)
			# Add zeros as stuffing bytes if changing the current
			cmd[6], cmd[7] = 0, 0

		else:
			cmd[2] = ord('F')
			# High to Low bytes
			cmd[4], cmd[5], cmd[6], cmd[7] = self.split_int_4byte(value)

		print('Sent data without crc bytes: {}'.format(cmd))

		cmd_crc = self.addcrcCheckSum(cmd)

		print('Sent data with crc bytes: {}'.format(cmd_crc))

		self.sendData(cmd_crc)






		

		
	def calculate_crc(self, data):

		crc_sum = 0

		for ii in range(len(data)):
			crc_sum = self.crc_16_update(crc_sum, data[ii])

		return crc_sum # crc checksum over all data elements


	def crc_16_update(self, crc, a):

		crc ^= a

		for ii in range(8):
			if (crc & 1):
				crc = (crc >> 1) ^ 0xA001
			else:
				crc = (crc >> 1)
		
		return crc

	def addcrcCheckSum(self, data):

		dataLen = len(data)
		# We need two more bytes to store the checksum 
		data_new = bytearray(dataLen + 2)

		crc_sum = self.calculate_crc(data)

		data_new[:dataLen] = data

		# For CRC Low byte first and then High byte
		# the int2byte function returns High Byte first
		data_new[dataLen + 1 ], data_new[dataLen] = self.split_int_2byte(crc_sum)

		return data_new








lens1 = lens_comm()
lens1.setMode("Sinusoid")
lens1.sendMode()
lens1.sendProperty('Freq', 1000)
lens1.sendProperty('UpperCurr', 50)
lens1.sendProperty('LowerCurr', 50)
