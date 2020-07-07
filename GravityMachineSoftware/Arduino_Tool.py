# -*- coding: utf-8 -*-
"""
Created on Wed May 16 14:29:00 2018

@author: Francois
"""

import serial 	
import platform
import sys
import time
import numpy as np
import warnings
import serial.tools.list_ports
import crc16
'''       
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                        Arduino Communication
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''
class Arduino_Communication():
    def __init__(self,parent=None):

        self.serialconn=None
        self.platformName = platform.system()
        self.sendBufferSize = 17
        self.recBufferSize = 28

    def send_Integer(self,entier):
        if(sys.version_info[0] < 3):
            self.serialconn.write('%d\n' % entier)
            
        elif(sys.version_info[0] == 3):
            # Serial write for Python 3
            cmd_str0 = '{:d}\n'.format(entier)
            cmd0 = bytearray(cmd_str0.encode())
            self.serialconn.write(cmd0)

    def send_Float(self,entier):
        if(sys.version_info[0] < 3):
            self.serialconn.write('%f\n' % entier)
            
        elif(sys.version_info[0] == 3):
            # Serial write for Python 3
            cmd_str0 = '{:f}\n'.format(entier)
            cmd0 = bytearray(cmd_str0.encode())
            self.serialconn.write(cmd0)
        
            
    def hand_shaking_protocole(self):
        # Read string from Arduino
        print('try handshaking')
        initial_number = ord(self.serialconn.read())
        print(initial_number)
        print('first number received')
        if(initial_number == 1):
            print('\n ------------Communication established with the Arduino------------\n')
            cmd=bytearray(1)
            cmd[0]=2
            self.serialconn.write(cmd)

        second_number = ord(self.serialconn.read())
            
        if(second_number == 2):
            print('\n ------------Communication established both ways with the Arduino------------\n')
        print('handshaking finished')

class Arduino_Wheel(Arduino_Communication):

    def __init__(self,parent=None):
        super().__init__(parent)
        
        # AUTO-DETECT the Arduino!
        arduino_ports = [
                p.device
                for p in serial.tools.list_ports.comports()
                if 'Arduino' in p.description]
        
        if not arduino_ports:
            raise IOError("No Arduino found")
        else:
            print('Arduino found!')
        
        
        if len(arduino_ports) > 1:
            warnings.warn('Multiple Arduinos found - using the first')
        else:
            print('Using Arduino found at : {}'.format(arduino_ports[0]))
    
        self.serialconn = serial.Serial(arduino_ports[0],2000000)

    		# Serial communication setup with Arduino initialization
#        if self.platformName.lower() == 'darwin':
#            self.serialconn = serial.Serial('/dev/cu.usbmodem1411', 115200)
#        elif self.platformName.lower() == 'linux':
#            self.serialconn = serial.Serial('/dev/ttyACM0', 2000000)
#        elif self.platformName.lower() == 'windows':
#            self.serialconn=serial.Serial('COM6', 115200,timeout=1)
            
        self.serialconn.close()
        self.serialconn.open()
        time.sleep(2.0)
        print('Serial Connection Open')
        
        self.hand_shaking_protocole()
    
    
    def split_int_2byte(self,number):
        return int(number)% 256,int(number) >> 8

    def split_signed_int_2byte(self,number):
        if abs(number) > 32767:
            number = np.sign(number)*32767

        if number!=abs(number):
            number=65536+number
        return int(number)% 256,int(number) >> 8

    def split_int_3byte(self,number):
        return int(number)%256, int(number) >> 8, int(number) >> 16

    def data2byte_to_int(self,a,b):
        return a + 256*b

    def data2byte_to_signed_int(self,a,b):
        nb= a+256*b
        if nb>32767:
            nb=nb-65536
        return nb

    def data4byte_to_int(self,a,b,c,d):
        return a + (256)*b + (65536)*c + (16777216)*d
        
    def send_to_Arduino(self,command):  #function connected to the signal sent by Object Tracking
        # print('command sent to arduino',command)
        cmd = bytearray(self.sendBufferSize)
        cmd[0],cmd[1] = self.split_int_2byte(round(command[0]*100))                #liquid_lens_freq
        # cmd[2],cmd[3]=self.split_int_2byte(round(command[1]*1000))               #liquid_lens_ampl
        # cmd[4],cmd[5]=self.split_int_2byte(round(command[2]*100))                #liquidLens_offset
        cmd[2] = int(command[1])                                                   # Focus-Tracking ON or OFF
        cmd[3] = int(command[2])                                                   #Homing
        cmd[4] = int(command[3])                                                   #tracking
        cmd[5],cmd[6] = self.split_signed_int_2byte(round(command[4]*100))         #Xerror
        cmd[7],cmd[8] = self.split_signed_int_2byte(round(command[5]*100))       #Yerror                           
        cmd[9],cmd[10] = self.split_signed_int_2byte(round(command[6]*100))       #Zerror
        cmd[11],cmd[12] = self.split_int_2byte(round(0))#command[9]*10))               #averageDt (millisecond with two digit after coma) BUG
        cmd[13] = int(command[8])                                               # LED intensity

        # Adding Trigger flag for other Video Streams (Boolean)
        # print('Trigger command sent {}'.format(command[9]))
        cmd[14] = int(command[9])

        # Adding Sampling Interval for other Video Streams
        # Minimum 10 ms (0.01 s) Maximum: 3600 s (1 hour)
        # Min value: 1 to 360000 
        # print('Interval command sent {}'.format(command[10]))

        cmd[15], cmd[16] = self.split_int_2byte(round(100*command[10]))

        # signed_int:complement to 65536, max value:32767
        #use of round and '*100': transforme a float with two digit after coma into an integer


        self.serialconn.write(cmd)
        # print('commande sent_to arduino',cmd)

        #we send 10 numbers encoded on 18 bytes
    
    def read_from_arduino(self):

        # print('try to read from the serial rx buffer: ' + str(time.time())) #@@@

        # wait to receive data
        while self.serialconn.in_waiting==0:
            # print('wait for data to arrive')
            pass
        while self.serialconn.in_waiting%self.recBufferSize != 0:
            # print('wait for data to arrive')
            pass

        numBytesInRXBuffer = self.serialconn.in_waiting
        # print("number of bytes in the Rx buffer: " + str(numBytesInRXBuffer))

        '''
        # check if the right number of bytes are in the buffer
        if numBytesInRXBuffer%self.recBufferSize != 0:
            print('wrong number of bytes received')
            return [0,0,0,0, 0],0
        '''

        # get rid of old data
        if numBytesInRXBuffer > self.recBufferSize:
            # print('starting to get rid of old data')
            for i in range(numBytesInRXBuffer-self.recBufferSize):
                self.serialconn.read()
        
        '''
        # check if numBytesInRXBuffer is 0
        if numBytesInRXBuffer != self.recBufferSize:
            print('no data received')
            return [0,0,0,0, 0],0
        '''

        # read the buffer
        data=[]
        for i in range(self.recBufferSize):
            data.append(ord(self.serialconn.read()))

        '''
        # check crc
        crc = crc16.crc16xmodem(bytes(data[:-2]),0xffff) # CRC-CCITT (0xFFFF) - http://www.tahapaksu.com/crc/
        crc_received = (data[self.recBufferSize-2]<<8) + data[self.recBufferSize-1]
        if crc != crc_received:
            print('crc received: ' + hex(crc_received))
            print('crc calculated: ' + hex(crc))
            print('CRC failed')
            # return [0,0,0,0, 0],0
        '''

        YfocusPhase = self.data2byte_to_int(data[0],data[1])*2*np.pi/65535.

        # Xpos_arduino = self.data2byte_to_signed_int(data[2],data[3])
        # Ypos_arduino = self.data2byte_to_signed_int(data[4],data[5])
        Xpos_arduino = data[3]*2**24 + data[4]*2**16+data[5]*2**8 + data[6]
        if data[2]==1:
            Xpos_arduino =-Xpos_arduino

        Ypos_arduino = data[8]*2**24 + data[9]*2**16+data[10]*2**8 + data[11]
        if data[7]==1:
            Ypos_arduino =-Ypos_arduino

        Zpos_arduino = data[13]*2**24 + data[14]*2**16+data[15]*2**8 + data[16]
        if data[12]==1:
            Zpos_arduino =-Zpos_arduino

        manualMode = data[17]

        LED_measured = self.data2byte_to_int(data[18], data[19])

        timeStamp = data[20]*2**24 + data[21]*2**16+data[22]*2**8 + data[23]

        tracking_triggered = bool(data[24])
        # print('Trigger # from Arduino: ' + str(timeStamp))
        # print('-------')


        trigger_FL = bool(data[25])

        # print('Trigger command recvd {}'.format(trigger_FL))

        # sampling_interval  = self.data2byte_to_int(data[26], data[27])

        # print('Interval cmd recvd {}'.format(sampling_interval))

        #print('arduino data',[YfocusPhase,Xpos_arduino,Ypos_arduino,Zpos_arduino],manualMode)
        return [YfocusPhase,Xpos_arduino,Ypos_arduino,Zpos_arduino, LED_measured, tracking_triggered],manualMode



        
    def launch_homing(self):
        self.send_to_Arduino([0,0,1,0,0,0,0,0,0,0,0]) #[lens_freq,lens_ampl,lensoffset,homing,tracking,Xstep,Ystep,Zstep,Dt]
        
        while not self.serialconn.in_waiting:
            pass
        
        #HomingFlag = int(self.serialconn.read())

        #if HomingFlag == 1:
        #    print('Stage Homing Completed!')
        #else:
            
        #    ('Warning... Stage Homing not completed!')
        
        # This now 18 bytes - Deepak
        # This is now 20 bytes - Deepak 2018_12_15
        self.serialconn.read(self.recBufferSize) #ping pong game

    

class Arduino_LED_Panel(Arduino_Communication):
    
    def __init__(self,parent=None):
        
        super().__init__(parent)
        
        self.color=[0,0,0]
        self.isActivated=False
        self.isTrackingActivated=False
        self.previousTimeUpdate=0
        
    		# Serial communication setup with Arduino initialization
        if self.platformName.lower() == 'darwin':
            self.serialconn = serial.Serial('/dev/cu.usbmodem1411', 115200)
        elif self.platformName.lower() == 'linux':
            self.serialconn = serial.Serial('/dev/ttyACM0', 115200)
        elif self.platformName.lower() == 'windows':
            self.serialconn=serial.Serial('COM7', 115200,timeout=1) #Timeout de 1000
            
        self.serialconn.close()
        self.serialconn.open()
        
        self.hand_shaking_protocole()
    
        
    def update_Panel(self):        
        if self.isActivated:
            activation_nb=1
        else:
            activation_nb=0
        colorR=self.color[0]
        colorG=self.color[1]
        colorB=self.color[2]
        self.send_Float(activation_nb)
        self.send_Float(colorR)
        self.send_Float(colorG)
        self.send_Float(colorB)
        print('Instruction sent',[activation_nb,colorR,colorG,colorB])
        self.receive()
        
        
    def setPanel_On(self):
        self.isActivated=True
        self.update_Panel()
        return
    
    def setPanel_Off(self):
        self.isActivated=False
        self.update_Panel()
        return
    
    def update_Color(self,colorRGB):
        self.color=self.rgb_conversion(colorRGB)
        t=time.time()
        if (t-self.previousTimeUpdate>0.5): #maximum ten instruction by second
            self.update_Panel()
            self.previousTimeUpdate=t
        
    def rgb_conversion(self,colorRGB):
        return [int(colorRGB[0]),int(colorRGB[1]),int(colorRGB[2])]
        
    def receive(self):
        print('Arduino')
        print(int(self.serialconn.readline()))
        print(int(self.serialconn.readline()))
        print(int(self.serialconn.readline()))
        print(int(self.serialconn.readline()))
        
    def activateTracking(self,Z):
        #if self.isTrackingActivated:   
        #else:
            
        return
