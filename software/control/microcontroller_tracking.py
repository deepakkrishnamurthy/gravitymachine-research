import platform
import serial
import serial.tools.list_ports
import time
import numpy as np

from control._def import *

# add user to the dialout group to avoid the need to use sudo

class Microcontroller():
    def __init__(self,parent=None):
        self.serial = None
        self.platform_name = platform.system()
        self.tx_buffer_length = 4
        self.rx_buffer_length = 4

        # AUTO-DETECT the Arduino! By Deepak
        arduino_ports = [
                p.device
                for p in serial.tools.list_ports.comports()
                if 'Arduino' in p.description]
        if not arduino_ports:
            raise IOError("No Arduino found")
        if len(arduino_ports) > 1:
            warnings.warn('Multiple Arduinos found - using the first')
        else:
            print('Using Arduino found at : {}'.format(arduino_ports[0]))

        # establish serial communication
        self.serial = serial.Serial(arduino_ports[0],2000000)
        time.sleep(0.2)
        print('Serial Connection Open')

    def close(self):
        self.serial.close()


    def send_command(self,command):
        cmd = bytearray(self.tx_buffer_length)
        '''
        cmd[0],cmd[1] = self.split_int_2byte(round(command[0]*100))                #liquid_lens_freq
        # cmd[2],cmd[3]=self.split_int_2byte(round(command[1]*1000))               #liquid_lens_ampl
        # cmd[4],cmd[5]=self.split_int_2byte(round(command[2]*100))                #liquidLens_offset
        cmd[2] = int(command[1])                                                   # Focus-Tracking ON or OFF
        cmd[3] = int(command[2])                                                   #Homing
        cmd[4] = int(command[3])                                                   #tracking
        cmd[5],cmd[6] = self.split_signed_int_2byte(round(command[4]*100))         #Xerror
        cmd[7],cmd[8] = self.split_signed_int_2byte(round(command[5]*100))         #Yerror                           
        cmd[9],cmd[10] = self.split_signed_int_2byte(round(command[6]*100))        #Zerror
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
        '''
        self.serial.write(cmd)

    def read_received_packet(self):
        # wait to receive data
        while self.serialconn.in_waiting==0:
            pass
        while self.serialconn.in_waiting % self.rx_buffer_length != 0:
            pass

        num_bytes_in_rx_buffer = self.serial.in_waiting

        # get rid of old data
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            print('getting rid of old data')
            for i in range(num_bytes_in_rx_buffer-self.rx_buffer_length):
                self.serial.read()
        
        # read the buffer
        data=[]
        for i in range(self.rx_buffer_length):
            data.append(ord(self.serialconn.read()))

        '''
        YfocusPhase = self.data2byte_to_int(data[0],data[1])*2*np.pi/65535.
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
        trigger_FL = bool(data[25])
        return [YfocusPhase,Xpos_arduino,Ypos_arduino,Zpos_arduino, LED_measured, tracking_triggered],manualMode
        '''
        return data

# Define a micro controller emulator

class Microcontroller_Simulation():
    def __init__(self,parent=None):
        #REC_DATA = ['FocusPhase', 'X_stage', 'Y_stage', 'Theta_stage', 'track_obj_image', 'track_obj_stage']
        self.FocusPhase = 0
        self.Xpos = 0
        self.Ypos = 0
        self.Thetapos = 0

        self.deltaX_stage = 0
        self.deltaY_stage = 0
        self.deltaTheta_stage = 0
        
        self.track_obj_image = 0
        self.track_focus = 0

        self.track_obj_stage = 1

        self.RecData = {'FocusPhase':self.FocusPhase, 'deltaX_stage': self.deltaX_stage, 'deltaY_stage':self.deltaY_stage, 'deltaTheta_stage':self.deltaTheta_stage, 'track_obj_image':self.track_obj_image, 'track_obj_stage' : self.track_obj_stage}

        self.SendData = {key:[] for key in SEND_DATA}

    def close(self):
        pass

    def toggle_LED(self,state):
        pass
    
    def toggle_laser(self,state):
        pass

    def move_x(self,delta):
        self.deltaX_stage = delta
        self.Xpos += delta

    def move_y(self,delta):
        self.deltaY_stage = delta
        self.Ypos += delta

    def move_Theta(self,delta):
        self.deltaTheta_stage = delta
        self.Thetapos += delta

    def move_z(self,delta):
        pass
    
    def send_command(self,command):
        #SEND_DATA = ['X_order', 'Y_order', 'Z_order', 'track_obj_image', 'track_focus', 'homing_state']
        X_order, Y_order, Theta_order = command['X_order'], command['Y_order'], command['Theta_order']
        self.track_obj_image = command['track_obj_image']
        self.track_focus = command['track_focus']

        self.move_x(X_order)
        self.move_y(Y_order)
        self.move_Theta(Theta_order)


    def read_received_packet(self):
        self.RecData = {'FocusPhase':self.FocusPhase, 'deltaX_stage': self.deltaX_stage, 'deltaY_stage':self.deltaY_stage, 'deltaTheta_stage':self.deltaTheta_stage, 'track_obj_image':self.track_obj_image, 'track_obj_stage' : self.track_obj_stage}
        return self.RecData



# from Gravity machine
def split_int_2byte(number):
    return int(number)% 256,int(number) >> 8

def split_signed_int_2byte(number):
    if abs(number) > 32767:
        number = np.sign(number)*32767

    if number!=abs(number):
        number=65536+number
    return int(number)% 256,int(number) >> 8

def split_int_3byte(number):
    return int(number)%256, int(number) >> 8, int(number) >> 16

def data2byte_to_int(a,b):
    return a + 256*b

def data2byte_to_signed_int(a,b):
    nb= a+256*b
    if nb>32767:
        nb=nb-65536
    return nb

def data4byte_to_int(a,b,c,d):
    return a + (256)*b + (65536)*c + (16777216)*d
