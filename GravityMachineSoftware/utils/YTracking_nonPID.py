# -*- coding: utf-8 -*-

#Tuning the parameters:
    #Integrator max(resp .min): max nb of steps the motor can make in DeltaT

from collections import deque
import scipy.optimize as opt
import scipy.interpolate as interpolate
import scipy.signal as signal
import numpy as np


class YTracker():
    
    def __init__(self,parent=None):
        
        self.YdequeLen=50
        self.YfocusMeasure = deque(maxlen = self.YdequeLen)  #The length of the buffer must be ~fps_sampling/liquid_lens_freq
        self.YfocusPosition = deque(maxlen = self.YdequeLen) #in mm
        self.YfocusPhase = deque(maxlen = self.YdequeLen)
        
        # kept for class compatibility
        self.YmaxFM = 0
        self.gain=1
        self.maxGain=1

        self.error = 0
        
        self.ampl=0.025
        self.freq=2

        self.window_size = 25
        
        # flags
        self.TrackingCycleStarted = 0
        self.TrackingCycleFinished = 0

        # cycle counter
        self.cycleCounter = 0

        # arrays used in the tracking cycle
        self.FocusMeasure = np.empty((0,0))
        self.Position = np.empty((0,0))
        
        # kept for class compatibility
        self.FM_slope = 0 # measured decay rate of FM with distance (Delta FM/ Delta mm)
        
        freq=[0,10]
        phase_lag=[0,0]
        self.phase_lag_funct=interpolate.interp1d(freq,phase_lag)
        self.phase_lag=self.phase_lag_funct(self.freq)
    
    def update_data(self,phase):
        self.YfocusPhase.append(phase)
        self.YfocusPosition.append(self.ampl*np.sin(phase))

    def get_error(self, focusMeasure):

        self.YfocusMeasure.append(focusMeasure)
        focusMeasure_list=np.array(self.YfocusMeasure)
        focusPosition_list=np.array(self.YfocusPosition)
        focusPhase_list=np.array(self.YfocusPhase)
        isYorder = 0

        if len(focusMeasure_list) < self.window_size:
            
            # fill the buffer first
            isYorder = 0

        else:
            
            # arduino firmware ensures that phase is between 0 and 2*pi
            # set check if a new sweep cycle has started
            if focusPhase_list[-1] < focusPhase_list[-2]:
                
                self.cycleCounter = self.cycleCounter % 2
                print('tracking cycle: ' + str(self.cycleCounter)) # @@@
                
                if self.cycleCounter == 0:
                    
                    self.TrackingCycleStarted = 1
                    self.FocusMeasure = np.empty((0,0))
                    self.Position = np.empty((0,0))
                
                elif self.cycleCounter == 1:

                    self.TrackingCycleStarted = 0
                    self.TrackingCycleFinished = 1
                
                # do nothing for the remaining cycles
                        
                self.cycleCounter = self.cycleCounter + 1
            
            # in the tracking cycle
            if self.TrackingCycleStarted:
            
                # record focus measure and position
                self.FocusMeasure = np.append(self.FocusMeasure,focusMeasure_list[-1])
                self.Position = np.append(self.Position,focusPosition_list[-1])
            
            # after the tracking cycle is finished
            if self.TrackingCycleFinished:
                
                self.TrackingCycleFinished = 0
                # now it's time to calculate focus error
                idx = np.argmax(self.FocusMeasure)
                print('Max FM: {}'.format(max(self.FocusMeasure)))
                self.error = self.Position[idx]
                isYorder = 1

        print('Y-error: {}'.format(self.error))
        return -self.error, isYorder

    #@@@
    def resize_buffers(self,buffer_size):
        self.YdequeLen=buffer_size
        self.YfocusMeasure=deque(self.YfocusMeasure,maxlen = self.YdequeLen)
        self.YfocusPosition=deque(self.YfocusPosition,maxlen = self.YdequeLen)
        self.YfocusPhase=deque(self.YfocusPhase,maxlen = self.YdequeLen)
        self.YmaxFM=deque(self.YmaxFM,maxlen = self.YdequeLen)
        
    def set_ampl(self,ampl):
        self.ampl=ampl
            
    def set_freq(self,freq):
        # print("Frequency: {}".format(freq))
        self.freq=freq
        self.phase_lag=self.phase_lag_funct(self.freq)
        
    def set_maxGain(self,gain):
        self.maxGain=gain

    def initialise_ytracking(self):
        self.YfocusMeasure = deque(maxlen = self.YdequeLen)
        self.YfocusPosition = deque(maxlen = self.YdequeLen) #in mm
        self.YfocusPhase = deque(maxlen = self.YdequeLen)
        self.TrackingCycleStarted = 0 # @@@ rename
        self.TrackingCycleFinished = 0 # @@@ rename
        self.cycleCounter = 0


############################
#           TEST           #
############################
def two_scales(ax1, time, data1, data2,label1,label2, c1, c2):

    ax2 = ax1.twinx()

    ax1.plot(time, data1, color=c1)
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel(label1)

    ax2.plot(time, data2, color=c2)
    ax2.set_ylabel(label2)
    return ax1, ax2

# Change color of each axis
def color_y_axis(ax, color):
    """Color your axes."""
    for t in ax.get_yticklabels():
        t.set_color(color)
    return None

############################
#           TEST           #
############################

if __name__ == "__main__":
	
    import matplotlib.pyplot as plt
    import csv as csv
    


    focusAmpl=0.12 #Console setting (crete to crete)
    fps_sampling=60
    focusfreq=2
    buffer_lenght=round(fps_sampling/focusfreq)
    
    yerror=[]

    ytracker=YTracker()
    ytracker.resize_buffers(buffer_lenght)
    ytracker.set_ampl(focusAmpl/2)
    ytracker.set_freq(focusfreq)
    
    path="C:/Users/Francois/Documents/11-Stage_3A/6-Code_Python/Test_for_Y8/statique_in_focus/"
    file="track.csv"
    #Test6_0_0_8mm_movTest2_0_2mm_away
    Data=[]
    reader = csv.reader(open(path+file,newline=''))
    for row in reader:
        Data.append(row)
    n=len(Data)
    Time=[float(Data[i][0]) for i in range(1,n)]             # Time stored is in milliseconds
    Xobjet=[float(Data[i][1]) for i in range(1,n)]             # Xpos in motor full-steps
    Yobjet=[float(Data[i][2]) for i in range(1,n)]             # Ypos in motor full-steps
    Zobjet=[float(Data[i][3]) for i in range(1,n)]             # Zpos is in encoder units
    ThetaWheel=[float(Data[i][4]) for i in range(1,n)]
    ZobjWheel=[float(Data[i][5]) for i in range(1,n)]
    ManualTracking=[int(Data[i][6]) for i in range(1,n)]   # 0 for auto, 1 for manual
    ImageName=[Data[i][7] for i in range(1,n)]
    focusMeasure=[float(Data[i][8]) for i in range(1,n)]
    focusPhase=[float(Data[i][9]) for i in range(1,n)]
    MaxfocusMeasure=[float(Data[i][10]) for i in range(1,n)]

    position=[ytracker.ampl*np.sin(focusPhase[i]) for i in range(len(focusPhase))]
    focusMeasure=sliding_average(focusMeasure,2)
    
    for i in range(len(focusPhase)):
        ytracker.update_data(focusPhase[i],position[i])
        yerr,isYorder,focusMeasure_buffer,new_focusMeasure,focusPosition_buffer,new_focusPosition=ytracker.get_error(focusMeasure[i])
        yerror.append(yerr)


    fig, ax = plt.subplots()
    ax1, ax2 = two_scales(ax, Time, focusMeasure, Yobjet,'focusMeasure','Yobjet', 'r', 'b')
    color_y_axis(ax1, 'r')
    color_y_axis(ax2, 'b')
    plt.title('focus measure vs position of stage')
    plt.savefig(path+"FMvsYobjet.png")
    
    fig, ax = plt.subplots()
    ax1, ax2 = two_scales(ax, Time, focusMeasure, position,'focusMeasure',"lens'Position", 'r', 'b')
    color_y_axis(ax1, 'r')
    color_y_axis(ax2, 'b')
    plt.title('focus measure vs position of the lens')
    plt.savefig(path+"FMvsYobjet.png")
    
    fig, ax = plt.subplots()
    ax1, ax2 = two_scales(ax, Time, yerror, position,'yerror',"lens'Position", 'r', 'b')
    color_y_axis(ax1, 'r')
    color_y_axis(ax2, 'b')
    plt.title('yerror vs position of the lens')
    plt.savefig(path+"YerrorvslensPos.png")  
    
    fig, ax = plt.subplots()
    ax1, ax2 = two_scales(ax, Time, yerror, Yobjet,'yerror','Yobjet', 'r', 'b')
    color_y_axis(ax1, 'r')
    color_y_axis(ax2, 'b')
    plt.title('yerror vs position of stage')
    plt.savefig(path+"yerrorvsYobjet.png")
#
#    
#    
#    #print the curve_fit result 
#    
    time1=np.linspace(0,1,len(focusPosition_buffer))
    time2=np.linspace(0,1,len(new_focusPosition))
    
    plt.figure()
    plt.plot(time1,focusMeasure_buffer,'k^:')
    plt.plot(time2,new_focusMeasure)
    plt.title('focus Measure buffer')
    plt.savefig(path+"lastFMbuffer.png")
    
    plt.figure()
    plt.plot(time1,focusPosition_buffer,'k^:')
    plt.plot(time2,new_focusPosition)
    plt.title('focus Position buffer')
    plt.savefig(path+"lastposbuffer.png")
    
    

    # Create axes
    fig, ax = plt.subplots()
    ax1, ax2 = two_scales(ax, time2, new_focusMeasure, new_focusPosition,'focusMeasure','focusPosition', 'r', 'b')
    color_y_axis(ax1, 'r')
    color_y_axis(ax2, 'b')
    plt.title('focus measure vs position on buffer')
    plt.savefig(path+"pos_FM_buffer.png")
    
    maxIndex=signal.find_peaks(new_focusMeasure,distance=len(new_focusMeasure)/3,width=9)
    
    maxIndex=maxIndex[0]
    
    
    maxList=[0 for i in range(len(new_focusMeasure))]
    for i in maxIndex:
        maxList[i]=1
    
    fig, ax = plt.subplots()
    ax1, ax2 = two_scales(ax, time2, new_focusMeasure, maxList,'focusMeasure','maxima', 'r', 'b')
    color_y_axis(ax1, 'r')
    color_y_axis(ax2, 'b')
    plt.title('focus measure vs maxima')
    plt.savefig(path+"pos_FM_buffer.png")  


    plt.show()


