# -*- coding: utf-8 -*-
"""
Created on Thu May 17 12:17:20 2018

@author: Francois
"""

#Tuning the parameters:
    #Integrator max(resp .min): max nb of steps the motor can make in DeltaT

from collections import deque
import scipy.optimize as opt
import scipy.interpolate as interpolate
import scipy.signal as signal
import numpy as np


#the sliding average has to be centered otherwise it induces an additional phase lag
def sliding_average(data,n):
    new_data=[]
    if len(data)<=2*n:
        new_data=data
    else:        
        for j in range(0,n):
            y=data[j]
            for i in range(1,j+1):
                y+=data[j+i]+data[j-i]
            new_data.append(y/(2*j+1))
        for j in range(n,len(data)-n):
            y=data[j]
            for i in range(1,n+1):
                y+=data[j-i]+data[j+i]
            new_data.append(y/(2*n+1))
        for j in range(len(data)-n,len(data)):
            y=data[j]
            for i in range(1,len(data)-j):
                y+=data[j+i]+data[j-i]
            new_data.append(y/(2*(len(data)-1-j)+1))
            
    return new_data

        
    
class YTracker():
    
    def __init__(self,parent=None):
        
        self.YdequeLen=50
        self.YfocusMeasure = deque(maxlen = self.YdequeLen)  #The length of the buffer must be ~fps_sampling/liquid_lens_freq
        self.YfocusPosition = deque(maxlen = self.YdequeLen) #in mm
        self.YfocusPhase = deque(maxlen = self.YdequeLen)
        
        #in order to auto tune the gain 
        # Initialize as 0 to avoid indexing issues later
        self.YmaxFM = deque([0],maxlen=10*self.YdequeLen) #store the value of the focus measure in order to tune the gain
        self.gain=1
        self.maxGain=1

        self.error = 0

        self.error_sign = 0
        
        self.ampl = 0.025
        self.freq = 2

        self.window_size = 25

        self.FM_slope = 1250 # measured decay rate of FM with distance (Delta FM/ Delta mm)
        #-----------------------------------------------------------------------------------------
        # These are liquid lens variables and should ideally be contained within the liquid lens class
        #-----------------------------------------------------------------------------------------
        #freq,ampl,phase_lag=[1,2],[0.2,0.2],[6./100*2*np.pi,6./100*2*np.pi]
        freq=[0.2,8,9,10,13.92,12.001,13,13.99,15,0.4999,0.9976,2.0002,3.0000,4.0001,5,6,6.9995]
        phase_lag=[0.067,0.9173,0.9738,1.0315,1.0766,1.1163,1.1367,1.1812,1.1957,0.1335,0.2188,0.3655,0.5047,0.6294,0.7103,0.7759,0.8456]
        # self.phase_lag_funct = interpolate.interp1d(freq,phase_lag)
        self.phase_lag_funct = lambda freq: 0
        # self.phase_lag=self.phase_lag_funct(self.freq)
        self.phase_lag = 0
        #-----------------------------------------------------------------------------------------
        
        self.count_between_peaks=0
        self.lastPeakIndex=0
        
    def update_data(self,phase):
        self.YfocusPhase.append(phase)
        # We are not fitting a sine curve anymore. 
        # print('Phase lag: {}'.format(self.phase_lag))
        self.YfocusPosition.append(self.ampl*np.sin(phase + np.pi/2.0))
        # self.YfocusPosition.append(position)
       

    # 2019-05-22: New algorithm for calculating the error based on local slope of Focus-measure vs position

    def get_error(self, focusMeasure):

        self.YfocusMeasure.append(focusMeasure)

        focusMeasure_list=np.array(self.YfocusMeasure)        

        focusPosition_list=np.array(self.YfocusPosition)


        if len(focusMeasure_list)>=self.window_size:

            focusMeasure_list = focusMeasure_list[-self.window_size:]
            focusPosition_list = focusPosition_list[-self.window_size:]


            if(self.ampl!=0):
                p = np.polyfit(focusPosition_list, focusMeasure_list, deg = 1)

                # self.error = self.gain*p[0]/self.FM_slope
                self.error = p[0]
            else:
                self.error = 0

            

            # if(p[0]<1e-6):
            #     self.error = 0
            # else:
            #     self.error = p[1]/(2*p[0])


            # print('1st Derivative: {}'.format(p[1]))
            # print('1st Derivative: {}'.format(p[0]))

            self.YmaxFM.append(np.nanmean(focusMeasure_list))

            isYorder = 1

            self.update_gain()

        else:

            self.error_sign = 0
            isYorder = 0

        # print('Y-error: {}'.format(self.error))
        return -self.error, isYorder
        
    # def get_error(self,focusMeasure):
    #     self.YfocusMeasure.append(focusMeasure)
        
    #     focusMeasure_list=list(self.YfocusMeasure)
    #     focusMeasure_list=sliding_average(focusMeasure_list,2)
        
    #     focusPosition_list=list(self.YfocusPosition)
        
    #     Yerror=0
    #     isYorder=0
        
    #     if len(focusMeasure_list)==self.YdequeLen:
            
    #         try:
                
    #             new_time,new_focusMeasure=self.smooth_FM(focusMeasure_list)
    #             new_time,new_focusPosition=self.smooth_position(focusPosition_list)
                
    #         except Exception:
    #             print(Exception)
    #             new_focusMeasure=np.array(focusMeasure_list)
    #             new_focusPosition=np.array(focusPosition_list)
    #             pass

            
    #         #Finding the new peak
    #         dist=len(new_focusMeasure)/3
    #         FMmaxIndex = signal.find_peaks(new_focusMeasure,distance=dist,width=9)#return the index and other stats
    #         # We only retain the peak indices and not their properties.
    #         FMmaxIndex=FMmaxIndex[0]
            
    
    #         self.count_between_peaks+=3 #the interpolation give three time more point
                

    #         if len(FMmaxIndex)>0:
    #             maxIndex=FMmaxIndex[-1] #we take the last peak
    #             if (abs(self.lastPeakIndex-self.count_between_peaks-maxIndex)>2):
    #                 self.lastPeakIndex=maxIndex
    #                 self.count_between_peaks=0
    #                 Yerror=new_focusPosition[maxIndex]
    #                 self.YmaxFM.append(new_focusMeasure[maxIndex])
    #                 isYorder=1
    #                 self.update_gain()
            
    #     else: #if the buffer is not full / for testing / not usefull
    #         new_focusMeasure=np.array(focusMeasure_list)
    #         new_focusPosition=np.array(focusPosition_list)
            
    #     return -self.gain*Yerror,isYorder#,focusMeasure_list,new_focusMeasure,focusPosition_list,new_focusPosition

    def update_gain(self):
        # if len(self.YmaxFM)>self.YdequeLen:
            
        self.gain = (max(self.YmaxFM)-self.YmaxFM[-1]+1)/(max(self.YmaxFM)-min(self.YmaxFM)+1)

        if (self.gain < 0.001):
            self.gain = 0

        # print('Gain: {}'.format(self.gain))

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
        # self.phase_lag=self.phase_lag_funct(self.freq)
        self.phase_lag = 0

        
    def set_maxGain(self,gain):
        self.maxGain=gain

    def opt_function_pos(self,x,freq,phi,offset):
        return self.ampl*np.sin(2*np.pi*freq*x+phi)+offset

        # Note: Having an offset is necessary as a fitting parameter since the part of the sine curve in the deque can start at any phase


        
    def smooth_position(self,data):
        
        time=np.linspace(0,1,len(data))
        data_param, data_stat=opt.curve_fit(self.opt_function_pos,time,data,[1.5,np.pi,0],bounds=((1,-np.inf,-np.inf),(2,np.inf,np.inf)))
        
        # Print out the fitted parameters
        # print("Frequency: {} Hz \n".format(data_param[0]))
        # print("Phase difference: {} \n".format(data_param[1]))
        # print("Offset: {} \n".format(data_param[2]))

        new_time=np.linspace(0,1,len(data)*3)
        # Based on my experiments, the phase-lag needs to be added and not subtracted from the phase to get the correct position
        new_data=self.opt_function_pos(new_time,data_param[0],data_param[1]-self.phase_lag,data_param[2])
        
        # new_data=self.opt_function_pos(new_time,data_param[0],data_param[1] + self.phase_lag,data_param[2])


        return new_time,new_data
    
    
    def smooth_FM(self,data):
        time=np.linspace(0,1,len(data))
        f = interpolate.interp1d(time, data)
        new_time=np.linspace(0,1,len(data)*3)
        new_data=f(new_time)
        
        return new_time,new_data
    
    def smooth_FM_analysis(self,data):
        time=np.linspace(0,1,len(data))
        f = interpolate.interp1d(time, data)
        new_time=np.linspace(0,1,len(data))
        new_data=f(new_time)
        
        return new_time,new_data

    def initialise_ytracking(self):
        self.YfocusMeasure = deque(maxlen = self.YdequeLen)  #The lenght of the buffer must be ~fps_sampling/liquid_lens_freq
        self.YfocusPosition = deque(maxlen = self.YdequeLen) #in mm
        self.YfocusPhase = deque(maxlen = self.YdequeLen)
        
        #in order to auto tune the gain
        self.YmaxFM = deque([0],maxlen=10*self.YdequeLen) #stock the value of the focus mesure in order to tune the gain
        self.gain=1
        self.count_between_peaks=0
        self.lastPeakIndex=0
    
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


