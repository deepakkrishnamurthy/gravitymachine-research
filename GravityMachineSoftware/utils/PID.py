# -*- coding: utf-8 -*-
"""
Created on Thu May 17 12:17:20 2018

@author: Francois
"""

#Tuning the parameters:
    #Integrator max(resp .min): max nb of steps the motor can make in DeltaT

class PID:
    def __init__(self, P=0.5, I=0, D=0.5,Output_max=100,Output_min=-100):
        
        #Parameters
        self.Kp=P
        self.Ki=I
        self.Kd=D
        
        self.Output_max=Output_max
        self.Output_min=Output_min
        
        self.isDirect=True
        
        #current values
        self.error=0.0
        self.Integrator=0
        self.D_value=0
        
        #previous values
        self.previousError=0
        self.previousTime=0
        self.previousOutput=0
        
        
    def initiate(self,error,time):
        self.previousTime=time
        self.previousError=error
        self.Integrator=self.Ki*self.previousOutput

        
        
    def update(self,error,time): #then you can call 
        """
        Calculate PID output value for given reference input and feedback
        """
        
        self.error = error #self.set_point - current_value

        self.P_value = self.Kp * self.error
        # self.D_value = 0.7*self.D_value+0.3*self.Kd * ( self.error - self.previousError)/(time-self.previousTime)
        self.D_value = self.Kd * ( self.error - self.previousError)/(time-self.previousTime)
        self.previousError = self.error
        

        self.Integrator = self.Integrator + self.Ki*self.error*(time-self.previousTime)

        self.previousTime=time
        
        if self.Integrator > self.Output_max:
            self.Integrator = self.Output_max
        elif self.Integrator < self.Output_min:
            self.Integrator = self.Output_min

        self.I_value = self.Integrator

        PID = self.P_value + self.I_value + self.D_value
        
        if PID > self.Output_max:
            PID = self.Output_max
        elif PID < self.Output_min:
            PID = self.Output_min
        
        self.previousOutput=PID

        return PID

    def set_Integrator(self, Integrator):
        self.Integrator = Integrator

    def set_previousError(self, previousError):
        self.previousError = previousError

    def set_Tuning(self,P,I,D):
        
        
        a=1
        #if not(self.isDirect):
        #    a=-1

        if (P>=0):
            self.Kp=a*P
            print('P',P)
        if (I>=0):
            self.Ki=a*I
            print('I',I)
        if (D>=0):
            self.Kd=a*D
            print('D',D)
        
    def set_isDirect(self,boolean):
        self.isDirect=boolean

    def get_previousError(self):
        return self.Derivator
    
    def get_Error(self):
        return self.error

    def get_Integrator(self):
        return self.Integrator

