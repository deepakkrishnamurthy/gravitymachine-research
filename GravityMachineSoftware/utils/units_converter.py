# -*- coding: utf-8 -*-

import numpy as np


class units_converter:

    def __init__(self):


        self.chamberWidth = 4    # Width of chamber in mm (Wheel 18)

        # Pixel per mm 10x objective
        self.pixelPermm = 1122.67
        self.imW_max = None
        # Pixel per mm 4x objective
        # pixelPermm = 456
        # --------------------------------------------------
        #  X Stepper (Linear Stage)
        # --------------------------------------------------

        self.StepsPerRev_X = 200
        self.mmPerRev_X = 1            # Pitch of the lead screw in mm

        # --------------------------------------------------
        #  Y Stepper (Focussing Stage)
        # --------------------------------------------------

        self.StepsPerRev_Y = 200
        self.mmPerRev_Y = 1 
        # StepsPerRev_Y = 20
        # mmPerStep_Y = 0.001524;     # Pitch of the lead screw in mm

        # --------------------------------------------------
        # Z stepper (Phidget Stepper) that drives the Wheel
        # --------------------------------------------------
        self.StepsPerRev_Z_motor = 200                           # No:of steps for 1 complete revolution of the motor shaft
        self.gearRatio = 99+1044/float(2057) 					# Gear ratio of Phidgets Stepper
        # Rcenter = 87.5 										# Radius to the center line of the fluidic chamber in mm (Wheel 16, 17): Ri=80 mm, Ro=95 mm
        self.Rcenter = (110 + 85)/2                                   # radius to the center-line of the fluidic chamber in mm (Wheel 18). Ri=80 mm R0= 110 mm
        # self.mmPerRev_Z = 2*np.pi*self.Rcenter          			# Displacement along the centerline of wheel in mm for 1 revolution of the output shaft
        self.StepsPerRev_Z = self.gearRatio * self.StepsPerRev_Z_motor            # No:of steps of the main motor shaft for 1 Rev of the output shaft

        # --------------------------------------------------
        # X encoder (linear)
        # --------------------------------------------------
        self.CountPermm_X = 500 # 1um per count RLS miniature linear encoder
        # --------------------------------------------------
        # Y encoder (linear)
        # --------------------------------------------------
        self.CountPermm_Y = 500 # 1um per count RLS miniature linear encoder
        # --------------------------------------------------
        # Z encoder
        # --------------------------------------------------
        self.CountsPerRev_Zenc = 600
        self.CountsPerRev_Z = self.CountsPerRev_Zenc*self.gearRatio
        # --------------------------------------------------
        # Distance in mm between the center of the Wheel and the origin of Arduino's Xpos
        # --------------------------------------------------
        self.DeltaX_Arduino_mm = 99.325 # Measured value for GM v2.0 setup (Berg)

        # --------------------------------------------------
        # Distance in mm between the front wall(adajancent to the fluid, nearest to camera)  and the origin of Arduino's Ypos
        # --------------------------------------------------
        self.DeltaY_Arduino_mm = 0 # 10x Y offset so that the chamber wall (adjacent to the fluid) closest to the 
        # self.DeltaY_Arduino_mm = -2.18 # 4x objective
    # --------------------------------------------------
    # Functions
    # --------------------------------------------------
    def set_max_imWidth(self, imW):
        self.imW_max = imW

    def update_pixel_size(self, new_pixelPermm):
        self.pixelPermm = new_pixelPermm

    def px_to_mm(self, Dist,resolution_width):
        return 1./self.pixelPermm/(resolution_width/self.imW_max)*Dist   

    def mm_to_px(self, Dist,resolution_width):
        return Dist*self.pixelPermm*resolution_width/self.imW_max

    #---------------------------------------------------

    def X_mm_to_step(self, Xmm):
        Xstep = Xmm/self.mmPerRev_X*self.StepsPerRev_X
        return Xstep

    def Y_mm_to_step(self, Ymm):
        Ystep = Ymm/self.mmPerRev_Y*self.StepsPerRev_Y
        return Ystep

    def mmPerRev_Z(self, Xpos_mm):                            #Xpos_mm position in the centerlign of the fluid channel's referenciel
        return 2*np.pi*(self.Rcenter)#+Xpos_mm)


    def Z_mm_to_step(self, Zmm, Xpos_mm):
        Zstep = Zmm/self.mmPerRev_Z(Xpos_mm)*self.StepsPerRev_Z
        return Zstep

    #---------------------------------------------------
    def X_count_to_mm(self, Xstep):
        Xmm = Xstep/self.CountPermm_X
        return Xmm

    def Y_count_to_mm(self, Ystep):
        Ymm = Ystep/self.CountPermm_Y
        return Ymm
     
    def X_microstep_to_mm(self, Xstep):                #Arduino card send the data for X and Y in Microstep
        Xmm = Xstep*self.mmPerRev_X/(self.StepsPerRev_X*16)
        return Xmm

    def Y_microstep_to_mm(self, Ystep):
        Ymm=Ystep*self.mmPerRev_Y/(self.StepsPerRev_Y*16)
        return Ymm

    def Z_step_to_mm(self, Zstep, Xpos_mm):
        Zmm=Zstep*mmPerRev_Z(Xpos_mm)/self.StepsPerRev_Z
        return Zmm

    #---------------------------------------------------
    #Give the absolute position of the image in the referentiel of the centerline of the flow channel
    def X_arduino_to_mm(self, Xarduino):
        Xmm = self.X_count_to_mm(Xarduino)
        Xpos_mm = Xmm + self.DeltaX_Arduino_mm - self.Rcenter
        return Xpos_mm

    def Y_arduino_to_mm(self, Yarduino):
        Ymm = self.Y_count_to_mm(Yarduino)
        Ypos_mm = Ymm + self.DeltaY_Arduino_mm
        
        return Ypos_mm

    def theta_arduino_to_rad(self, Zarduino):
        theta = Zarduino/self.CountsPerRev_Z*2*np.pi       # 2018-09-01 Major correcton. We are using encoder counts for the Z position so this should be EncoderCounts and not Stepper Motor pulses
        return theta

    def rad_to_mm(self, ThetaWheel,Xobjet):
        return ThetaWheel*(self.Rcenter+Xobjet)          # 2018_09_01: by Deepak. Note major Error previously the radian value was divided by 2*pi which makes the calculation of distance incorrect. 
        #ThetaWheel is already in radians so there should be no dividing 2*pi factor. Have checked by manually rotating the wheel that this now corresponds to the actual physical distance. To completely confirm will run calibration experiments again.