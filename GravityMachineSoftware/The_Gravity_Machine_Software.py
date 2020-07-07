# -*- coding: utf-8 -*-


import sys
import os
from os.path import realpath, dirname, join

import cv2
import numpy as np

from pyqtgraph.Qt import QtWidgets,QtCore, QtGui #possible to import form PyQt5 too ... what's the difference? speed? 
import pyqtgraph as pg
#import pyqtgraph.ptime as ptime
import pyqtgraph.dockarea as dock
from pyqtgraph.dockarea.Dock import DockLabel


from slickpicker.slickpicker import QColorEdit
from utils import rangeslider as rangeslider

import utils.image_enhancement as image_enhancement
import utils.image_processing as image_processing
# import utils.units_converter as units_converter
import utils.units_converter as units_converter
import utils.PID as PID

import utils.YTracking_nonPID as YTracking
from LiquidLens_Tool import optotune_lens

import CSV_Tool
import Arduino_Tool

from aqua.qsshelper import QSSHelper
import utils.dockareaStyle as dstyle

from collections import deque
from queue import Queue
				   

from camera.VideoStream import VideoStream as VideoStream
from camera.ImageSaver import ImageSaver as ImageSaver

import time
import imutils

import glob, torch 

from DaSiamRPN.code.net import SiamRPNvot
from DaSiamRPN.code import vot 
from DaSiamRPN.code.run_SiamRPN import SiamRPN_init, SiamRPN_track
from DaSiamRPN.code.utils import get_axis_aligned_bbox, cxy_wh_2_rect

from functools import partial

from datetime import datetime

import pandas as pd


# 2018_12_27
# Test change to check git commits
	
'''       
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                          Object Tracking
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

class Object_Tracking(QtCore.QObject):
	
	centroid_glob = QtCore.pyqtSignal(np.ndarray)
	Rect_pt1_pt2 = QtCore.pyqtSignal(np.ndarray)
	plot_data = QtCore.pyqtSignal(np.ndarray)
	set_trackBusy = QtCore.pyqtSignal()
	clear_trackBusy = QtCore.pyqtSignal()

	def __init__(self, central_widget, color = False, parent=None):
		super().__init__(parent)
		
		self.central_widget = central_widget
		# Variables that are stored in CSV file
		Image_name_headers = {key:[] for key in self.central_widget.imaging_channels}
		Image_name_headers[self.central_widget.tracking_channel] = 'Image name'

		for channel in self.central_widget.imaging_channels:
			if(channel is not self.central_widget.tracking_channel):
				Image_name_headers[channel] = '{} Image name'.format(channel)

		
		self.header = [['Time', 'Xobj','Yobj','Xobj_image','Zobj','ThetaWheel','ZobjWheel',
			'Manual Tracking','Focus Measure','Liquid Lens Phase','Liquid Lens Freq','Liquid Lens Ampl',
			'Liquid Lens maxGain','Y FM maximum','Light Experiment', 'LED_Intensity','Lens position'] 
			+ [Image_name_headers[key] for key in self.central_widget.imaging_channels]]
		
		#CSV register
		self.csv_register = CSV_Tool.CSV_Register(header = self.header)

		# Flag for Color vs Grayscale images for the input data
		self.color = color
		
		# load net
		print('Loading Net ...')
		self.net = SiamRPNvot()
		self.net.load_state_dict(torch.load(join(realpath(dirname(__file__)), 'DaSiamRPN','code','SiamRPNOTB.model')))
		self.net.eval().cuda()
		print('Finished loading net ...')

		#Arduino controller for the wheel
		self.isWheelConnected=True
		if self.isWheelConnected:        
			self.arduino_wheel=Arduino_Tool.Arduino_Wheel()
		else:
			self.arduino_wheel=None
			
		#Arduino Controller for the LED Panel  
		self.isLEDPanelConnected=False
		if self.isLEDPanelConnected:
			self.arduino_led_panel=Arduino_Tool.Arduino_LED_Panel()
		else:
			self.arduino_led_panel=True
			
		#counter and flag
		self.start_saving=False
		self.start_tracking=False      #tracking activation / computer side
		self.start_Y_tracking=False
		self.light_experiment = False		# Flag to start the light intesity modulation experiments

		self.current_image_name = {key:'' for key in self.central_widget.imaging_channels}     
		#name of the current image being processing when start_saving=True
		self.manualMode=1              #tracking activation / Arduino side:  manualMode=0 ifthe computer control the wheel
		self.distractionFreeMode = 1   #distractionFreeMode=1 if a crop image is use to track the particule when she has already been flagged
		self.globCounter=0             #number of frame an object is tracked for a track period
		self.trackGapCounter = 0;      #Keeps track of nb of consecutive frames when object is not tracked
		self.flag=0                    #flag=1 if a centroid is detected on the current frame
		self.SpeedBasedfactor=1        #0<SpeedBasedfactor<1: 1:pure position tracking, 0: pure speed tracking
		self.trackCounter = 0			# Counter that is updated each time we start a new track (start_tracking button)

		# OpenCV tracking suite
		self.OPENCV_OBJECT_TRACKERS = {
		"csrt": cv2.TrackerCSRT_create,
		"kcf": cv2.TrackerKCF_create,
		"boosting": cv2.TrackerBoosting_create,
		"mil": cv2.TrackerMIL_create,
		"tld": cv2.TrackerTLD_create,
		"medianflow": cv2.TrackerMedianFlow_create,
		"mosse": cv2.TrackerMOSSE_create
		}

		self.NeuralNetTrackers = {"daSiamRPN":[]}



		# Choose the default tracker type
		# Changing the tracker to check git commits
		self.tracker_type = "csrt"

		self.bbox = []
		self.prev_bbox = []
		
		self.createTracker()


		#image geometrical data (depending on the resolution)
		self.cropRatio=10.
		self.image_width=480           #width of the thresh image received
		self.image_center=False        #np.array size 2
		self.cropSize=48
		
		#data to keep
		self.dequeLen = 20
		
		#Time
		self.begining_Time = 0           #Time begin the first time we click on the start_tracking button
		self.Time = deque(maxlen=self.dequeLen)
		self.averageDt=0
		
		#Total angle traveled by the wheel (unit=rad)
		self.ThetaWheel=deque(maxlen=self.dequeLen) #trigonometric sens
		
		#Location of the left corner of the cropped image in the Image referenciel (unit=px)
		self.origLoc=np.array((0,0))   
		
		#Object position in Image referenciel (unit=px)
		self.centroids = deque(maxlen=self.dequeLen) # Create a buffer to store the tracked object centroids
		
		#Image center position in the centerline referentiel (unit=mm)
		self.Ximage=deque(maxlen=self.dequeLen)            #distance to the centerline of the flow channel
		self.Yimage=deque(maxlen=self.dequeLen)            #distance to the centerline of the flow channel
		self.Zimage=deque(maxlen=self.dequeLen)            #distance to the centerline of the flow channel
		
		# Distance of object from the image-center (or more generally the origin of the tracking point)
		self.Xobj_image = deque(maxlen=self.dequeLen)
		self.Zobj_image = deque(maxlen=self.dequeLen)

		
		#Object position in lab reference frame (unit=mm)
		self.Xobj=deque(maxlen=self.dequeLen)            #distance to the centerline of the flow channel
		self.Yobj=deque(maxlen=self.dequeLen)            #distance to the centerline of the flow channel
		self.Zobj=deque(maxlen=self.dequeLen)            #distance to the centerline of the flow channel
	   
		self.ZobjWheel=deque(maxlen=self.dequeLen)         #Object's Z position relative to a frame of reference rotating with the stage
		self.VobjWheel=0                                   #object's speed in the wheel referential
		
		self.ZobjWheel_init = 0							# Stores the starting virtual depth for doing environmental patterning experiments

		self.relDepth = 0								# Depth relative to the start of the track

		#Y-Tracking parameters and data
		self.ytracker=YTracking.YTracker()       #We use a class in a separate folder to handle all the YTracking
		self.Yerror=0                            #distance in mm to the optimal position

		self.liquid_lens_freq = 2	# Frequency in Hz
		self.liquid_lens_ampl = 0.025	# Lens amplitude in mm
		# self.liquid_lens_offset = 39.5 # lens offset 
		self.liquid_lens_offset = 0 # lens offset in mm 


		# Initiate the Liquid Lens controller
		self.liquid_lens = optotune_lens(freq = self.liquid_lens_freq, amp = self.liquid_lens_ampl, offset = self.liquid_lens_offset)

		
		self.prev_Y_order_time=0
		
		#PID
		self.pid_z=PID.PID()
		self.pid_x=PID.PID()
		self.pid_y=PID.PID()
		
		#threshold parameters
		# self.lower_HSV=np.array([0,0,0],dtype='uint8') 
		# self.upper_HSV=np.array([178,255,255],dtype='uint8')
		
		#color LED panel
		self.LEDpanel_color=[0,0,0]

		self.LED_intensity = 0 # Number between 0 and 255 to control LED intensity
		self.maxLEDIntensity = 255
		self.minLEDIntensity = 0
		self.LED_intensity_measured = 0

		self.threshDepth = 20  # Depth/Height after which LED turns ON in mm

		self.surface_intensity = 255
		self.surface_Z = 100			# For light modulation experiments assume the water surface is at 100 mm 

		self.K = 1e-2					# Vertical extinction coefficient in mm^-1

		# Offset for the image center (future functionality for changing the image center dynamically)
		self.imageCenterOffset = np.array([0,0], dtype = 'int')

		self.tracking_triggered_prev = False
		
		
		
	def track(self,image_data): #receive the new image from the camera (emit(data)) #color tracking or not will be decided by the threshold image received
	# This receives the full image from the camera_functions class	
		
		self.set_trackBusy.emit()
		time_tick = time.time()
		#@@@ print("track_busy set")
		
		#receive data from arduino_Wheel
		arduino_data,manualMode = self.get_img_position() #We capt the exact position and wait to know if we could store it or not

		tracking_triggered = arduino_data[5]
		if tracking_triggered and tracking_triggered != self.tracking_triggered_prev:
			self.central_widget.button_tracking.toggle()
			self.central_widget.start_tracking()
		self.tracking_triggered_prev = tracking_triggered

		if self.start_tracking:

			# print('Using tracker: {}'.format(self.tracker_type))

			#initial parameters
			shouldInitiatePID=False
			self.update_img_param(image_data) #update image width / cropSize / image_center
			isCropped=False
			#------------------------------------------------------------------------------------
			# Tracking using the OpenCV single-object tracking suite
			#------------------------------------------------------------------------------------

			# When we are already tracking an object. Centroid detected in previous step.
			if(self.globCounter != 0 and self.flag==1):	
				

				# If we are using OPENCV based tracker
				if(self.tracker_type in self.OPENCV_OBJECT_TRACKERS.keys()):
				# Update the tracker 
					print(self.tracker_type)
					ok, self.bbox = self.tracker.update(image_data)

					# If we are using OpenCV trackers, we don't need to crop the image.
					isCropped=False
					# print('Using tracker: {}'.format(self.tracker_type))

					# If Tracking was successful
					if(ok):
						# print('Tracking success!')
						isCentroidFound = True
						self.prev_bbox = self.bbox
						pts = np.array([[self.bbox[0],self.bbox[1]],[self.bbox[0]+self.bbox[2],self.bbox[1]+self.bbox[3]]], dtype = 'int')

						# This is the bounding box from the tracker
						# self.Rect_pt1_pt2.emit(pts)
						# self.origLoc = pts[0]
						
						# print(self.origLoc)

						# Coordinates of the object centroid are taken as the center of the bounding box
						# Note: Could make this more sophisticated in the future.
						cx = int(self.bbox[0] + self.bbox[2]/2)
						cy = int(self.bbox[1] + self.bbox[3]/2)

						centroidLoc = np.array([cx, cy])

					else:
						print('Tracking failure')
						isCentroidFound = False
						pts = np.array([[self.prev_bbox[0],self.prev_bbox[1]],[self.prev_bbox[0]+self.prev_bbox[2],self.prev_bbox[1]+self.prev_bbox[3]]], dtype = 'int')

						self.Rect_pt1_pt2.emit(pts)


						# NeuralNet based tracker
				elif(self.tracker_type in self.NeuralNetTrackers.keys()):
						
					self.state = SiamRPN_track(self.state, image_data)  # track
						# If we are using NeuralNet trackers, we don't need to crop the image.
					isCropped=False

					# if(self.state is not None):
					# 	ok = True
					# else:
					# 	ok = False
					ok = True
						# If Tracking was successful

					print(self.state['score'])

					if(ok):
						print('Tracking success!')
						isCentroidFound = True

						self.bbox = cxy_wh_2_rect(self.state['target_pos'], self.state['target_sz'])

						self.bbox = [int(l) for l in self.bbox]


						self.prev_bbox = self.bbox
						pts = np.array([[self.bbox[0],self.bbox[1]],[self.bbox[0]+self.bbox[2],self.bbox[1]+self.bbox[3]]], dtype = 'int')

						# This is the bounding box from the tracker
						# self.Rect_pt1_pt2.emit(pts)
						# self.origLoc = pts[0]
						
						

						# Coordinates of the object centroid are taken as the center of the bounding box
						# Note: Could make this more sophisticated in the future.
						cx = int(self.bbox[0] + self.bbox[2]/2)
						cy = int(self.bbox[1] + self.bbox[3]/2)

						centroidLoc = np.array([cx, cy])
						print('Centroid loc:{}'.format(centroidLoc))

					else:
						print('Tracking failure')
						isCentroidFound = False
						pts = np.array([[self.prev_bbox[0],self.prev_bbox[1]],[self.prev_bbox[0]+self.prev_bbox[2],self.prev_bbox[1]+self.prev_bbox[3]]], dtype = 'int')

						self.Rect_pt1_pt2.emit(pts)

				# Nearest neighbhour tracking method			
				else:
					isCropped=True
					image_data.shape
					# We only crop the image for the nearest neighbhour tracking
					pts,image_data=image_processing.crop(image_data, self.centroids[-1],self.cropSize)
					self.Rect_pt1_pt2.emit(pts)
					self.origLoc = pts[0]
					# thresh_image = image_processing.threshold_image(image_data,self.lower_HSV,self.upper_HSV)

					isCentroidFound,centroidLoc = image_processing.find_centroid_basic(self.thresh_image) 



			# If we are starting to track an object and it has not been tracked previously.
			else:
				self.origLoc = np.array([0,0])
				# thresh_image=image_processing.threshold_image(image_data,self.lower_HSV,self.upper_HSV)
				
				print('Trying to find a new object...')
				#print(self.tracker_type)
				# If we are using OPENCV based tracker
				if(self.tracker_type in self.OPENCV_OBJECT_TRACKERS.keys()):
					
					isCentroidFound,centroidLoc, self.bbox = image_processing.find_centroid_basic_Rect(self.thresh_image)
					
					self.prev_bbox = self.bbox
					
					# Initialize the tracker with the detected object and bounding box
					if(isCentroidFound):
						print('Centroid found! Starting tracker')
						print(self.bbox)
						self.tracker.init(image_data, self.bbox)
					else:
						print('No centroid found!')

						# NeuralNet based tracker
				elif(self.tracker_type in self.NeuralNetTrackers.keys()):

					print('Trying to find a new object...')
					isCentroidFound,centroidLoc, self.bbox = image_processing.find_centroid_basic_Rect(self.thresh_image)
					self.prev_bbox = self.bbox
					# Initialize the tracker with the detected object and bounding box
					if(isCentroidFound):
						print('Centroid found! Starting tracker')
						print(self.bbox)

						# Initialize the tracker with this centroid position
						target_pos, target_sz = np.array([centroidLoc[0], centroidLoc[1]]), np.array([self.bbox[2], self.bbox[3]])

						self.state = SiamRPN_init(image_data, target_pos, target_sz, self.net)
				
				# Nearest neighbour tracking method
				else:
					isCentroidFound,centroidLoc = image_processing.find_centroid_basic(self.thresh_image)

			#Then if a centroid is found we communicate with the arduino and update image position
			if isCentroidFound:  # centroid=False if no object detected
				# #receive data from arduino_Wheel
				# arduino_data,manualMode=self.get_img_position() #We capt the exact position and wait to know if we could store it or not
				if ((self.manualMode==1 and manualMode==0) or len(self.Time)<=1 or self.trackGapCounter>5):
					shouldInitiatePID=True
				 
				self.trackGapCounter=0
				self.flag=1
				self.globCounter+=1
				self.manualMode=manualMode

				centroidGlob = self.origLoc + centroidLoc	# centroid location in the global image reference frame
				self.centroids.append(centroidGlob)
				self.centroid_glob.emit(centroidGlob)  #send the centroid position in order to draw the cercle on the screen
				if (not isCropped):                     #If the image hasn't been cropped already we do it to allow Y-tracking
					pts,image_data = image_processing.crop(image_data, self.centroids[-1],self.cropSize)
					self.Rect_pt1_pt2.emit(pts)

				self.update_img_position(arduino_data)
				self.get_Yerror(image_data)  #update self.Yerror. Need to be after "self.update_img_position"
				self.update_object_position(self.image_center,centroidGlob)  
				
				if(self.start_Y_tracking):
					self.plot_data.emit(np.array([self.Time[-1],self.Xobj[-1],self.Yobj[-1],self.Zobj[-1],self.ZobjWheel[-1],self.ThetaWheel[-1], self.ytracker.YfocusMeasure[-1], self.Yerror, self.ytracker.YfocusPhase[-1]]))                  #now we have the position of the centroid
				else:
					self.plot_data.emit(np.array([self.Time[-1],self.Xobj[-1],self.Yobj[-1],self.Zobj[-1],self.ZobjWheel[-1],self.ThetaWheel[-1], 0, 0, 0]))                   #now we have the position of the centroid


				if (self.isWheelConnected):
					#order corresponds to [liquid_lens param,homing,tracking,Xstep,Ystep,Zstep,DeltaT]
					# This is the order we then send to the arduino
					orders=self.getOrder(self.image_center,centroidGlob,shouldInitiatePID)
					self.arduino_wheel.send_to_Arduino(orders)      #send order thanks to the Arduino class
				if self.start_saving:
					self.register_data()
					
			else:
				
				self.flag=0
				self.trackGapCounter+=1
				self.globCounter=0
		

		self.clear_trackBusy.emit()
		#@@@ print("track_busy cleared")
		# print("time spent in track() including reading from Arduino: "+ str(time.time()-time_tick))
		# print("-------")
	
	def returnThresholdImage(self, thresh_img):
		self.thresh_image = thresh_img


	def createTracker(self):
		if(self.tracker_type in self.OPENCV_OBJECT_TRACKERS.keys()):
			self.tracker = self.OPENCV_OBJECT_TRACKERS[self.tracker_type]()

		elif(self.tracker_type in self.NeuralNetTrackers.keys()):
			
			
			print('Using {} tracker'.format(self.tracker_type))



	def update_img_param(self,image):
		
		# self.image_center, self.image_width = image_processing.get_image_top_center_width(image)
		self.image_center,self.image_width = image_processing.get_image_center_width(image)
		self.cropSize = round(self.image_width/self.cropRatio)

	def get_img_position(self):
		# We now read 5 things from the Arduino including the measured light intensity
		arduino_data,manualMode=[0 for i in range(5)],1
		if self.isWheelConnected:
			arduino_data,manualMode=self.arduino_wheel.read_from_arduino()
		return arduino_data,manualMode
	
	def update_img_position(self,arduino_data):
		
		if self.isWheelConnected:

			[YfocusPhase,Xpos_arduino,Ypos_arduino,Zpos_arduino, self.LED_intensity_measured, tracking_triggered] = arduino_data

			#YfocusPosition=self.liquid_lens_ampl*np.sin(YfocusPhase)
			self.ytracker.update_data(YfocusPhase)

			self.Ximage.append(self.central_widget.units_converter.X_arduino_to_mm(Xpos_arduino))#We invert the postion of the motor
			self.Yimage.append(self.central_widget.units_converter.Y_arduino_to_mm(Ypos_arduino))
			self.Zimage.append(0) #not nul when we will have a Z-axis tracking
			self.Time.append(time.time()-self.begining_Time) #The time is based on the computer, not on the arduino
			# self.ThetaWheel.append(units_converter.X_arduino_to_mm(Zpos_arduino)) #Attention, the arduino count in the anti trigo sens
			self.ThetaWheel.append(-self.central_widget.units_converter.theta_arduino_to_rad(Zpos_arduino)) #Attention, the arduino count in the anti trigo sens
			

			
		else:
			self.Ximage.append(0)
			self.Yimage.append(0)
			self.Zimage.append(0)
			self.Time.append(time.time()-self.begining_Time)
			self.ThetaWheel.append(0)
		
		#update average DeltaT (Delta=0 when len(T)=1)
		self.update_averageDt()
		

	def update_object_position(self,imageCenter,centroidGlob):
		#to see if we take into account of not the phase

		self.Xobj_image.append(self.central_widget.units_converter.px_to_mm(centroidGlob[0]- imageCenter[0] + self.imageCenterOffset[0],self.image_width))
		self.Zobj_image.append(-self.central_widget.units_converter.px_to_mm(centroidGlob[1]- imageCenter[1] + self.imageCenterOffset[1],self.image_width))

		self.Xobj.append(self.Ximage[-1] + self.Xobj_image[-1])
		self.Yobj.append(self.Yimage[-1])
		self.Zobj.append(self.Zimage[-1] + self.Zobj_image[-1])

		if len(self.Time)>1:
			self.ZobjWheel.append(self.ZobjWheel[-1]+(self.Zobj[-1]-self.Zobj[-2])-self.central_widget.units_converter.rad_to_mm(self.ThetaWheel[-1]-self.ThetaWheel[-2],self.Xobj[-1]))
		else:
			self.ZobjWheel.append(0)


		#update image speed
		self.update_VobjZWheel()

		#print('Velocity of the object (mm/s):',self.VobjWheel)
		
	def get_Yerror(self,cropped_image):
		self.isYorder=0
		self.Yerror=0
		if self.start_Y_tracking:
			focusMeasure = image_processing.YTracking_Objective_Function(cropped_image, self.color)
			self.Yerror,self.isYorder= self.ytracker.get_error(focusMeasure)
			# Disabling Y tracking for 3D PIV
			# self.isYorder = 0



	def getOrder(self,imageCenter,centroidGlob,shouldInitiatePID):
	   
		
		Xerror = -self.central_widget.units_converter.X_mm_to_step(self.central_widget.units_converter.px_to_mm(imageCenter[0]-centroidGlob[0], self.image_width))
		
		Yerror = self.central_widget.units_converter.Y_mm_to_step(self.Yerror)
		Yerror = round(Yerror,2)

		Zerror= self.central_widget.units_converter.Z_mm_to_step(self.central_widget.units_converter.px_to_mm(imageCenter[1]-centroidGlob[1] , self.image_width),self.Xobj[-1]) #Attention, it is Z_Wheel and not Z stage
				
		# Zerror = -Zerror
		# Xerror = -Xerror
		#speed based control
		#Zerror=units_converter.Z_mm_to_step(self.SpeedBasedfactor*units_converter.px_to_mm(imageCenter[1]-centroidGlob[1], self.image_width)+self.VobjWheel*0.001*self.averageDt,self.Xobj[-1])
		
		#PID control
		if shouldInitiatePID:
			self.pid_z.initiate(Zerror,self.Time[-1]) #reset the PID
			self.pid_x.initiate(Xerror,self.Time[-1]) #reset the PID
			self.pid_y.initiate(Yerror,self.Time[-1]) #reset the PID
			Zorder = 0
			Xorder = 0
			Yorder = 0
		else:
			Zorder=self.pid_z.update(Zerror,self.Time[-1])
			Zorder=round(Zorder,2)
			Xorder=self.pid_x.update(Xerror,self.Time[-1])
			Xorder=round(Xorder,2)

			#Yorder=self.pid_y.update(Yerror,self.Time[-1])
			Yorder = Yerror #@@@ NonPID focus tracking; may need to reverse the sign
			Yorder = round(Yorder,2)
			
			# print('Xorder: {}'.format(Xorder))
			# print('Zorder: {}'.format(Zorder))
			
		#falling object specific parameter
		#Zorder=min(Zorder,0)

		# Update the light intensity level to be sent to the Arduino (Only if we are currently running a Light modulation experiment)
		
		if(self.light_experiment):
			self.setLEDIntensity(profile_type = 'exp')
		else:
			self.LED_intensity = 0


		#print('LED intensity at : {}'.format(self.LED_intensity))
		
		if self.start_Y_tracking:
			# Updated to send the Trigger command and Sampling Rate for Video streams
			if('FL' in self.central_widget.imaging_channels):
				return np.array([self.liquid_lens_freq, self.isYorder,0,self.flag,Xorder,Yorder,Zorder,self.averageDt, self.LED_intensity, self.central_widget.trigger_flag['FL'], 
					self.central_widget.sampling_interval['FL']])
			else:
				return np.array([self.liquid_lens_freq, self.isYorder,0,self.flag,Xorder,Yorder,Zorder,self.averageDt, self.LED_intensity, 0, 0])


		else:
			if('FL' in self.central_widget.imaging_channels):
			# Updated to send the Trigger command and Sampling Rate for Video streams
				return np.array([self.liquid_lens_freq, self.isYorder,0,self.flag,Xorder,0,Zorder,self.averageDt, self.LED_intensity, self.central_widget.trigger_flag['FL'], self.central_widget.sampling_interval['FL']])
			else:
				return np.array([self.liquid_lens_freq, self.isYorder,0,self.flag,Xorder,0,Zorder,self.averageDt, self.LED_intensity, 0, 0])


	def register_data(self):

		# print(len(self.ytracker.YmaxFM))

		if (self.start_Y_tracking):
			# self.csv_register.write_line([[self.Time[-1],self.Xobj[-1],self.Yobj[-1],self.Zobj[-1],self.ThetaWheel[-1],self.ZobjWheel[-1],self.manualMode,self.current_image_name, self.ytracker.YfocusMeasure[-1],self.ytracker.YfocusPhase[-1],self.liquid_lens_freq,self.liquid_lens_ampl,self.ytracker.maxGain, self.ytracker.YmaxFM[-1],self.LEDpanel_color[0],self.LEDpanel_color[1],self.LEDpanel_color[2]]])
			self.csv_register.write_line([[self.Time[-1],self.Xobj[-1],self.Yobj[-1], self.Xobj_image[-1], 
				self.Zobj[-1],self.ThetaWheel[-1],self.ZobjWheel[-1],self.manualMode, 
					self.ytracker.YfocusMeasure[-1],self.ytracker.YfocusPhase[-1],self.liquid_lens_freq,
						self.liquid_lens_ampl,self.ytracker.maxGain, self.ytracker.YmaxFM, int(self.light_experiment), 
							self.LED_intensity, self.ytracker.YfocusPosition[-1]] 
								+ [self.current_image_name[key] for key in self.central_widget.imaging_channels]])
		else:
			# self.csv_register.write_line([[self.Time[-1],self.Xobj[-1],self.Yobj[-1],self.Zobj[-1],self.ThetaWheel[-1],self.ZobjWheel[-1],self.manualMode,self.current_image_name, 0,0,self.liquid_lens_freq,self.liquid_lens_ampl,self.ytracker.maxGain, 0,self.LEDpanel_color[0],self.LEDpanel_color[1],self.LEDpanel_color[2]]])
			self.csv_register.write_line([[self.Time[-1],self.Xobj[-1],self.Yobj[-1], self.Xobj_image[-1], 
				self.Zobj[-1],self.ThetaWheel[-1],self.ZobjWheel[-1],self.manualMode, 
					0,0,self.liquid_lens_freq,
						self.liquid_lens_ampl,self.ytracker.maxGain, 0, int(self.light_experiment), 
							self.LED_intensity, self.ytracker.YfocusPosition[-1]] 
								+ [self.current_image_name[key] for key in self.central_widget.imaging_channels]])

		for key in self.central_widget.imaging_channels:
			self.current_image_name[key] = '' #reset image name to ''
			
	def setImageName(self,image_name):
		self.current_image_name[self.central_widget.tracking_channel] = image_name

	def setImageName_FL(self,image_name):
		self.current_image_name['FL'] = image_name

	def tune_pid_z(self,P,I,D):
		self.pid_z.set_Tuning(P,I,D)
		
	def tune_pid_x(self,P,I,D):
		self.pid_x.set_Tuning(P,I,D)

	def tune_pid_y(self,P,I,D):
		self.pid_y.set_Tuning(P,I,D)

	#Calculation of derivatives
	def update_VobjZWheel(self):
		self.VobjWheel=0
		n=min(len(self.Time),10)
		if n==1:
			self.VobjWheel=0
		else:
			for i in range(1,n):
				self.VobjWheel+=(self.ZobjWheel[-i]-self.ZobjWheel[-(i+1)])/(self.Time[-i]-self.Time[-(i+1)])
			self.VobjWheel=self.VobjWheel/n
	
	def update_averageDt(self):
		self.averageDt=0
		n=min(len(self.Time),5)
		if n==1:
			self.averageDt=0
		else:
			for i in range(1,n-1):

				self.averageDt+=round(1000*(self.Time[-i]-self.Time[-(i+1)]),2)
			self.averageDt=self.averageDt/n
			self.averageDt=max(300,self.averageDt) #if we track and loose the guy for a long time, dt will be huge, too big for serial communication

	#not the best way to do it ...
	def initialise_data(self): #when click on "stop tracking, data is being reinitialised"

		self.globCounter=0             #number of frame a particle is tracked for a track period
		self.trackGapCounter = 0;      #Keeps track of nb of consecutive frames when object is not tracked
		self.flag=0                    #flag=1 if a centroid is detected on the current frame

		#Time
		self.begining_Time=0           #Time begin the first time we click on the start_tracking button
		self.Time=deque(maxlen=self.dequeLen)
		self.averageDt=0
		
		#Total angle traveled by the wheel (unit=rad)
		self.ThetaWheel=deque(maxlen=self.dequeLen) #trigonometric sens
		
		#Location of the left corner of the cropped image in the Image referenciel (unit=px)
		self.origLoc=np.array((0,0))   
		
		#Object position in Image referenciel (unit=px)
		self.centroids = deque(maxlen=self.dequeLen) # Create a buffer to store the tracked object centroids
		
		#Image center position in the centerline referentiel (unit=mm)
		self.Ximage=deque(maxlen=self.dequeLen)            #distance to the centerline of the flow channel
		self.Yimage=deque(maxlen=self.dequeLen)            #distance to the centerline of the flow channel
		self.Zimage=deque(maxlen=self.dequeLen)            #distance to the centerline of the flow channel

		# Distance of object from the image-center (or more generally the origin of the tracking point)
		self.Xobj_image = deque(maxlen=self.dequeLen)
		self.Zobj_image = deque(maxlen=self.dequeLen)
		
		#Object position in the centerline referentiel  #distance to the centerline of the flow channel
		self.Yobj=deque(maxlen=self.dequeLen)            #distance to the centerline of the flow channel
		self.Zobj=deque(maxlen=self.dequeLen)            #distance to the centerline of the flow channel
	   
		self.ZobjWheel=deque(maxlen=self.dequeLen)         #Zobj in the wheel's referentiel
		self.VobjWheel=0                                   #object's speed in the wheel referential
		
		#Y-Tracking parameters and data
		self.ytracker=YTracking.YTracker()

		# We also create a new instance of the OpenCV tracker
		self.createTracker()

		# Reset the LED intensity
		self.LED_intensity = 0
		self.ZobjWheel_init = 0

	def setLEDIntensity(self, profile_type = 'binary'):

		ZobjWheel_rel = self.ZobjWheel[-1] - self.ZobjWheel_init

		print('Relative depth: {} mm'.format(ZobjWheel_rel))

		if(profile_type == 'binary'):
			if(ZobjWheel_rel > self.threshDepth):
				self.LED_intensity = self.maxLEDIntensity
			else:
				self.LED_intensity = 0
		elif(profile_type == 'exp'):

			self.LED_intensity = int(self.surface_intensity * np.exp( -self.K*(self.surface_Z - ZobjWheel_rel)))

		elif(profile_type == 'sqwave'):

			sign = np.sin(np.pi*ZobjWheel_rel/self.threshDepth)

			if(sign >=0):
				self.LED_intensity = 0
			else:
				self.LED_intensity = self.maxLEDIntensity


		if(self.LED_intensity > self.maxLEDIntensity):
			self.LED_intensity = self.maxLEDIntensity
		elif(self.LED_intensity < self.minLEDIntensity):
			self.LED_intensity = self.minLEDIntensity



'''       
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                            Video Acquisition
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

class Camera_Functions(QtCore.QObject):
	
	"""
	Camera_Functions provides functions to handle a video stream, including basic image processing steps 
	to pass the video stream to different widgets like: track, display and save.

	Parameters
	---------------



	Attributes
	---------------


	Signals
	---------------
	image_data: The raw image (maybe resized) used for tracking
	image_display: Image stream for display
	thresh_image_display: Thresholded image stream for display (enables identifying object of interest)
	plot_data_display: Array object containing data to be sent to the plot widget. 
	@@@ DK: This is controlled right now by the Camera_Functions class because the display frame rate sets 
	the rate at which the plots are updated. Consider modifying. @@@






	"""
	image_data = QtCore.pyqtSignal(np.ndarray)  #original image resized with working size

	image_display = QtCore.pyqtSignal(np.ndarray)  #original image resized with working size
	thresh_image_display = QtCore.pyqtSignal(np.ndarray) #thresholded image (working size)
	plot_data_display=QtCore.pyqtSignal(np.ndarray) #sent the data to plot
	
	fps_data = QtCore.pyqtSignal(np.ndarray)  #real number of frame per second received by Camera_Function
	image_name = QtCore.pyqtSignal(str)  #name of the image being recorder (so we could match the tracking data with the image)
	image_width=QtCore.pyqtSignal(int)   
	
	# stream_type : Type of video stream: DF, Fluorescence, etc
	def __init__(self, channel = None, camera_port=0, CAMERA=3, Serial = None, resolution = (2048,1536), 
		camFPS = 30, color = False, parent=None, saveStream = False, dispStream = False, dispThreshStream = False, 
		trackingStream = False): #CAMERA=2 corresponds to a classic webcam

		super().__init__(parent)
		self.channel = channel
		self.trackingStream = trackingStream
		self.saveStream = saveStream
		self.dispStream = dispStream
		self.dispThreshStream = dispThreshStream

		self.camera_port = camera_port
		# DMK 33UX252 camera resolution
		self.camResolution = resolution
		# self.camResolution=(1440,1080)
		# self.camResolution=(640,480)
		self.camFPS = camFPS
		# self.camFPS=2500000 # for 2500000/10593
		self.CAMERA=CAMERA

		self.serial = Serial

		# track busy flag
		self.track_busy = False

		# trigger counter
		self.triggerCounter = 0

		# Color vs grayscale camera
		self.color = color
		
		
		#thread to et the image
		self.VideoSrc = VideoStream(src = self.camera_port,CAMERA = self.CAMERA, Serial = self.serial, 
		resolution = self.camResolution,framerate = self.camFPS, color = self.color) # Grab a reference to a VideoStream object (cross-platform functionality)
		self.VideoSrc.set_property("Trigger Mode", False)
	# if(self.color==True):
		self.VideoSrc.set_property("Whitebalance Auto", False)
		self.VideoSrc.set_property("Whitebalance Red", 64)
		self.VideoSrc.set_property("Whitebalance Green", 50)
		self.VideoSrc.set_property("Whitebalance Blue", 64)
		self.VideoSrc.set_property("Gain Auto", False)
		self.VideoSrc.set_property("Gain", 100)
		self.VideoSrc.set_property("Exposure Auto", False)
		self.VideoSrc.set_property("Exposure", 100)
		# if(self.trackingStream):
		self.VideoSrc.set_property("Trigger Delay (us)", 0)
		# else:

		self.VideoSrc.set_property("Trigger Polarity", "RisingEdge")
		self.VideoSrc.set_property("Trigger Mode", False) # for now this needs to be here
		self.VideoSrc.start()

		# Testing : Keep the other video stream as software-trigger
		# if(self.trackingStream):
		time.sleep(2.0)
		self.VideoSrc.set_property("Trigger Mode", True)
		# print('Set Trigger Mode of Channel : {} to True'.format(self.channel))
		#----------------------------------------------------------
		# This allows readFrame (the main function that handles the output of the video stream), 
		# to be called whenever a new sample is available from the camera
		#----------------------------------------------------------
		self.VideoSrc.set_newImage_callback(self.readFrame,self)
		
		#thead for saving image
		self.image_saver = ImageSaver()
		
		#imaging parameters
		self.rampFrames = 30 #number of frames for the camera warm_up
		self.working_width = 720 # width of the image use for data analasys
		self.maximal_width = 720 #width of the image
		
		
		self.fps_sampling = 2000.  #frequency of the QTimer signals. fps_real<=fps_sampling. 
		# Right now this doesnt really control anything.
		
		self.fps_displaying = 10.
		self.fps_saving = 15.
		 
		self.fps_sampling_real=0
		self.fps_displaying_real=0
		self.fps_saving_real=0
		
		self.prev_sampling_time=0 #time of the last frame read
		self.prev_displaying_time=0
		self.prev_saving_time=0     
		
		
		self.take_photo=False
		self.photo_path=''
		
		self.record_video=False
		self.video_root_path = ''
		self.video_path=''
		self.image_folder_num = 0
		self.saved_img_nb=0 #+1 for each new frame
		self.saved_img_nb_local = 0		# This counts from 0 to images_per_folder and is reset when the upper bound is reached.
		self.images_per_folder = 2000

#        self.timer = QtCore.QTimer() #produce event on a given interval
#        self.timer.setTimerType(0)
#        self.timer.setInterval (1./self.fps_sampling*1000.)#time in ms
#        self.timer.timeout.connect(self.readFrame) #each in interval of time is over, the function readFrame() is launched
		self.timer = QtCore.QBasicTimer()
		
		self.image_setting=False  
		self.contrast=0
		self.brightness=0
		self.saturation=0

		self.isGrayscale=False #if true the working image will be converted in grayscale at the very beginning
		 
		# The thresholding and image annotation features are only required for the tracking stream
		if(self.trackingStream): 
			self.lower_HSV=np.array([0,0,0],dtype='uint8') 
			self.upper_HSV=np.array([178,255,255],dtype='uint8')
			
			#To draw the cercle on the displayed image
			self.isCircle_todraw=False
			self.centroid=(0,0)
			self.isRect_todraw=False
			self.ptRect1=(0,0)
			self.ptRect2=(0,0)

			# plot data is displayed only for the video stream used for tracking
			#receive the data to plot at fps_sampling and resent them at fps_display

			self.plot_data=np.array([0,0,0,0,0,0,0,0,0]) 
		else:
			self.plot_data = None

	def set_trackBusy(self):
		self.track_busy = True
	
	def clear_trackBusy(self):
		self.track_busy = False

	def timerEvent(self, event):
		if (event.timerId() != self.timer.timerId()):
			return
		# if ((time.time()-self.prev_sampling_time)>1./self.fps_sampling):
		# 	self.readFrame()

			
	def start_displaying(self):
		#camera warmup
		for i in range(self.rampFrames):
			temp = self.VideoSrc.read() 

		if(temp.shape[1]==0):
			print("Maybe plug and unplug your camera dude !")   
		self.maximal_width = temp.shape[1]
		self.image_width.emit(self.maximal_width) #to scale the resolution slider
		self.timer.start(0,self) #Launch the QTimer --> images will begin to be read

	def saveFrame(self, img, current_time):

		if(self.saved_img_nb_local >= self.images_per_folder):
			# Create a new sub directory to save the images
			self.image_folder_num += 1

			self.video_path = os.path.join(self.video_root_path, 'images{:05d}'.format(self.image_folder_num))

			# We dont need to do this check. Consider removing.
			if(not os.path.exists(self.video_path)):
				os.mkdir(self.video_path)

			self.saved_img_nb_local = 0

		imageName = 'IMG_{:07d}'.format(self.saved_img_nb)+'.tif'
	
		path = os.path.join(self.video_path, imageName)
		
		self.image_saver.wait() #wait for the previous image to be completely saved
		self.image_saver.register(path,img)
		
		self.image_name.emit(imageName)
		self.saved_img_nb += 1
		self.saved_img_nb_local += 1
		self.set_fps_real(current_time,'saving')



	# This function specifies what happens when a frame is read from camera.
	def readFrame(self,self_):
		
		# print(self.VideoSrc.triggered())
		
		# check if camera is triggered
		if not self.VideoSrc.triggered():
			return

		# print('readFrame become aware of the new image: ' + str(time.time())) #@@@
		self.triggerCounter = self.triggerCounter + 1
		# print('Number of images received: ' + str(self.triggerCounter))

		# check if the last frame is still being processed
		if(self.trackingStream):
			if self.track_busy:
				return
		
		# @@@ to be implemented @@@
		# @@@ check if a frame is being processed by track() by checking track_busy flag
		# @@@ return if there is
		# >>> may need to change the way data is read from arduino to handle the case when one or more frames have to be dropped (draw the timing diagram) 

		# print('Reading image from {} channel'.format(self.channel))
		img = self.VideoSrc.read() 	#all images are naturally encode in BRG on a computer
		# if rotate_90:
		# 	img = cv2.rotate(img,cv2.ROTATE_90_CLOCKWISE)
			# ROTATE_90_CLOCKWISE
			# ROTATE_90_COUNTERCLOCKWISE


		
		current_time=time.time()

		self.set_fps_real(current_time,'sampling') #calculate the real fps

		# if stop here, can reach > 200 fps		
		# # debug start
		# self.fps_data.emit(np.array([self.fps_sampling_real,self.fps_displaying_real,self.fps_saving_real])) 
		# return
		# # debug end
				
		# Store the raw image in grayscale format
		#data_gray = image_processing.bgr2gray(img)
		
		
		#save data if needed, in full quality
		if self.take_photo:
			cv2.imwrite(self.photo_path,img)
			self.take_photo=False
			
		#resize image
		# img = np.array(img, dtype="uint8")                       #To be sure the image is in 8bit int. Maybe useless but do no harm

		# Need to flip the image since we are imaging through a mirror
		if(self.trackingStream):
			img = cv2.flip(img, 0)
		# else:
			# img = cv2.flip(img, -1)


		# Resize the image for analysis purposes
		if(self.trackingStream):
			img_resized = imutils.resize(img, width = self.working_width)



			
			
		
		#image enhancement (super costly)
		if self.image_setting:
			img = image_enhancement.Contrast_Brightness(img,self.contrast,self.brightness)
			img = image_enhancement.Saturation(img,self.saturation)

		# We only display thresholded image for the tracking stream
		if(self.trackingStream):

			# Only save the image if ...
	
			if self.record_video and ( (current_time-self.prev_saving_time)>1./self.fps_saving) and self.saveStream is True :
				# Save frame according to which imaging channel, and in the sub-folder structure specified accordingly.
				self.saveFrame(img, current_time)

		#Send image to display.
			if  (current_time-self.prev_displaying_time)>1./self.fps_displaying:
				
				self.set_fps_real(current_time,'displaying')
				
				if(self.color):
					thresh_image = image_processing.threshold_image(img_resized,self.lower_HSV,self.upper_HSV)  #The threshold image as one channel
					data_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB) #pg.ImageItem read images in RGB by default

				else:
					# print(self.lower_HSV[2])
					# print(self.upper_HSV[2])
					# img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
					thresh_image = image_processing.threshold_image_gray(img_resized, self.lower_HSV[2], self.upper_HSV[2])
					data_rgb = img_resized

				# Store a distinct copy of the image for display purposes
				data_rgb_draw = np.copy(data_rgb)
			
				# Image annotation only for the tracking stream
		
				#rectangle drawing (cropped image)
				if self.isRect_todraw:
					
					cv2.rectangle(data_rgb_draw, self.ptRect1, self.ptRect2,(0,0,0) , 2) #cv2.rectangle(img, (20,20), (300,300),(0,0,255) , 2)#
					self.isRect_todraw=False
					
				#circle drawing in case of a tracked object
				if self.isCircle_todraw:
					# cv2.circle(data_rgb_draw,self.centroid, 20, (255,0,0), 2)
					self.isCircle_todraw=False
			
				# This displays the image
				# print("Sending DF image to image display")
				self.image_display.emit(data_rgb_draw) #send the image to 'image_widget' 
				self.thresh_image_display.emit(thresh_image) #sent to the displayer and to "Object Tracking
				self.fps_data.emit(np.array([self.fps_sampling_real,self.fps_displaying_real,self.fps_saving_real])) 
				self.plot_data_display.emit(self.plot_data) #sent the last plot_data received to the displayer

			
		else:
			# If this is not a tracking stream then just display the image in the appropriate window
		
			# Here we only need to display the raw image
			# print("Sending FL image to image display")
			# print(img)
			self.image_display.emit(img)
			# Save the image if this not the tracking stream (the rate here is set by the hardware trigger)
			self.saveFrame(img, current_time)

			
		
		#print('new image is read from the RAM: ' + str(time.time())) #@@@
		#Sent resized image at the sampling frequency
		# print('Image sum : {}'.format(np.sum(img)))

		# We send the image as image_data for tracking if this is a tracking stream
		if(self.trackingStream):
			# The neural network trackers expect to see a 3-channel image
			img_data_bgr = cv2.cvtColor(img_resized, cv2.COLOR_GRAY2BGR)
			self.image_data.emit(img_data_bgr)
				
	
	def set_fps_real(self,t1,string):    
		if string=='sampling':
			fps2 = 1.0 / (t1-self.prev_sampling_time)
			self.prev_sampling_time=t1
			self.fps_sampling_real=self.fps_sampling_real * 0.9 + fps2* 0.1
		elif string=='displaying':
			fps2 = 1.0 / (t1-self.prev_displaying_time)
			self.prev_displaying_time=t1
			self.fps_displaying_real=self.fps_displaying_real * 0.9 + fps2* 0.1
		elif string=='saving':
			fps2 = 1.0 / (t1-self.prev_saving_time)
			self.prev_saving_time=t1
			self.fps_saving_real=self.fps_saving_real * 0.9 + fps2* 0.1
		
	def draw_circle(self,centroid_Glob):
		self.isCircle_todraw=True
		self.centroid=(centroid_Glob[0],centroid_Glob[1])
	
	def draw_rectangle(self,pts):
		self.isRect_todraw=True
		self.ptRect1=(pts[0][0],pts[0][1])
		self.ptRect2=(pts[1][0],pts[1][1])
		
	def stop(self):
		self.VideoSrc.stop()
		self.timer.stop()
		
		
'''
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                            Plot widget
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

class dockAreaPlot(dock.DockArea):
	def __init__(self, parent=None):
		super().__init__(parent)
		DockLabel.updateStyle = dstyle.updateStylePatched
		self.plot1=PlotWidget('XObject')
		self.plot2=PlotWidget('YObject')
		self.plot3=PlotWidget('ZObject')
		self.plot4=PlotWidget('ZObjWheel')
		self.plot5=PlotWidget('ThetaWheel')
		self.plot6=PlotWidget('FocusMeasure')
		self.plot7=PlotWidget('Yerror')
		self.plot8=PlotWidget('Phase')
		
		dock1=dock.Dock('XObject')
		dock1.addWidget(self.plot1)
		
		dock2=dock.Dock('YObject')
		dock2.addWidget(self.plot2)
		
		dock3=dock.Dock('ZObject')
		dock3.addWidget(self.plot3)
		
		dock4=dock.Dock('ZObjWheel')
		dock4.addWidget(self.plot4)

		dock5=dock.Dock('ThetaWheel')
		dock5.addWidget(self.plot5)

		dock6=dock.Dock('FocusMeasure')
		dock6.addWidget(self.plot6)

		dock7=dock.Dock('Yerror')
		dock7.addWidget(self.plot7)

		dock8=dock.Dock('Phase')
		dock8.addWidget(self.plot8)
		
		self.addDock(dock6)
		self.addDock(dock8,'right',dock6)
		self.addDock(dock1,'above',dock6)
		self.addDock(dock2,'above',dock1)
		self.addDock(dock3,'above',dock2)
		self.addDock(dock4,'above',dock3)
		self.addDock(dock5,'above',dock4)
		self.addDock(dock7,'above',dock5)

	def initialise_plot_area(self):
		self.plot1.initialise_plot()
		self.plot2.initialise_plot()
		self.plot3.initialise_plot()
		self.plot4.initialise_plot()
		self.plot5.initialise_plot()
		self.plot6.initialise_plot()
		self.plot7.initialise_plot()


class PlotWidget(pg.GraphicsLayoutWidget):
	def __init__(self,title, parent=None):
		super().__init__(parent)
		self.title=title
		#plot Zobj
		self.Abscisse=deque(maxlen=20)
		self.Ordonnee=deque(maxlen=20)
		
		self.Abs=[]
		self.Ord=[]
		self.plot1=self.addPlot(title=title)
		self.curve=self.plot1.plot(self.Abs,self.Ord)
		#self.plot1.plot(self.Ord)
		self.plot1.enableAutoRange('xy', True)
		self.plot1.showGrid(x=True, y=True)
		
		
	def update_plot(self,data):
		
		self.Abscisse.append(data[0])
		if self.title=='XObject':
			self.Ordonnee.append(data[1])
			self.label='mm'
		elif self.title=='YObject':
			self.Ordonnee.append(data[2])
			self.label='mm'
		elif self.title=='ZObject':
			self.Ordonnee.append(data[3])
			self.label='mm'
		elif self.title=='ZObjWheel':
			self.Ordonnee.append(data[4])
			self.label='mm'
		elif self.title=='ThetaWheel':
			self.Ordonnee.append(data[5])
			self.label='Radians'
		elif self.title == 'FocusMeasure':
			self.Ordonnee.append(data[6])
			self.label='Variance'
		elif self.title == 'Yerror':
			self.Ordonnee.append(data[7])
			self.label='mm'
		elif self.title == 'Phase':
			self.Ordonnee.append(data[8])
			self.label='mm'
			
		self.Abs=list(self.Abscisse)
		self.Ord=list(self.Ordonnee)

		self.curve.setData(self.Abs,self.Ord)

	def initialise_plot(self):
		self.Abscisse=deque(maxlen=20)
		self.Ordonnee=deque(maxlen=20)
		self.Abs=[]
		self.Ord=[]
		self.curve.setData(self.Abs,self.Ord)
'''
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                            Video_widget
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

class GraphicsLayoutWidget(pg.GraphicsLayoutWidget):
	
	
	def __init__(self, parent=None):
		super().__init__(parent)

		self.view = self.addViewBox()
		
		## lock the aspect ratio so pixels are always square
		self.view.setAspectLocked(True)
		
		## Create image item
		self.img = pg.ImageItem(border='w')
		self.view.addItem(self.img)
		
	def image_data_slot(self, data_rgb):      
		
		data_rgb=cv2.rotate(data_rgb,cv2.ROTATE_90_CLOCKWISE) #pgItem display the image with 90° anticlockwise rotation
		self.img.setImage(data_rgb)


class GraphicsLayoutWidget_newWindow(QtGui.QMainWindow):
	
	
	def __init__(self, parent=None):
		super().__init__(parent)
		self.setWindowTitle('Fluorescence Channel')
		self.widget = QtWidgets.QWidget()

		# Add a graphics widget to the window
		self.graphics_widget = pg.GraphicsLayoutWidget()

		self.graphics_widget.view = self.graphics_widget.addViewBox()
		
		## lock the aspect ratio so pixels are always square
		self.graphics_widget.view.setAspectLocked(True)
		
		## Create image item
		self.graphics_widget.img = pg.ImageItem(border='w')
		self.graphics_widget.view.addItem(self.graphics_widget.img)

		layout = QtGui.QGridLayout()
		
		layout.addWidget(self.graphics_widget, 0, 0) 

		self.widget.setLayout(layout)

		self.setCentralWidget(self.widget)
		
	def image_data_slot(self, data_rgb):      
		
		data_rgb=cv2.rotate(data_rgb,cv2.ROTATE_90_CLOCKWISE) #pgItem display the image with 90° anticlockwise rotation
		self.graphics_widget.img.setImage(data_rgb)
		
		
class ImgDisplayerThread(QtCore.QThread):
	
	def __init__(self, new_window = False):
		QtCore.QThread.__init__(self)
		self.queue = Queue()
		if(new_window is True):
			self.graphic_layout = GraphicsLayoutWidget_newWindow()
		else:
			self.graphic_layout = GraphicsLayoutWidget()

	def __del__(self):
		self.wait()
		
	def run(self):
		while True:
			data_rgb=self.queue.get()
			self.graphic_layout.image_data_slot(data_rgb)
			self.queue.task_done()

	def display_new_img(self,data_rgb):
		
		self.queue.put(data_rgb)
		
		
'''
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                            Threshold_image
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''
		
class Threshold_image(pg.GraphicsLayoutWidget):
	
	def __init__(self, parent=None):
		super().__init__(parent)
		
		
		self.setWindowTitle('threshold_image')
		


		self.view = self.addViewBox()
		
		## lock the aspect ratio so pixels are always square
		self.view.setAspectLocked(True)
		
		## Create image item
		self.img=pg.ImageItem(border='w')
		self.view.addItem(self.img)       

	def thresh_data_slot(self, threshold_image_display):      
		
		threshold_image_display=cv2.rotate(threshold_image_display,cv2.ROTATE_90_CLOCKWISE) #pgItem display the image with 90° anticlockwise rotation
		self.img.setImage(threshold_image_display)
		
class ThresImgDisplayerThread(QtCore.QThread):
	
	def __init__(self):
		QtCore.QThread.__init__(self)
		self.queue = Queue()
		self.threshold_image = Threshold_image()

	def __del__(self):
		self.wait()
		
	def run(self):
		while True:
			thres_image=self.queue.get()
			self.threshold_image.thresh_data_slot(thres_image)
			self.queue.task_done()
		

	def display_new_thresimg(self,data_rgb):
		self.queue.put(data_rgb)
	   
		
'''
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                            PID class
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''
class PIDgroupbox(QtGui.QGroupBox):
	
	def __init__(self,name,Pmax=2,Dmax=1,Imax=1):
		super().__init__()
		
		self.setTitle(name)
	
		# Slider Groupe P
		defaultP = Pmax/2
		stepP = Pmax/100

		self.labelP = QtGui.QLabel('P')
		self.hsliderP = QtGui.QSlider(QtCore.Qt.Horizontal)
		self.hsliderP.setRange(0,int(Pmax*100))
		self.hsliderP.setValue(int(defaultP*100))
		self.spinboxP=QtGui.QDoubleSpinBox()
		self.spinboxP.setRange(0,round(Pmax,2))
		self.spinboxP.setSingleStep(round(stepP,2))
		self.spinboxP.setValue(round(defaultP,2))
		self.hsliderP.valueChanged.connect(self.spinBoxP_setValue)
		self.spinboxP.valueChanged.connect(self.hsliderP_setValue)
		sliderP_layout=QtGui.QHBoxLayout()
		sliderP_layout.addWidget(self.labelP)
		sliderP_layout.addWidget(self.hsliderP)
		sliderP_layout.addWidget(self.spinboxP)
		group_sliderP=QtWidgets.QWidget()
		group_sliderP.setLayout(sliderP_layout)
		

		defaultI = 0
		stepI = Imax/100
		# Slider Groupe I
		self.labelI = QtGui.QLabel('I')
		self.hsliderI = QtGui.QSlider(QtCore.Qt.Horizontal)
		self.hsliderI.setRange(0,int(Imax*100))
		self.hsliderI.setValue(int(defaultI*100))
		self.spinboxI=QtGui.QDoubleSpinBox()
		self.spinboxI.setSingleStep(round(stepI,2))
		self.spinboxI.setRange(0,int(Imax))
		self.spinboxI.setValue(round(defaultI,2))
		self.hsliderI.valueChanged.connect(self.spinBoxI_setValue)
		self.spinboxI.valueChanged.connect(self.hsliderI_setValue)
		sliderI_layout=QtGui.QHBoxLayout()
		sliderI_layout.addWidget(self.labelI)
		sliderI_layout.addWidget(self.hsliderI)
		sliderI_layout.addWidget(self.spinboxI)
		group_sliderI=QtWidgets.QWidget()
		group_sliderI.setLayout(sliderI_layout)
		
		# Slider Groupe D
		defaultD = Dmax/4
		stepD = Dmax/100

		self.labelD = QtGui.QLabel('D')
		self.hsliderD = QtGui.QSlider(QtCore.Qt.Horizontal)
		self.hsliderD.setRange(0,int(Dmax*100))
		self.hsliderD.setValue(int(defaultD*100))
		self.spinboxD=QtGui.QDoubleSpinBox()
		self.spinboxD.setRange(0,int(Dmax))
		self.spinboxI.setSingleStep(round(stepD,2))
		self.spinboxD.setValue(round(defaultD,2))
		self.hsliderD.valueChanged.connect(self.spinBoxD_setValue)
		self.spinboxD.valueChanged.connect(self.hsliderD_setValue)
		sliderD_layout=QtGui.QHBoxLayout()
		sliderD_layout.addWidget(self.labelD)
		sliderD_layout.addWidget(self.hsliderD)
		sliderD_layout.addWidget(self.spinboxD)
		group_sliderD=QtWidgets.QWidget()
		group_sliderD.setLayout(sliderD_layout)
		
				# Big PID group
		groupbox_layout_PID = QtGui.QVBoxLayout()
		groupbox_layout_PID.addWidget(group_sliderP)   
		groupbox_layout_PID.addWidget(group_sliderI)
		groupbox_layout_PID.addWidget(group_sliderD)
		
		
		self.setLayout(groupbox_layout_PID)
	
	def spinBoxP_setValue(self,value):
		newvalue=float(value)/100.
		self.spinboxP.setValue(newvalue)

	def hsliderP_setValue(self,value):
		self.hsliderP.setValue(int(value*100)) 

	def spinBoxI_setValue(self,value):
		newvalue=float(value)/100.
		self.spinboxI.setValue(newvalue)

	def hsliderI_setValue(self,value):
		self.hsliderI.setValue(int(value*100)) 

	def spinBoxD_setValue(self,value):
		newvalue=float(value)/100.
		self.spinboxD.setValue(newvalue)

	def hsliderD_setValue(self,value):
		self.hsliderD.setValue(int(value*100))

'''
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                            Light Experiment class
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
This allows the user to setup a light-control experiment. eg. Changing light intensity with depth etc.
'''		
# class LightExperiment(QtGui.QMainWindow):

# 	def __init__(self,parent=None):
#     	super().__init__()

# 		#GROUPBOX LED PANEL
# 		layout_LEDPanel=QtGui.QHBoxLayout()
# 		self.label_LEDcolor=QtGui.QLabel('LED Color')
# 		self.LED_color_picker=QColorEdit()
# 		self.button_LED_on = QtGui.QPushButton('Run Light Experiment')
# 		self.button_LED_on.setIcon(QtGui.QIcon('icon/light.png'))
# 		self.button_LED_on.setCheckable(True)
# 		self.button_LED_on.setChecked(False)
# 		self.button_LED_tracking = QtGui.QPushButton('LED Panel Tracking')
# 		self.button_LED_tracking.setIcon(QtGui.QIcon('icon/video.png'))
# 		self.button_LED_tracking.setCheckable(True)
# 		self.button_LED_tracking.setChecked(False)
# 		layout_LEDPanel.addWidget(self.label_LEDcolor)
# 		layout_LEDPanel.addWidget(self.LED_color_picker)
# 		layout_LEDPanel.addWidget(self.button_LED_on)
# 		layout_LEDPanel.addWidget(self.button_LED_tracking)
		
# 		self.LEDPanel_box = QtGui.QGroupBox('Light Control Panel')
# 		self.LEDPanel_box.setLayout(layout_LEDPanel)

'''
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                            Central Widget
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''
class CentralWidget(QtWidgets.QWidget):
	
   
	def __init__(self):
		super().__init__()

		# Call the units converter class
		self.units_converter = units_converter.units_converter()
		#------------------------------------------------------------
		# Optical Path configurations and pixel size (Pixels per mm)
		#------------------------------------------------------------
		self.objectives = {'4x':456, '10x':1122.67, '20x':2280}

		self.default_objective = '10x'

		self.pixelsPermm = self.objectives[self.default_objective]

		self.units_converter.update_pixel_size(self.pixelsPermm)

		"""
		No:of video-streams/ imaging channels
		Possible options:
		1. DF + FL
		2. DF1 + DF2 (for two-camera tracking)
		3. DF1 + DF2 + FL 
		4. etc.
		"""
		self.imaging_channels = ['DF', 'FL']


		# Set the DF channel to be the tracking channel
		self.tracking_channel = 'DF'

		# Sets whether each video stream is displayed in a new window or in the main GUI window
		self.window_style = {'DF':False, 'FL':True}
		#------------------------------------------------------------
		#DISPLAYING THREAD
		#------------------------------------------------------------
		# Define display threads for each imaging channel
		self.img_displayer_thread = {key:ImgDisplayerThread(self.window_style[key]) for key in self.imaging_channels}

		# Start all the display threads
		for ii in self.imaging_channels:
			self.img_displayer_thread[ii].start()

		# Thread for thresholded images (only for the tracking thread)
		self.thres_img_displayer_thread=ThresImgDisplayerThread()
		self.thres_img_displayer_thread.start()
		#MOST IMPORTANT WIDGET / CLASS
		
		# Sampling rate of images for other video streams (this is in units of seconds)
		self.sampling_interval = {key: [] for key in self.imaging_channels}
		self.sampling_interval[self.tracking_channel] = (1/120)
		if('FL' in self.imaging_channels):
			self.sampling_interval['FL'] = 3

		# Trigger state of video streams
		# Currently the tracking stream is always triggered using Arduino
		self.trigger_flag = {key: False for key in self.imaging_channels}

		self.trigger_flag[self.tracking_channel] = True



		#Display widget
		#self.image_widget=GraphicsLayoutWidget()

		# Define image widgets for each video stream
		self.image_widget = {key: self.img_displayer_thread[key].graphic_layout 
							for key in self.imaging_channels}


	
		self.plot_widget = dockAreaPlot()
		
	
		#Camera(s) / Video Stream (s)
		# # Berg cameras
		# self.df_camera = "08910102"
		
		# self.fl_camera = "08910100"

		# GM-H-1 cameras
		self.df_camera = "48910083"
		
		self.fl_camera = "48910098"

		self.cameras = {key:[] for key in self.imaging_channels}

		# Assign a physical camera to each video stream (we can implement a drop-down menu if we want later)
		self.cameras['DF'] = self.df_camera
		self.cameras['FL'] = self.fl_camera

		# self.df_camera = "17910090"
		self.color = False


		self.max_imWidth = 1920
		# Sets the maximum width of the image based on the sensor format being used.
		self.units_converter.set_max_imWidth(self.max_imWidth)
		
		# Each video stream corresponds to a Camera_Functions instance. Create a new instance to add another video stream.
		# Create a dictionary so we can connect each video stream to one Camera Functions instance
		self.camera_functions = {key:[] for key in self.imaging_channels}

		# DF stream
		self.camera_functions['DF'] = Camera_Functions(channel = 'DF', camera_port = 0, Serial = self.cameras['DF'], 
			resolution = (1920,1080), camFPS = 180, color = False, saveStream = True, trackingStream = True, 
			dispThreshStream = True)

		# FL stream
		self.camera_functions['FL'] = Camera_Functions(channel = 'FL', camera_port = 0, Serial = self.cameras['FL'], 
			resolution = (1920,1080), camFPS = 30, color = False, saveStream = True, trackingStream = False, 
			dispThreshStream = False)


		# self.camera_functions=Camera_Functions(camera_port = 0, Serial = self.fl_camera, resolution = (1920,1080), camFPS = 120, color = False)

		#Tracking class (note that we pass the central widget object to object tracking)
		self.object_tracking=Object_Tracking(self, color = self.color)
		
		#OTHER WIDGETS
		self.XPID=PIDgroupbox(name = 'X PID Setting', Pmax = 2, Imax = 1, Dmax = 1)
		self.YPID=PIDgroupbox(name = 'Y PID Setting', Pmax = 2, Imax = 1, Dmax = 1)
		self.ZPID=PIDgroupbox(name = 'Z PID Setting', Pmax = 2, Imax = 1, Dmax = 1)
		
		# Homing completed flag
		self.homingComplete = False

		# Keeps track if a main directory has been created for the current dataset
		self.directory_flag = False
		#------------------------------------------------------------------------------------------------------------------------------------------------------
		#------------------------------------------------------------------------------------------------------------------------------------------------------
		# photo button
		self.button_photo = QtGui.QPushButton(' Take a picture')
		self.button_photo.setIcon(QtGui.QIcon('icon/photo.png'))
		self.photoName = QtGui.QLineEdit()
		self.photoName.setPlaceholderText('Name of the next pictures')
	   
		# checkable video pushbutton

		# Create one push-button per imaging channel
		self.button_video = {key:[] for key in self.imaging_channels}

		self.button_video[self.tracking_channel] = QtGui.QPushButton('Run')
		self.button_video[self.tracking_channel].setIcon(QtGui.QIcon('icon/video.png'))
		self.button_video[self.tracking_channel].setCheckable(True)
		self.button_video[self.tracking_channel].setChecked(False)

		
		if(len(self.imaging_channels) > 1):

			if('FL' in self.imaging_channels):

				self.groupbox_FL = QtGui.QGroupBox('{} channel'.format('FL'))

				self.button_video['FL'] = QtGui.QPushButton('Run')
				self.button_video['FL'].setIcon(QtGui.QIcon('icon/video.png'))
				self.button_video['FL'].setCheckable(True)
				self.button_video['FL'].setChecked(False)

				self.button_settings = QtGui.QPushButton('Settings')
				# self.button_settings.setIcon(QtGui.QIcon('icon/video.png'))
				self.button_settings.setCheckable(True)
				self.button_settings.setChecked(False)

				groupbox_FL_layout = QtGui.QHBoxLayout()
				groupbox_FL_layout.addWidget(self.button_video['FL'])
				groupbox_FL_layout.addWidget(self.button_settings)
				self.groupbox_FL.setLayout(groupbox_FL_layout)


		self.videoName = QtGui.QLineEdit()
		self.videoName.setPlaceholderText('Name of the next video folder')

		#Choice of the directory
		self.folder_path = os.getcwd()
		self.choose_directory=QtGui.QPushButton('Choose Directory')
		self.choose_directory.setIcon(QtGui.QIcon('icon/folder.png'))
		self.label_directory=QtGui.QLabel(self.folder_path)


		# Default ranges for the HSV sliders
		self.lower = [0,0,45]
		self.upper = [255,255,255]

		



		# GROUPBOX VIDEO SETTING
			
	   
		#sampling frequency
		self.label_fps_sampling = QtGui.QLabel('Sampling frequency')
		self.hslider_fps_sampling = QtGui.QSlider(QtCore.Qt.Horizontal)
		self.hslider_fps_sampling.setRange(1,200)
		self.hslider_fps_sampling.setValue(self.camera_functions[self.tracking_channel].fps_sampling)
		self.spinbox_fps_sampling=QtGui.QSpinBox()
		self.spinbox_fps_sampling.setRange(1,200)
		self.spinbox_fps_sampling.setValue(self.camera_functions[self.tracking_channel].fps_sampling)
		self.hslider_fps_sampling.valueChanged.connect(self.spinbox_fps_sampling.setValue)
		self.spinbox_fps_sampling.valueChanged.connect(self.hslider_fps_sampling.setValue)
		self.lcd_fps_sampling = QtGui.QLCDNumber()
		self.lcd_fps_sampling.setNumDigits(4)
		self.lcd_fps_sampling.display(self.camera_functions[self.tracking_channel].fps_sampling_real)
		slider_fps_sampling_layout=QtGui.QHBoxLayout()
		slider_fps_sampling_layout.addWidget(self.label_fps_sampling)
		slider_fps_sampling_layout.addWidget(self.hslider_fps_sampling)
		slider_fps_sampling_layout.addWidget(self.spinbox_fps_sampling)
		slider_fps_sampling_layout.addWidget(self.lcd_fps_sampling)
		group_slider_fps_sampling=QtWidgets.QWidget()
		group_slider_fps_sampling.setLayout(slider_fps_sampling_layout)
			  
  

		#Displaying frequency
		self.label_fps_displaying = QtGui.QLabel('Displaying frequency')
		self.hslider_fps_displaying = QtGui.QSlider(QtCore.Qt.Horizontal)
		self.hslider_fps_displaying.setRange(1,200)
		self.hslider_fps_displaying.setValue(self.camera_functions[self.tracking_channel].fps_displaying)
		self.spinbox_fps_displaying=QtGui.QSpinBox()
		self.spinbox_fps_displaying.setRange(1,200)
		self.spinbox_fps_displaying.setValue(self.camera_functions[self.tracking_channel].fps_displaying)
		self.hslider_fps_displaying.valueChanged.connect(self.spinbox_fps_displaying.setValue)
		self.spinbox_fps_displaying.valueChanged.connect(self.hslider_fps_displaying.setValue)
		self.lcd_fps_displaying = QtGui.QLCDNumber()
		self.lcd_fps_displaying.setNumDigits(4)
		self.lcd_fps_displaying.display(self.camera_functions[self.tracking_channel].fps_displaying_real)
		slider_fps_displaying_layout=QtGui.QHBoxLayout()
		slider_fps_displaying_layout.addWidget(self.label_fps_displaying)
		slider_fps_displaying_layout.addWidget(self.hslider_fps_displaying)
		slider_fps_displaying_layout.addWidget(self.spinbox_fps_displaying)
		slider_fps_displaying_layout.addWidget(self.lcd_fps_displaying)
		group_slider_fps_displaying=QtWidgets.QWidget()
		group_slider_fps_displaying.setLayout(slider_fps_displaying_layout)
		
		#Saving frequency
		self.label_fps_saving = QtGui.QLabel('Saving frequency')
		self.hslider_fps_saving = QtGui.QSlider(QtCore.Qt.Horizontal)
		self.hslider_fps_saving.setRange(1,200)
		self.hslider_fps_saving.setValue(self.camera_functions[self.tracking_channel].fps_saving)
		self.spinbox_fps_saving=QtGui.QSpinBox()
		self.spinbox_fps_saving.setRange(1,200)
		self.spinbox_fps_saving.setValue(self.camera_functions[self.tracking_channel].fps_saving)
		self.hslider_fps_saving.valueChanged.connect(self.spinbox_fps_saving.setValue)
		self.spinbox_fps_saving.valueChanged.connect(self.hslider_fps_saving.setValue)
		self.lcd_fps_saving = QtGui.QLCDNumber()
		self.lcd_fps_saving.setNumDigits(4)
		self.lcd_fps_saving.display(self.camera_functions[self.tracking_channel].fps_saving_real)
		slider_fps_saving_layout=QtGui.QHBoxLayout()
		slider_fps_saving_layout.addWidget(self.label_fps_saving)
		slider_fps_saving_layout.addWidget(self.hslider_fps_saving)
		slider_fps_saving_layout.addWidget(self.spinbox_fps_saving)
		slider_fps_saving_layout.addWidget(self.lcd_fps_saving)
		group_slider_fps_saving=QtWidgets.QWidget()
		group_slider_fps_saving.setLayout(slider_fps_saving_layout) 
		
		# resolution
		self.label_res = QtGui.QLabel('Working resolution (width)')
		self.hslider_res = QtGui.QSlider(QtCore.Qt.Horizontal)
		self.hslider_res.setRange(100,self.camera_functions[self.tracking_channel].maximal_width)
		self.hslider_res.setValue(self.camera_functions[self.tracking_channel].working_width)
		self.spinbox_res=QtGui.QSpinBox()
		self.spinbox_res.setRange(100,self.camera_functions[self.tracking_channel].maximal_width)
		self.spinbox_res.setValue(self.camera_functions[self.tracking_channel].working_width)
		self.hslider_res.valueChanged.connect(self.spinbox_res.setValue)
		self.spinbox_res.valueChanged.connect(self.hslider_res.setValue)
		slider_res_layout=QtGui.QHBoxLayout()
		slider_res_layout.addWidget(self.label_res)
		slider_res_layout.addWidget(self.hslider_res)
		slider_res_layout.addWidget(self.spinbox_res)
		group_slider_res=QtWidgets.QWidget()
		group_slider_res.setLayout(slider_res_layout)

		# Color/gray not being used anymore so replace with Optical path selection
		#color or gray 
		groupbox_type_image = QtGui.QGroupBox('Type of image')
		self.radiobutton_color = QtGui.QRadioButton('Colored image')
		self.radiobutton_gray = QtGui.QRadioButton('Grayscale image')
		self.radiobutton_color.setChecked(True)
		groupbox_layout_type_image = QtGui.QHBoxLayout()
		groupbox_layout_type_image.addWidget(self.radiobutton_color)
		groupbox_layout_type_image.addWidget(self.radiobutton_gray)
		groupbox_type_image.setLayout(groupbox_layout_type_image)

		# Dropdown menu to choose objective
		self.comboBox = QtGui.QComboBox()
		self.comboBox.addItems(self.objectives.keys())
		default_index = self.comboBox.findText(self.default_objective, QtCore.Qt.MatchFixedString)
		if default_index >= 0:
			self.comboBox.setCurrentIndex(default_index)
		# self.comboBox.setCurrentIndex(self.objectives.keys().index(self.default_objective))        
		
		groupbox_layout = QtGui.QVBoxLayout()
		groupbox_layout.addWidget(group_slider_fps_sampling)
		groupbox_layout.addWidget(group_slider_fps_displaying)
		groupbox_layout.addWidget(group_slider_fps_saving)
		groupbox_layout.addWidget(group_slider_res)
		groupbox_layout.addWidget(self.comboBox)
		#groupbox_layout.addWidget(groupbox_type_image)
		
		self.groupbox_video_setting = QtGui.QGroupBox('Video Setting')
		self.groupbox_video_setting.setLayout(groupbox_layout)

		# TRACKING PARAMETERS

		# Replace Image Setting with tracker type

		# Tracker Type

		self.groupbox_type_tracker = QtGui.QGroupBox('Tracker Type')
		self.radiobutton_nearestnbr = QtGui.QRadioButton('Nearest Nbr')
		self.radiobutton_csrt = QtGui.QRadioButton('CSRT')
		self.radiobutton_daSiam = QtGui.QRadioButton('daSiamRPN')
		self.radiobutton_csrt.setChecked(True)
		groupbox_layout_type_tracker = QtGui.QVBoxLayout()
		groupbox_layout_type_tracker.addWidget(self.radiobutton_nearestnbr)
		groupbox_layout_type_tracker.addWidget(self.radiobutton_csrt)
		groupbox_layout_type_tracker.addWidget(self.radiobutton_daSiam)
		self.groupbox_type_tracker.setLayout(groupbox_layout_type_tracker)

		# # GROUPBOX IMAGE SETTING
		
		# # Slider Groupe 1
		# self.label1 = QtGui.QLabel('Contrast')
		# self.hslider1 = QtGui.QSlider(QtCore.Qt.Horizontal)
		# self.hslider1.setRange(-50,150)
		# self.hslider1.setValue(0)
		# self.spinbox1=QtGui.QSpinBox()
		# self.spinbox1.setRange(-150,150)
		# self.spinbox1.setValue(0)
		# self.hslider1.valueChanged.connect(self.spinbox1.setValue)
		# self.spinbox1.valueChanged.connect(self.hslider1.setValue)
		# slider1_layout=QtGui.QHBoxLayout()
		# slider1_layout.addWidget(self.label1)
		# slider1_layout.addWidget(self.hslider1)
		# slider1_layout.addWidget(self.spinbox1)
		# group_slider1=QtWidgets.QWidget()
		# group_slider1.setLayout(slider1_layout)
		
		# # Slider Groupe 2
		# self.label2 = QtGui.QLabel('Brightness')
		# self.hslider2 = QtGui.QSlider(QtCore.Qt.Horizontal)
		# self.hslider2.setRange(-100,100)
		# self.hslider2.setValue(0)
		# self.spinbox2=QtGui.QSpinBox()
		# self.spinbox2.setRange(-100,100)
		# self.spinbox2.setValue(0)
		# self.hslider2.valueChanged.connect(self.spinbox2.setValue)
		# self.spinbox2.valueChanged.connect(self.hslider2.setValue)
		# slider2_layout=QtGui.QHBoxLayout()
		# slider2_layout.addWidget(self.label2)
		# slider2_layout.addWidget(self.hslider2)
		# slider2_layout.addWidget(self.spinbox2)
		# group_slider2=QtWidgets.QWidget()
		# group_slider2.setLayout(slider2_layout)
		
		# # Slider Groupe 3
		# self.label3 = QtGui.QLabel('Saturation')
		# self.hslider3 = QtGui.QSlider(QtCore.Qt.Horizontal)
		# self.hslider3.setRange(-100,100)
		# self.hslider3.setValue(0)
		# self.spinbox3=QtGui.QSpinBox()
		# self.spinbox3.setRange(-100,100)
		# self.spinbox3.setValue(0)
		# self.hslider3.valueChanged.connect(self.spinbox3.setValue)
		# self.spinbox3.valueChanged.connect(self.hslider3.setValue)
		# slider3_layout=QtGui.QHBoxLayout()
		# slider3_layout.addWidget(self.label3)
		# slider3_layout.addWidget(self.hslider3)
		# slider3_layout.addWidget(self.spinbox3)
		# group_slider3=QtWidgets.QWidget()
		# group_slider3.setLayout(slider3_layout)


		
		# # Big group
		# groupbox_layout = QtGui.QVBoxLayout()
		# groupbox_layout.addWidget(group_slider1)   
		# groupbox_layout.addWidget(group_slider2)
		# groupbox_layout.addWidget(group_slider3)
		# self.image_setting_box = QtGui.QGroupBox('Image Setting')
		# self.image_setting_box.setLayout(groupbox_layout)
		# self.image_setting_box.setCheckable(True)
		# self.image_setting_box.setChecked(self.camera_functions.image_setting)

		
		# checkable start tracking pushbutton
		self.button_tracking = QtGui.QPushButton('Start Tracking')
		self.button_tracking.setCheckable(True)
		self.button_tracking.setChecked(False)
		
		self.button_YTracking = QtGui.QPushButton('Start Y Tracking')
		self.button_YTracking.setCheckable(True)
		self.button_YTracking.setChecked(False)

		self.button_homing = QtGui.QPushButton('Launch Homing')

		

		# cropRatio
		self.label_crop_ratio = QtGui.QLabel('Cropping ratio')
		self.hslider_crop_ratio = QtGui.QSlider(QtCore.Qt.Horizontal)
		self.hslider_crop_ratio.setRange(1,50)
		self.hslider_crop_ratio.setValue(self.object_tracking.cropRatio)
		self.spinbox_crop_ratio=QtGui.QSpinBox()
		self.spinbox_crop_ratio.setRange(1,50)
		self.spinbox_crop_ratio.setValue(self.object_tracking.cropRatio)
		self.hslider_crop_ratio.valueChanged.connect(self.spinbox_crop_ratio.setValue)
		self.spinbox_crop_ratio.valueChanged.connect(self.hslider_crop_ratio.setValue)
		slider_crop_ratio_layout=QtGui.QHBoxLayout()
		slider_crop_ratio_layout.addWidget(self.label_crop_ratio)
		slider_crop_ratio_layout.addWidget(self.hslider_crop_ratio)
		slider_crop_ratio_layout.addWidget(self.spinbox_crop_ratio)
		group_slider_crop_ratio=QtWidgets.QWidget()
		group_slider_crop_ratio.setLayout(slider_crop_ratio_layout)
		
		#self.threshold_image_block=Threshold_image()
		self.threshold_image_block=self.thres_img_displayer_thread.threshold_image
		
		group_nb_tracks=QtWidgets.QWidget()
		layout_nb_tracks=QtGui.QHBoxLayout()
		self.label_nb_tracks=QtGui.QLabel('Number of tracked object')
		self.spinbox_nb_tracks = QtGui.QSpinBox()
		self.spinbox_nb_tracks.setValue(1)
		self.spinbox_nb_tracks.setEnabled(False)               #function desactivated for the moment
		layout_nb_tracks.addWidget(self.button_tracking)
		layout_nb_tracks.addWidget(self.button_YTracking)
		layout_nb_tracks.addWidget(self.button_homing)
		#layout_nb_tracks.addWidget(self.label_nb_tracks)
		#layout_nb_tracks.addWidget(self.spinbox_nb_tracks)
		group_nb_tracks.setLayout(layout_nb_tracks)
		
		#Y-TRACKING
		self.groupbox_YTracking = QtGui.QGroupBox('Y Tracking')
		
		# Liquid lens freq
		self.label_lensFreq = QtGui.QLabel('Liquid lens frequency (Hz)')
		self.hslider_lensFreq = QtGui.QSlider(QtCore.Qt.Horizontal)
		self.hslider_lensFreq.setRange(20,500)
		self.hslider_lensFreq.setValue(200)
		self.spinbox_lensFreq=QtGui.QDoubleSpinBox()
		self.spinbox_lensFreq.setRange(0.2,5)
		self.spinbox_lensFreq.setSingleStep(0.1)
		self.spinbox_lensFreq.setValue(2.0)
		self.hslider_lensFreq.valueChanged.connect(self.spinbox_lensFreq_setValue)
		self.spinbox_lensFreq.valueChanged.connect(self.hslider_lensFreq_setValue)
		slider_lensFreq_layout=QtGui.QHBoxLayout()
		slider_lensFreq_layout.addWidget(self.label_lensFreq)
		slider_lensFreq_layout.addWidget(self.hslider_lensFreq)
		slider_lensFreq_layout.addWidget(self.spinbox_lensFreq)
		group_slider_lensFreq=QtWidgets.QWidget()
		group_slider_lensFreq.setLayout(slider_lensFreq_layout)
		
		# Liquid lens amplitude
		self.label_lensAmpl = QtGui.QLabel('Liquid lens amplitude (mm)')
		self.hslider_lensAmpl = QtGui.QSlider(QtCore.Qt.Horizontal)
		self.hslider_lensAmpl.setRange(0,150)
		self.hslider_lensAmpl.setValue(2*100*self.object_tracking.liquid_lens_ampl)
		self.spinbox_lensAmpl=QtGui.QDoubleSpinBox()
		self.spinbox_lensAmpl.setRange(0,1.5)
		self.spinbox_lensAmpl.setSingleStep(0.01)
		self.spinbox_lensAmpl.setValue(2*self.object_tracking.liquid_lens_ampl)
		self.hslider_lensAmpl.valueChanged.connect(self.spinbox_lensAmpl_setValue)
		self.spinbox_lensAmpl.valueChanged.connect(self.hslider_lensAmpl_setValue)
		slider_lensAmpl_layout=QtGui.QHBoxLayout()
		slider_lensAmpl_layout.addWidget(self.label_lensAmpl)
		slider_lensAmpl_layout.addWidget(self.hslider_lensAmpl)
		slider_lensAmpl_layout.addWidget(self.spinbox_lensAmpl)
		group_slider_lensAmpl=QtWidgets.QWidget()
		group_slider_lensAmpl.setLayout(slider_lensAmpl_layout)

		# Liquid lens current value (For Optical charactetrization only)
		# self.label_lensAmpl = QtGui.QLabel('Liquid lens current value')
		# self.hslider_lensAmpl = QtGui.QSlider(QtCore.Qt.Horizontal)
		# self.hslider_lensAmpl.setRange(-4096,4096)
		# self.hslider_lensAmpl.setValue(0)
		# self.spinbox_lensAmpl=QtGui.QDoubleSpinBox()
		# self.spinbox_lensAmpl.setRange(-4096,4096)
		# self.spinbox_lensAmpl.setSingleStep(1)
		# self.spinbox_lensAmpl.setValue(0)
		# self.hslider_lensAmpl.valueChanged.connect(self.spinbox_lensAmpl_setValue)
		# self.spinbox_lensAmpl.valueChanged.connect(self.hslider_lensAmpl_setValue)
		# slider_lensAmpl_layout=QtGui.QHBoxLayout()
		# slider_lensAmpl_layout.addWidget(self.label_lensAmpl)
		# slider_lensAmpl_layout.addWidget(self.hslider_lensAmpl)
		# slider_lensAmpl_layout.addWidget(self.spinbox_lensAmpl)
		# group_slider_lensAmpl=QtWidgets.QWidget()
		# group_slider_lensAmpl.setLayout(slider_lensAmpl_layout)
		
		groupbox_layout_YTracking = QtGui.QVBoxLayout()
		groupbox_layout_YTracking.addWidget(group_slider_lensFreq) 
		groupbox_layout_YTracking.addWidget(group_slider_lensAmpl)
		# groupbox_layout_YTracking.addWidget(group_slider_lensGain) 
		self.groupbox_YTracking.setLayout(groupbox_layout_YTracking)

		#PARAMETERS FOR COLOR TRACKING
		self.group_color_picker=QtWidgets.QWidget()
		layout_color_picker=QtGui.QHBoxLayout()
		self.label_color_picker=QtGui.QLabel('Color of tracked object')
		self.color_picker=QColorEdit()
		layout_color_picker.addWidget(self.label_color_picker)
		layout_color_picker.addWidget(self.color_picker)
		self.group_color_picker.setLayout(layout_color_picker)
		self.group_color_picker.setEnabled(True)
		
		self.group_sliders=QtWidgets.QWidget()
		layout_sliders=QtGui.QGridLayout()
		
		self.label_Hue=QtGui.QLabel('Hue')
		self.range_slider1=rangeslider.QRangeSlider()
		self.range_slider1.setMax(255)
		self.label_Saturation=QtGui.QLabel('Saturation')
		self.range_slider2=rangeslider.QRangeSlider()
		self.range_slider2.setMax(255)
		self.label_Vibrance=QtGui.QLabel('Value')
		self.range_slider3=rangeslider.QRangeSlider()
		self.range_slider3.setMax(255)


		
		layout_sliders.addWidget(self.label_Hue,0,0,1,1)
		layout_sliders.addWidget(self.range_slider1,0,1,1,1)
		layout_sliders.addWidget(self.label_Saturation,1,0,1,1)
		layout_sliders.addWidget(self.range_slider2,1,1,1,1)
		layout_sliders.addWidget(self.label_Vibrance,2,0,1,1)
		layout_sliders.addWidget(self.range_slider3,2,1,1,1)
		self.group_sliders.setLayout(layout_sliders)
		self.group_sliders.setEnabled(True)

		# self.set_slider_defaults(LOWER=self.lower, UPPER = self.upper)
		
		groupbox_tracking_layout1 = QtGui.QGridLayout()
		groupbox_tracking_layout1.addWidget(group_nb_tracks,0,0,1,1)
		groupbox_tracking_layout1.addWidget(group_slider_crop_ratio,1,0,1,1)
		groupbox_tracking_layout1.addWidget(self.groupbox_YTracking,2,0,1,1)
		groupbox_tracking_layout1.addWidget(self.group_color_picker,3,0,1,1)
		groupbox_tracking_layout1.addWidget(self.threshold_image_block,0,1,4,1)
		groupbox_tracking_layout1.setColumnStretch(0,1)
		groupbox_tracking_layout1.setColumnStretch(1,1)
		
		groupbox_tracking_layout2 = QtGui.QVBoxLayout()
		groupbox_tracking_layout2.addLayout(groupbox_tracking_layout1)
		groupbox_tracking_layout2.addWidget(self.group_sliders)

		
		self.groupbox_tracking = QtGui.QGroupBox('Tracking parameters')
		self.groupbox_tracking.setLayout(groupbox_tracking_layout2)

		
		# VERTICAL LAYOUT ON THE LEFT
		vlayout_left = QtGui.QGridLayout()
		vlayout_left.addWidget(self.button_photo,0,0,1,1)
		vlayout_left.addWidget(self.photoName,0,1,1,1)
		vlayout_left.addWidget(self.button_video[self.tracking_channel],1,0,1,1)

		# Display the GroupBox for FL controls if it's enabled.


		
		vlayout_left.addWidget(self.videoName,1,1,1,1)
		vlayout_left.addWidget(self.choose_directory,2,0,1,1)
		vlayout_left.addWidget(self.label_directory,2,1,1,1)
		vlayout_left.addWidget(self.groupbox_video_setting,3,0,1,-1)
		if('FL' in self.imaging_channels):
			vlayout_left.addWidget(self.groupbox_FL,4,0,1,-1)
		# Light experiments are now a separate class
		# vlayout_left.addWidget(self.LEDPanel_box,4,0,1,-1)
		vlayout_left.addWidget(self.image_widget['DF'],5,0,1,-1)
		vlayout_left.setContentsMargins(10, 10, 10, 10)

		# VERTICAL LAYOUT ON THE RIGHT
		vlayout_right = QtGui.QGridLayout()
		vlayout_right.addWidget(self.groupbox_type_tracker,0,0,1,1)
		vlayout_right.addWidget(self.ZPID,0,1,1,1)
		vlayout_right.addWidget(self.YPID,0,2,1,1)
		vlayout_right.addWidget(self.XPID,0,3,1,1)
		vlayout_right.addWidget(self.groupbox_tracking,1,0,1,-1)
		vlayout_right.addWidget(self.plot_widget,2,0,-1,-1)
		vlayout_right.setContentsMargins(10, 10, 10, 10)

		# horizontal layout
		hlayout = QtGui.QHBoxLayout()
		hlayout.addLayout(vlayout_left)
		hlayout.addLayout(vlayout_right)
		hlayout.setStretchFactor(vlayout_right,1)
		hlayout.setStretchFactor(vlayout_left,1)

		# Final action     
		self.setLayout(hlayout)

		# FL image is displayed in a new window
		if('FL' in self.imaging_channels):
			self.image_widget['FL'].show()
		
	
		
	def button_video_clicked(self, channel = 'DF'):
		if self.button_video[channel].isChecked():
			self.button_video[channel].setText("Stop Acquisition")
		else:
			self.button_video[channel].setText("Record images")
			
	def pick_new_directory(self):
		dialog = QtGui.QFileDialog()
		self.folder_path = dialog.getExistingDirectory(None, "Select Folder")
		self.label_directory.setText(self.folder_path)
		
		
	def save_picture(self):
		self.camera_functions.photo_path=self.folder_path+'/'+self.photoName.text()+'.png'
		self.camera_functions.take_photo=True

		
	def save_video(self, channel = 'DF'):
		# Function that is called when we click the Save Video button. Which is the start of recording a dataset.
		if self.button_video[channel].isChecked():
			# directory=self.folder_path+'/'+self.videoName.text()

			print('Video Button Pressed for {} channel'.format(channel))
			# Set the trigger flag of the channel to True
			self.trigger_flag[channel] = True

			self.directory = os.path.join(self.folder_path, self.videoName.text())

			if (not os.path.exists(self.directory)):
				
				os.mkdir(self.directory)



				# Set the directory flag to True if called by the tracking channel stream
				if(channel == self.tracking_channel):
					self.directory_flag = True

			elif (os.path.exists(self.directory) and self.directory_flag == True ):
			# Case where tracking channel has already created the folder and we are now starting another video stream. 
			# In this case do nothing.
				pass

			else:
				self.button_video[channel].setChecked(False)
				QtWidgets.QMessageBox.information(self,'','The folder path already exist')
				return
				


			# Create a configuration file to save the optical config and pixel size and other metadata
			config_file = os.path.join(self.directory, 'metadata.csv')

			# If the file doesnt exist then create it
			if not os.path.exists(config_file):                                 
				
				df = pd.DataFrame({'Objective':[self.comboBox.currentText()], 
					'PixelPermm':[self.pixelsPermm],'Local time':[datetime.now().strftime('%Y-%m-%d %H:%M:%S')]})
				df.to_csv(config_file)



			# for channel in self.imaging_channels:
				# If we are making a new folder (new dataset), reset the images folder number 
			self.camera_functions[channel].image_folder_num = 0
			# Also reset the track counter and 
			self.object_tracking.trackCounter = 0

			# Reset the image number counter
			self.camera_functions[channel].saved_img_nb = 0
			# Also reset the local image number counter that counts from 0 to image_per_folder
			self.camera_functions[channel].saved_img_nb_local = 0

			if(channel == 'FL'):
				# For the FL channel we want to store images in a sub-folder

				# First create the sub-folder
				subPath = os.path.join(self.directory,channel)

				if(not os.path.exists(subPath)):
					os.mkdir(subPath)

				# Then create the folder for storing images
				images_path = os.path.join(subPath,'images{:05d}'.
					format(self.camera_functions[channel].image_folder_num))

				if(not os.path.exists(images_path)):
					os.mkdir(images_path)
			
				self.camera_functions[channel].video_root_path = os.path.join(self.directory, channel)

				self.camera_functions[channel].video_path= images_path

				self.camera_functions[channel].record_video = True

				
			else:
				images_path = os.path.join(self.directory,'images{:05d}'.
					format(self.camera_functions[channel].image_folder_num))

				if(not os.path.exists(images_path)):
					os.mkdir(images_path)
			
				self.camera_functions[channel].video_root_path = self.directory

				self.camera_functions[channel].video_path= images_path

				self.camera_functions[channel].record_video = True

					

			# Testing this out. I think we always want to have the start tracking to be ON when we record a video 
			# since otherwise the images that are recorded are without any time-stamp!

			# Setting start_tracking here would be an error since this will trigger the track function and csv writer without initializing the recent track file
			# self.object_tracking.start_tracking = True


			# Change the state of the tracking button
			self.button_tracking.setChecked(True)

			print('Start tracking flag: {}'.format(self.object_tracking.start_tracking))


			# Since we are setting the tracking button to checked the below code is not necessary
			# We only need to set the start_saving flag to TRUE
			self.object_tracking.start_saving = True
			# Start the tracking
			self.start_tracking()

			# In the previous implementation, the start_tracking needs to be ON before the Record to actually create a csv file
			# if self.object_tracking.start_tracking:

			# 	file_name=os.path.join(self.folder_path, self.videoName.text(), 'track{:03d}.csv'.format(self.object_tracking.trackCounter))

			# 	self.object_tracking.csv_register.file_directory=file_name
			# 	self.object_tracking.csv_register.start_write()
			# 	self.object_tracking.start_saving=True
			
				
				
		else:
			self.trigger_flag[channel] = False
			self.camera_functions[channel].record_video=False
			self.directory_flag = False
			# self.camera_functions.image_nb=0
			if self.object_tracking.start_saving:
				self.object_tracking.start_saving=False
				self.object_tracking.csv_register.close()
				
	def start_tracking(self): #launch the tracking + create a csv

		if self.button_tracking.isChecked():
			if self.object_tracking.start_saving:
				

				# Create a filename based on the current track number
				# file_name=os.path.join(self.folder_path, self.videoName.text(), 'track.csv')

				file_name=os.path.join(self.folder_path, self.videoName.text(), 'track{:03d}.csv'.format(self.object_tracking.trackCounter))

				print(file_name)
				#Update the track counter
				self.object_tracking.trackCounter += 1
				
				# If the file doesnt exist then create it
				if not os.path.exists(file_name):                                 #if it is the first time start_tracking is True while start_saving is true we initiate the new file
					self.object_tracking.csv_register.file_directory= file_name
					self.object_tracking.csv_register.start_write()
			
			if self.object_tracking.begining_Time==0:
				self.object_tracking.begining_Time=time.time()
			# Make sure the start_tracking flagis set to True if it already was not
			if not self.object_tracking.start_tracking:
				self.object_tracking.start_tracking=True
		else:
			self.object_tracking.start_tracking=False  #reinitialise time,buffer and plots
			
			self.object_tracking.start_Y_tracking=False

			self.button_YTracking.setChecked(False)
			self.button_YTracking.setText('Start Y-tracking')

			# Call the Y-tracking activation to send the state change of the button to the liquid lens
			self.YTrackingActivation()


			self.object_tracking.light_experiment = False


			self.plot_widget.initialise_plot_area()
			# This sets the tracker type based on the radio buttons.
			self.set_tracker_type()
			self.object_tracking.initialise_data()
			self.object_tracking.ytracker.initialise_ytracking()
	
			
	def set_fps_sampling(self):
		self.camera_functions[self.tracking_channel].fps_sampling=float(self.spinbox_fps_sampling.value())
		#self.camera_functions.timer.setInterval(1./self.camera_functions.fps_sampling*1000.)
		# self.set_Y_buffers_lenght()
		
		
	def set_fps_displaying(self):
		self.camera_functions[self.tracking_channel].fps_displaying=float(self.spinbox_fps_displaying.value())
		
	def set_fps_saving(self):
		self.camera_functions[self.tracking_channel].fps_saving=float(self.spinbox_fps_saving.value())
		
		
	def set_maximal_res(self,image_width):
		self.hslider_res.setRange(100,image_width)
		self.spinbox_res.setRange(100,image_width)
		
	def set_res(self,working_width):
		self.camera_functions[self.tracking_channel].working_width=self.hslider_res.value()
		
	def actualise_LED_panel(self,color):
		color_rgb=color.getRgb()
		self.object_tracking.arduino_led_panel.update_Color(color_rgb[0:3])
		self.object_tracking.LEDpanel_color=self.object_tracking.arduino_led_panel.color
		
	def activate_LED_panel(self,fps):
		# if self.button_LED_on.isChecked():
		# 	self.object_tracking.arduino_led_panel.setPanel_On()
		# 	self.object_tracking.LEDpanel_color=self.object_tracking.arduino_led_panel.color
		# else:
		# 	self.object_tracking.arduino_led_panel.setPanel_Off()
		# 	self.object_tracking.LEDpanel_color=[0,0,0]

		# DK: Modifying this function so that it controls the LED array through the same Arduino that controls the Wheel.
		if self.button_LED_on.isChecked():
			# Set the light experiment state variable to True
			self.object_tracking.light_experiment = True
			# Set the current Z-position as the reference position from which to calculate virtual depth
			self.object_tracking.ZobjWheel_init = self.object_tracking.ZobjWheel[-1]

			print('Beginning Light Modulation Experiment!')
		else:
			self.object_tracking.light_experiment = False



			
	def activate_LED_tracking(self,fps):
		if self.button_LED_on.isChecked():
			self.object_tracking.arduino_led_panel.activateTracking(True)
		else:
			self.object_tracking.arduino_led_panel.activateTracking(False)
		
	def actualise_LCD_panels(self,fps):
		self.lcd_fps_sampling.display(round(fps[0]))
		self.lcd_fps_displaying.display(round(fps[1]))
		self.lcd_fps_saving.display(round(fps[2]))


	def optical_path_choice(self, text):

		self.pixelsPermm = self.objectives[text]


		self.units_converter.update_pixel_size(self.pixelsPermm)

		print('{} objective chosen with pixelpermm {}'.format(text, self.units_converter.pixelPermm))


	


		
	def set_tracker_type(self):

		if(self.radiobutton_nearestnbr.isChecked()):
			self.object_tracking.tracker_type = "nearestnbr"
		elif(self.radiobutton_csrt.isChecked()):
			self.object_tracking.tracker_type = "csrt"
		elif(self.radiobutton_daSiam.isChecked()):
			self.object_tracking.tracker_type = "daSiamRPN"

		print(self.object_tracking.tracker_type)
		# If the tracker-type is modified on the fly, set the 
		self.object_tracking.createTracker()

		# Reset the tracking Flag so a new centroid if found and tracking starts based on that centroid
		self.object_tracking.flag = False

		
	def image_setting_action(self):
		self.camera_functions[self.tracking_channel].image_setting=self.image_setting_box.isChecked()
		
	def contrast(self):
		self.camera_functions[self.tracking_channel].contrast=self.hslider1.value()
		
	def brightness(self):
		self.camera_functions[self.tracking_channel].brightness=self.hslider2.value()
		
	def saturation(self):
		self.camera_functions[self.tracking_channel].saturation=self.hslider3.value()
			
	
	def set_slider_defaults(self, LOWER =[0,0,0], UPPER = [255,255,255]):

		LOWER=np.array(LOWER,dtype="uint8")
		UPPER=np.array(UPPER,dtype="uint8")

		self.range_slider1.setRange(LOWER[0],UPPER[0])
		self.range_slider2.setRange(LOWER[1],UPPER[1])
		self.range_slider3.setRange(LOWER[2],UPPER[2])

	def color_tracking(self,color):
		c=color.getRgb()
		color_RGB=[c[0],c[1],c[2]]
		color_HSV=cv2.cvtColor(np.uint8([[color_RGB]]), cv2.COLOR_RGB2HSV)[0][0]
		LOWER=image_processing.default_lower_HSV(color_HSV)
		UPPER=image_processing.default_upper_HSV(color_HSV)
		
		
		self.range_slider1.setRange(int(LOWER[0]),int(UPPER[0]))
		self.range_slider2.setRange(int(LOWER[1]),int(UPPER[1]))
		self.range_slider3.setRange(int(LOWER[2]),int(UPPER[2]))
		
		self.camera_functions[self.tracking_channel].lower_HSV=np.uint8(LOWER)
		self.camera_functions[self.tracking_channel].upper_HSV=np.uint8(UPPER)
		
	def sliders_move(self):
		LOWER=np.array([0,0,0],dtype="uint8")
		UPPER=np.array([255,255,255],dtype="uint8")
		
		LOWER[0],UPPER[0]=self.range_slider1.getRange()
		LOWER[1],UPPER[1]=self.range_slider2.getRange()
		LOWER[2],UPPER[2]=self.range_slider3.getRange()

		self.camera_functions[self.tracking_channel].lower_HSV=np.uint8(LOWER)
		# self.object_tracking.lower_HSV=np.uint8(LOWER)
		self.camera_functions[self.tracking_channel].upper_HSV=np.uint8(UPPER)
		# self.object_tracking.upper_HSV=np.uint8(UPPER		
	def launchHoming(self):
		self.object_tracking.arduino_wheel.launch_homing()
		self.homingComplete = True
		
	def tune_x_PID_P(self,value):
		self.object_tracking.tune_pid_x(value,-1,-1)
		
	def tune_x_PID_I(self,value):
		self.object_tracking.tune_pid_x(-1,value,-1)
		
	def tune_x_PID_D(self,value):
		self.object_tracking.tune_pid_x(-1,-1,value)

	def tune_y_PID_P(self,value):
		self.object_tracking.tune_pid_y(value,-1,-1)
		
	def tune_y_PID_I(self,value):
		self.object_tracking.tune_pid_y(-1,value,-1)
		
	def tune_y_PID_D(self,value):
		self.object_tracking.tune_pid_y(-1,-1,value)
		
	def tune_z_PID_P(self,value):
		self.object_tracking.tune_pid_z(value,-1,-1)
		
	def tune_z_PID_I(self,value):
		self.object_tracking.tune_pid_z(-1,value,-1)
		
	def tune_z_PID_D(self,value):
		self.object_tracking.tune_pid_z(-1,-1,value)
	
	def YTrackingActivation(self):
		
		if self.button_YTracking.isChecked():
			self.button_YTracking.setText("Stop Y Tracking")
			self.object_tracking.start_Y_tracking=True
			# For the optotune lens we now need to initialize the lens
			self.object_tracking.liquid_lens.start()
		else:
			self.button_YTracking.setText("Start Y Tracking")
			self.object_tracking.start_Y_tracking=False
			self.object_tracking.ytracker.initialise_ytracking()
			self.object_tracking.liquid_lens.stop()

	def set_Y_buffers_lenght(self):
		YdequeLen=round(float(self.camera_functions[self.tracking_channel].fps_sampling)/(self.object_tracking.liquid_lens_freq)) #to get to period of the liquid lens mvmt in the buffers
		self.object_tracking.ytracker.resize_buffers(YdequeLen)

	def spinbox_lensAmpl_setValue(self,value):
		newvalue=float(value)/100.
		self.spinbox_lensAmpl.setValue(newvalue)
		self.object_tracking.liquid_lens_ampl=newvalue/2
		self.object_tracking.ytracker.set_ampl(newvalue/2)
		# Now we need to also send the new amplitude to the liquid lens
		self.object_tracking.liquid_lens.changeAmp(newvalue/2)

	def hslider_lensAmpl_setValue(self,value):
		self.hslider_lensAmpl.setValue(int(value*100))

	# Setting liquid lens current functions (for optical characterization)
	# def spinbox_lensAmpl_setValue(self,value):
	# 	newvalue = int(value)
	# 	self.spinbox_lensAmpl.setValue(newvalue)
	# 	# self.object_tracking.liquid_lens_ampl=newvalue/2
	# 	# self.object_tracking.ytracker.set_ampl(newvalue/2)
	# 	# Now we need to also send the new amplitude to the liquid lens
	# 	# self.object_tracking.liquid_lens.changeAmp(newvalue/2)
	# 	self.object_tracking.liquid_lens.sendCurrent(newvalue)

	# def hslider_lensAmpl_setValue(self,value):
	# 	self.hslider_lensAmpl.setValue(int(value))

	def spinbox_lensFreq_setValue(self,value):
		newvalue=float(value)/100.
		self.spinbox_lensFreq.setValue(newvalue)
		self.object_tracking.liquid_lens_freq=newvalue 
		# self.set_Y_buffers_lenght()
		self.object_tracking.ytracker.set_freq(newvalue)
		# Now we need to also send the new frequency to the liquid lens

		self.object_tracking.liquid_lens.changeFreq(newvalue)

	def hslider_lensFreq_setValue(self,value):
		new_value = int(value*100)
		self.hslider_lensFreq.setValue(new_value)

	def spinbox_lensGain_setValue(self,value):
		newvalue=float(value)/100.
		self.spinbox_lensGain.setValue(newvalue)
		self.object_tracking.ytracker.set_maxGain(newvalue)

	def hslider_lensGain_setValue(self,value):
		self.hslider_lensGain.setValue(int(value*100))

	# def liquid_lensFreq(self,value):
	# 	self.object_tracking.liquid_lens_freq=value 
	# 	self.set_Y_buffers_lenght()
	# 	self.object_tracking.ytracker.set_freq(value)
			
	def set_cropRatio(self):
		self.object_tracking.cropRatio=self.spinbox_crop_ratio.value()
		
	def new_plot_data(self,plot_data):
		self.camera_functions[self.tracking_channel].plot_data=plot_data

	def set_isGrayscale(self):
		if self.radiobutton_color.isChecked():
			self.camera_functions[self.tracking_channel].isGrayscale=False
		else:
			self.camera_functions[self.tracking_channel].isGrayscale=True




	def connect_all(self):
		

		#save picture / video
		self.button_photo.clicked.connect(self.save_picture)
		for channel in self.imaging_channels:
			self.button_video[channel].clicked.connect(partial(self.button_video_clicked, channel))
			self.button_video[channel].clicked.connect(partial(self.save_video, channel))
		#Directory choice
		self.choose_directory.clicked.connect(self.pick_new_directory)
		#image setting
		# self.image_setting_box.clicked.connect(self.image_setting_action)
		# self.hslider1.valueChanged.connect(self.contrast)
		# self.hslider2.valueChanged.connect(self.brightness)
		# self.hslider3.valueChanged.connect(self.saturation)
		
		# Choice of objective
		self.comboBox.activated[str].connect(self.optical_path_choice)
		
		#Displaying fps
		self.camera_functions[self.tracking_channel].fps_data.connect(self.actualise_LCD_panels)
		
		self.spinbox_fps_sampling.valueChanged.connect(self.set_fps_sampling)
		self.spinbox_fps_displaying.valueChanged.connect(self.set_fps_displaying)
		self.spinbox_fps_saving.valueChanged.connect(self.set_fps_saving)
		
		self.hslider_res.valueChanged.connect(self.set_res)
		 
		self.button_tracking.clicked.connect(self.start_tracking)
		# This triggers the start_tracking function even if the start_tracking flag is changed manually in the code (or by a control signal from the Arduino)
		# self.button_tracking.checked.connect(self.start_tracking)

		# ALso change the state of Y-tracking when the tracking button is toggled
		# self.button_tracking.clicked.connect(self.YTrackingActivation)
		self.button_YTracking.clicked.connect(self.YTrackingActivation)
		self.button_homing.clicked.connect(self.launchHoming)
		
		self.spinbox_crop_ratio.valueChanged.connect(self.set_cropRatio)
		
		# self.hslider_lensFreq.valueChanged.connect(self.liquid_lensFreq)

		# self.spinbox_lensFreq.valueChanged.connect(self.spinbox_lensFreq_setValue)
		
		self.color_picker.colorChanged.connect(self.color_tracking)
		self.range_slider1.startValueChanged.connect(self.sliders_move)
		self.range_slider2.startValueChanged.connect(self.sliders_move)
		self.range_slider3.startValueChanged.connect(self.sliders_move)
		self.range_slider1.endValueChanged.connect(self.sliders_move)
		self.range_slider2.endValueChanged.connect(self.sliders_move)
		self.range_slider3.endValueChanged.connect(self.sliders_move)

		'''
		----------------------------------------------------------------
		# Connecting signals for image display
		----------------------------------------------------------------
		Connect each video stream to a corresponding display socket. Special case is the tracking video stream
		which gets connected to other widgets as well (eg. tracking, threshold image display etc.)
		'''
		#self.camera_functions.image_display.connect(self.image_widget.image_data_slot)

		# Image display for tracking channel
		self.camera_functions[self.tracking_channel].image_display.connect(self.img_displayer_thread[self.tracking_channel].display_new_img)
		
		# @@@ Image display for FL channel. This is explicit for now, will encapsulate in a list object later
		if('FL' in self.imaging_channels):
			self.camera_functions['FL'].image_display.connect(self.img_displayer_thread['FL'].display_new_img)

		self.camera_functions[self.tracking_channel].thresh_image_display.connect(self.threshold_image_block.thresh_data_slot)

		self.camera_functions[self.tracking_channel].thresh_image_display.connect(self.thres_img_displayer_thread.display_new_thresimg)
		# Connect the thresholded image to the object tracking function, so it is available for centroid detection (and avoid recomputing it!)
		self.camera_functions[self.tracking_channel].thresh_image_display.connect(self.object_tracking.returnThresholdImage)
		self.camera_functions[self.tracking_channel].image_data.connect(self.object_tracking.track)

		# @@@ Checking if this works, if not we need to implement separate functions for each channel
		# The partial method doesnt seem to work for this case.
	     
		self.camera_functions[self.tracking_channel].image_name.connect(self.object_tracking.setImageName)
		if('FL' in self.imaging_channels):
			self.camera_functions['FL'].image_name.connect(self.object_tracking.setImageName_FL)

		# Pass the FL image name to an appropriate function

		# self.camera_functions['FL'].image_name.connect(self.object_tracking.setImageName)

		# This sets the upper bound of the image resolution slider based on the image size received from camera
		self.camera_functions[self.tracking_channel].image_width.connect(self.set_maximal_res)
		
		#setting and clearing the track_busy flag (this is only relevant for the tracking stream)
		self.object_tracking.set_trackBusy.connect(self.camera_functions[self.tracking_channel].set_trackBusy)
		self.object_tracking.clear_trackBusy.connect(self.camera_functions[self.tracking_channel].clear_trackBusy)
		
		#draw circle / rectangle for tracking
		self.object_tracking.centroid_glob.connect(self.camera_functions[self.tracking_channel].draw_circle)
		self.object_tracking.Rect_pt1_pt2.connect(self.camera_functions[self.tracking_channel].draw_rectangle)
		#Displaying Plot
		self.object_tracking.plot_data.connect(self.new_plot_data)
		self.camera_functions[self.tracking_channel].plot_data_display.connect(self.plot_widget.plot1.update_plot)
		self.camera_functions[self.tracking_channel].plot_data_display.connect(self.plot_widget.plot2.update_plot)
		self.camera_functions[self.tracking_channel].plot_data_display.connect(self.plot_widget.plot3.update_plot)
		self.camera_functions[self.tracking_channel].plot_data_display.connect(self.plot_widget.plot4.update_plot)
		self.camera_functions[self.tracking_channel].plot_data_display.connect(self.plot_widget.plot5.update_plot)
		self.camera_functions[self.tracking_channel].plot_data_display.connect(self.plot_widget.plot6.update_plot)
		self.camera_functions[self.tracking_channel].plot_data_display.connect(self.plot_widget.plot7.update_plot)
		self.camera_functions[self.tracking_channel].plot_data_display.connect(self.plot_widget.plot8.update_plot)

		#LED PANEL
		# self.LED_color_picker.colorChanged.connect(self.actualise_LED_panel)
		# self.button_LED_on.clicked.connect(self.activate_LED_panel)
		# self.button_LED_tracking.clicked.connect(self.activate_LED_tracking)
		#Grayscale or colored image
		#self.radiobutton_color.clicked.connect(self.set_isGrayscale)
		#self.radiobutton_gray.clicked.connect(self.set_isGrayscale)
		self.radiobutton_nearestnbr.clicked.connect(self.set_tracker_type)
		self.radiobutton_csrt.clicked.connect(self.set_tracker_type)
		self.radiobutton_daSiam.clicked.connect(self.set_tracker_type)
		
		#PID
		self.XPID.spinboxP.valueChanged.connect(self.tune_x_PID_P)
		self.XPID.spinboxI.valueChanged.connect(self.tune_x_PID_I)
		self.XPID.spinboxD.valueChanged.connect(self.tune_x_PID_D)
		
		self.ZPID.spinboxP.valueChanged.connect(self.tune_z_PID_P)
		self.ZPID.spinboxI.valueChanged.connect(self.tune_z_PID_I)
		self.ZPID.spinboxD.valueChanged.connect(self.tune_z_PID_D)

		self.YPID.spinboxP.valueChanged.connect(self.tune_y_PID_P)
		self.YPID.spinboxI.valueChanged.connect(self.tune_y_PID_I)
		self.YPID.spinboxD.valueChanged.connect(self.tune_y_PID_D)

		# if(not self.homingComplete):
		#	self.launchHoming()



	

'''
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                            Main Window
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''
		
class MainWindowMill(QtWidgets.QMainWindow):
	
   
	def __init__(self):
		super().__init__()
		
		self.setWindowTitle('Gravity Machine Tracker')
		self.setWindowIcon(QtGui.QIcon('icon/icon.png'))
		self.statusBar().showMessage('Ready')
		
		
		#WIDGETS
		
		self.central_widget=CentralWidget()  
		self.setCentralWidget(self.central_widget)
		   
	
	  
	def closeEvent(self, event):
		
		reply = QtWidgets.QMessageBox.question(self, 'Message',
			"Are you sure you want to exit?", QtWidgets.QMessageBox.Yes | 
			QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Yes)

		if reply == QtWidgets.QMessageBox.Yes:

			for channel in self.central_widget.imaging_channels:
				self.central_widget.camera_functions[channel].stop()
			# cv2.destroyAllWindows()
			event.accept()
			sys.exit()
			
		else:
			event.ignore() 
			

'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
							 Main Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

if __name__ == '__main__':

	#To prevent the error "Kernel died"
	
	app = QtGui.QApplication.instance()
	if app is None:
	
		app = QtGui.QApplication(sys.argv)
	
	#Splash screen (image during the initialisation)
	splash_pix = QtGui.QPixmap('icon/icon.png')
	splash = QtGui.QSplashScreen(splash_pix, QtCore.Qt.WindowStaysOnTopHint)
	splash.setMask(splash_pix.mask())
	splash.show()
	
	

	#Mainwindow creation
	win= MainWindowMill()
	qss = QSSHelper.open_qss(os.path.join('aqua', 'aqua.qss'))
	win.setStyleSheet(qss)
	
	
	#connection and initialisation
	win.central_widget.connect_all()
	for channel in win.central_widget.imaging_channels:
		win.central_widget.camera_functions[channel].start_displaying()

		
	win.show()
	splash.finish(win)
	
	if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
		QtGui.QApplication.instance().exec_()
