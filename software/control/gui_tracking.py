# set QT_API environment variable
import os 
import sys
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *
import pyqtgraph.dockarea as dock

QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True) #enable highdpi scaling
QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True) #use highdpi icons

# Definitions
from control._def import *
import control.widgets as widgets
import control.widgets_tracking as widgets_tracking
import control.widgets_volumetric_imaging as widgets_volumetric_imaging
import control.camera as camera_Daheng
import control.core as core
import control.core_tracking as core_tracking
if VOLUMETRIC_IMAGING:
	import control.core_volumetric_imaging as core_volumetric_imaging
import control.microcontroller as microcontroller

class GUI(QMainWindow):

	def __init__(self, simulation = False, *args, **kwargs):
		super().__init__(*args, **kwargs)
		
		self.setWindowTitle('Gravity Machine v3.0')
		self.imaging_channels = CAMERAS.keys()
		print('Available imaging channels: {}'.format(self.imaging_channels))

		#------------------------------------------------------------------
		# Image Displays
		#------------------------------------------------------------------
		self.imageDisplayWindow = {}
		for key in self.imaging_channels:
			self.imageDisplayWindow[key] = core.ImageDisplayWindow(key + ' Display', DrawCrossHairs = True)
			if key == TRACKING:
				self.imageDisplayWindow[key].toggle_ROI_selector(True)
		self.imageDisplayWindow_ThresholdedImage = core.ImageDisplayWindow('Thresholded Image')
		
		#------------------------------------------------------------------
		# Load objects
		#------------------------------------------------------------------
		self.internal_state = core_tracking.InternalState()

		if simulation is True:
			# Define a camera object for each unique image-stream.
			self.camera = {key:camera_Daheng.Camera_Simulation() for key in self.imaging_channels}
			self.microcontroller = microcontroller.Microcontroller_Simulation()
		else:
			self.camera = {}
			for key in self.imaging_channels:
				if(CAMERAS[key]['make']=='TIS'):
					import control.camera_TIS as camera_TIS
					self.camera[key] = camera_TIS.Camera(serial=CAMERAS[key]['serial'], width = CAMERAS[key]['px_format'][0], 
						height = CAMERAS[key]['px_format'][1], framerate = CAMERAS[key]['fps'], color = CAMERAS[key]['is_color'])
				elif (CAMERAS[key]['make']=='Daheng'):
					self.camera[key] = camera_Daheng.Camera(sn = CAMERAS[key]['serial'])
				self.camera[key].open()
			self.microcontroller = microcontroller.Microcontroller()

		self.microcontroller.configure_actuators()

		self.streamHandler = {}
		self.imageSaver = {}
		for key in self.imaging_channels:
			# load stream handler
			if 'is_polarization' in CAMERAS[key]:
				if CAMERAS[key]['is_polarization']:
					is_polarization = True
				else:
					is_polarization = False
			else:
				is_polarization = False
			self.streamHandler[key] = core.StreamHandler(camera = self.camera[key], crop_width = CAMERAS[key]['px_format'][0], crop_height= CAMERAS[key]['px_format'][1], imaging_channel = key, 
                rotate_image_angle = CAMERAS[key]['rotate image angle'], flip_image = CAMERAS[key]['flip image'], is_polarization_camera = is_polarization)
			# load image saver
			self.imageSaver[key] = core_tracking.ImageSaver(self.internal_state, imaging_channel = key, image_format = IMAGE_FORMAT)

		self.liveController = {key:core.LiveController(self.camera[key],self.microcontroller) for key in self.imaging_channels}
		self.navigationController = core.NavigationController(self.microcontroller)
		self.stateUpdater = core_tracking.StateUpdater(self.navigationController, self.internal_state)
		self.microcontroller.set_callback(self.stateUpdater.read_microcontroller)
		self.trackingController = core_tracking.TrackingController(self.navigationController,self.microcontroller,self.internal_state)
		self.trackingDataSaver = core_tracking.TrackingDataSaver(self.internal_state)
		
		#------------------------------------------------------------------
		# load widgets
		#------------------------------------------------------------------
		self.cameraSettingsWidget = {key: widgets.CameraSettingsWidget(self.camera[key],self.liveController[key]) for key in self.imaging_channels}
		self.liveControlWidget = widgets.LiveControlWidget(self.streamHandler[TRACKING],self.liveController, self.internal_state)
		self.navigationWidget = widgets_tracking.NavigationWidget(self.navigationController, self.internal_state)
		self.trackingControlWidget = widgets_tracking.TrackingControllerWidget(self.streamHandler[TRACKING], self.trackingController, self.trackingDataSaver, self.internal_state, self.imageDisplayWindow[TRACKING], self.microcontroller)
		self.PID_Group_Widget = widgets_tracking.PID_Group_Widget(self.trackingController)
		self.recordingControlWidget = widgets.RecordingWidget(self.streamHandler,self.imageSaver, self.internal_state, self.trackingDataSaver, self.imaging_channels)			
		self.plotWidget = widgets.dockAreaPlot(self.internal_state)
		self.ledMatrixControlWidget = widgets.LEDMatrixControlWidget(self.microcontroller)
		
		self.liveSettings_Tab = QTabWidget()
		self.liveSettings_Tab.addTab(self.liveControlWidget, 'Live controller')
		for key in self.imaging_channels:
			self.liveSettings_Tab.addTab(self.cameraSettingsWidget[key],key)

		#------------------------------------------------------------------
		# Connections
		#------------------------------------------------------------------
		# Connections that involve all image streams
		for channel in self.imaging_channels:
			self.streamHandler[channel].signal_new_frame_received.connect(self.liveController[channel].on_new_frame)
			self.streamHandler[channel].image_to_display.connect(self.imageDisplayWindow[channel].display_image)
			self.streamHandler[channel].packet_image_to_write.connect(self.imageSaver[channel].enqueue)
			self.imageSaver[channel].imageName.connect(self.trackingDataSaver.setImageName)
			self.streamHandler[channel].signal_fps_save.connect(self.recordingControlWidget.update_save_fps)

		# Connections that involve only the tracking image stream
		self.streamHandler[TRACKING].thresh_image_to_display.connect(self.imageDisplayWindow_ThresholdedImage.display_image)
		self.streamHandler[TRACKING].packet_image_for_tracking.connect(self.trackingController.on_new_frame)
		# @@@ Currently the resolution-scaling only controls the TRACKING stream
		self.streamHandler[TRACKING].signal_working_resolution.connect(self.liveControlWidget.update_working_resolution)
		# Only display the image-display rate of the main/tracking image stream
		self.streamHandler[TRACKING].signal_fps_display.connect(self.liveControlWidget.update_display_fps)
		# self.trackingController.centroid_image.connect(self.imageDisplayWindow[TRACKING].draw_circle)
		self.trackingController.Rect_pt1_pt2.connect(self.imageDisplayWindow[TRACKING].draw_rectangle)
		self.trackingController.save_data_signal.connect(self.trackingDataSaver.enqueue)
		self.trackingController.signal_tracking_fps.connect(self.liveControlWidget.update_stream_fps)
		self.trackingController.signal_update_plots.connect(self.plotWidget.update_plots)

		for channel in self.imaging_channels:
			self.streamHandler[channel].signal_fps.connect(self.cameraSettingsWidget[channel].update_stream_fps)
		self.trackingController.get_roi_bbox.connect(self.imageDisplayWindow[TRACKING].send_bbox)
		self.imageDisplayWindow[TRACKING].roi_bbox.connect(self.trackingController.tracker_image.set_roi_bbox)
		self.trackingControlWidget.show_roi.connect(self.imageDisplayWindow[TRACKING].toggle_ROI_selector)
		self.recordingControlWidget.start_tracking_signal.connect(self.trackingControlWidget.slot_start_tracking)
		self.stateUpdater.signal_joystick_button_pressed.connect(self.trackingControlWidget.slot_joystick_button_pressed)
		self.stateUpdater.signal_stage_tracking_status_changed.connect(self.trackingControlWidget.slot_update_stage_tracking_status)
		self.liveControlWidget.signal_update_pixel_size.connect(self.trackingController.update_pixel_size)
		self.liveControlWidget._update_pixel_size()
		self.liveControlWidget.signal_update_image_resizing_factor.connect(self.trackingController.update_image_resizing_factor)
		self.liveControlWidget._update_image_resizing_factor()

		# tracking start/stop
		self.trackingController.signal_stop_tracking.connect(self.trackingControlWidget.slot_stop_tracking)
		self.trackingController.signal_stop_tracking.connect(self.streamHandler[TRACKING].stop_tracking)		
		
		#-----------------------------------------------------
		# Dock area for image display
		#-----------------------------------------------------
		self.image_window = QMainWindow()
		image_display_dockArea = dock.DockArea()
		self.image_window.setCentralWidget(image_display_dockArea)
		self.image_window.setWindowTitle('Image display')
		image_window_docks = dict()

		last_channel = None
		for channel in self.imaging_channels:
			image_window_docks[channel] = dock.Dock(channel, autoOrientation = False)
			image_window_docks[channel].setTitle(channel)
			image_window_docks[channel].showTitleBar()
			print(image_window_docks[channel].title())
			# image_window_docks[channel].setOrientation(o = 'vertical', force = True)
			# image_window_docks[channel].setStretch(x = 1000, y= 1000)
			image_display_dockArea.addDock(image_window_docks[channel], 'bottom')
			image_window_docks[channel].addWidget(self.imageDisplayWindow[channel].widget)
			last_channel = channel

		# Add dock for the thresholded image
		thresholded_image_dock = dock.Dock('Thresholded', autoOrientation = False)
		image_display_dockArea.addDock(thresholded_image_dock, 'right', image_window_docks[last_channel])
		thresholded_image_dock.addWidget(self.imageDisplayWindow_ThresholdedImage.widget)			

		#-----------------------------------------------------------------------------------------------
		# PDAF
		#-----------------------------------------------------------------------------------------------
		if TWO_CAMERA_PDAF:
			import control.core_PDAF as core_PDAF
			self.PDAFController = core_PDAF.PDAFController(self.trackingController)
			self.PDAFControllerWidget = widgets_tracking.PDAFControllerWidget(self.PDAFController)
			self.imageDisplayWindow['PDAF_image1'] = core.ImageDisplayWindow(key + ' Display', DrawCrossHairs = True) 
			self.imageDisplayWindow['PDAF_image2'] = core.ImageDisplayWindow(key + ' Display', DrawCrossHairs = True) 
			# add docked imaged display
			PDAF_image1_dock = dock.Dock('PDAF_image1', autoOrientation = False)
			image_display_dockArea.addDock(PDAF_image1_dock, 'bottom')
			PDAF_image1_dock.addWidget(self.imageDisplayWindow['PDAF_image1'].widget)
			PDAF_image2_dock = dock.Dock('PDAF_image2', autoOrientation = False)
			image_display_dockArea.addDock(PDAF_image2_dock, 'right', PDAF_image1_dock)
			PDAF_image2_dock.addWidget(self.imageDisplayWindow['PDAF_image2'].widget)
			# make connections
			self.streamHandler['DF1'].image_to_display.connect(self.PDAFController.register_image_from_camera_1) 
			self.streamHandler['DF2'].image_to_display.connect(self.PDAFController.register_image_from_camera_2) 
			self.PDAFController.signal_image1.connect(self.imageDisplayWindow['PDAF_image1'].display_image)
			self.PDAFController.signal_image2.connect(self.imageDisplayWindow['PDAF_image2'].display_image)

		#-----------------------------------------------------------------------------------------------
		# Volumetric Imaging
		#-----------------------------------------------------------------------------------------------
		import control.trigger_controller as trigger_controller
		import control.optotune_lens as optotune_lens
		if VOLUMETRIC_IMAGING:
			self.liquid_lens = optotune_lens.optotune_lens()
			if USE_SEPARATE_TRIGGER_CONTROLLER:
				if simulation:
					self.trigger_controller = trigger_controller.TriggerController_Simulation(TRIGGERCONTROLLER_SERIAL_NUMBER) 
				else:
					self.trigger_controller = trigger_controller.TriggerController(TRIGGERCONTROLLER_SERIAL_NUMBER) 
			else:
				self.trigger_controller = self.microcontroller
			self.volumetricImagingStreamHandler = core_volumetric_imaging.VolumetricImagingStreamHandler(self.trackingController, imaging_channel = 'volumetric imaging', rotate_image_angle = CAMERAS['volumetric imaging']['rotate image angle'], flip_image = CAMERAS['volumetric imaging']['flip image'])
			self.VolumetricImagingImageSaver = core_volumetric_imaging.VolumetricImagingImageSaver(self.internal_state)
			self.volumetricImagingController = core_volumetric_imaging.VolumetricImagingController(self.camera['volumetric imaging'],
				self.trigger_controller,self.liquid_lens,self.volumetricImagingStreamHandler,self.VolumetricImagingImageSaver,self.internal_state)
			self.volumetricImagingWidget = widgets_volumetric_imaging.VolumetricImagingWidget(self.volumetricImagingController)
			self.streamHandler['volumetric imaging'] = self.volumetricImagingStreamHandler
			self.imageSaver['volumetric imaging'].close()
			self.imageSaver['volumetric imaging'] = self.VolumetricImagingImageSaver
			self.focusMeasureDisplayWindow = widgets_volumetric_imaging.FocusMeasureDisplayWindow()
			self.focusMeasureDisplayWindow.show()
			self.imageArrayDisplayWindow = core_volumetric_imaging.ImageArrayDisplayWindow() 
			self.imageArrayDisplayWindow.show()
			self.volumetricImagingStreamHandler.packet_image_for_array_display.connect(self.imageArrayDisplayWindow.display_image)
			self.volumetricImagingStreamHandler.signal_focus_measure_plot.connect(self.focusMeasureDisplayWindow.plotWidget.plot)
			if simulation == False:
				# in simulation mode, do not change camera trigger mode (for now) or no new image would be delivered, as the trigger timer is in the livecontroller
				# to do: add a tigger timer in the simulation camera object for simulating hardware trigger
				self.volumetricImagingController.signal_trigger_mode.connect(self.cameraSettingsWidget['volumetric imaging'].set_trigger_mode)
			self.volumetricImagingStreamHandler.signal_defocus.connect(self.volumetricImagingWidget.display_defocus.display)
			self.streamHandler['volumetric imaging'].signal_new_frame_received.connect(self.liveController[channel].on_new_frame)
			self.streamHandler['volumetric imaging'].image_to_display.connect(self.imageDisplayWindow[channel].display_image)
			self.streamHandler['volumetric imaging'].packet_image_to_write.connect(self.imageSaver[channel].enqueue)
			self.imageSaver['volumetric imaging'].imageName.connect(self.trackingDataSaver.setImageName)
			self.streamHandler['volumetric imaging'].signal_fps_save.connect(self.recordingControlWidget.update_save_fps)

		#-----------------------------------------------------
		# Layout widgets
		#-----------------------------------------------------
		self.SettingsTab = QTabWidget()
		# self.SettingsTab.addTab(self.PID_Group_Widget, 'PID')
		if TWO_CAMERA_PDAF:
			self.SettingsTab.addTab(self.PDAFControllerWidget, 'PDAF')
		if VOLUMETRIC_IMAGING:
			self.SettingsTab.addTab(self.volumetricImagingWidget, 'Volumetric Imaging')
		self.SettingsTab.addTab(self.navigationWidget, 'Stage Control')
		self.SettingsTab.addTab(self.ledMatrixControlWidget, 'LED Matrix')
		self.SettingsTab.addTab(self.plotWidget, 'Plots')

		layout_left = QVBoxLayout()
		layout_left.addWidget(self.liveSettings_Tab)
		layout_left.addWidget(self.trackingControlWidget)
		layout_left.addWidget(self.recordingControlWidget)
		layout_left.addStretch()
		
		layout_right = QVBoxLayout()
		layout_right.addWidget(self.SettingsTab)
		layout_right.addWidget(self.PID_Group_Widget)
		
		overall_layout = QHBoxLayout()
		overall_layout.addLayout(layout_left)
		overall_layout.addLayout(layout_right)

		# transfer the layout to the central widget
		self.centralWidget = QWidget()
		self.centralWidget.setLayout(overall_layout)
		self.setCentralWidget(self.centralWidget)

		#-----------------------------------------------------
		# Start all image-streams
		#-----------------------------------------------------
		for channel in self.imaging_channels:
			self.camera[channel].set_software_triggered_acquisition() #self.camera.set_continuous_acquisition()
			self.camera[channel].set_callback(self.streamHandler[channel].on_new_frame)
			self.camera[channel].enable_callback()
			self.camera[channel].start_streaming()
		self.image_window.show()

	# @@@ TO DO
	def add_status_bar(self):
		# Status bar at the bottom
		self.statusBar = QStatusBar()
		self.homing_status = QLabel('Homing status:')
		self.stage_control = QLabel('Stage control:')

		self.statusBar.addPermanentWidget(self.homing_status)
		self.statusBar.addPermanentWidget(self.stage_control)

	# @@@ TO DO
	def update_status_bar(self):
		pass

	def closeEvent(self, event):
		reply = QMessageBox.question(self, 'Message',
			"Are you sure you want to exit?", QMessageBox.Yes | 
			QMessageBox.No, QMessageBox.Yes)
		if reply == QMessageBox.Yes:
			self.image_window.close()
			for key in self.imaging_channels:
				self.liveController[key].stop_live()
				self.camera[key].close()
				self.imageSaver[key].close()
				self.imageDisplayWindow[key].close()
			if TWO_CAMERA_PDAF:
				self.imageDisplayWindow['PDAF_image1'].close()
				self.imageDisplayWindow['PDAF_image2'].close()
			if VOLUMETRIC_IMAGING and USE_SEPARATE_TRIGGER_CONTROLLER:
				self.trigger_controller.close()
			if VOLUMETRIC_IMAGING:
				self.focusMeasureDisplayWindow.close()
				self.imageArrayDisplayWindow.close()
			self.trackingDataSaver.close()
			self.imageDisplayWindow_ThresholdedImage.close()
			self.microcontroller.close()
			event.accept()
		else:
			event.ignore() 
		