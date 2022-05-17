# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy
import numpy as np
import time

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

import pyqtgraph as pg
#import pyqtgraph.ptime as ptime
import pyqtgraph.dockarea as dock
from pyqtgraph.dockarea.Dock import DockLabel

from control._def import *
from control.utils import rangeslider as rangeslider

class TrackingControllerWidget(QFrame):
	'''
	Buttons to start image tracking
	Display window to show thresholded images
	Slider bars to threshold image
	Radio-buttons to choose trackers.
	Text boxes for base path and Experiment ID.

	'''
	show_roi = Signal(bool)
	signal_disable_offset_velocity = Signal()

	def __init__(self, streamHandler, trackingController, trackingDataSaver, internal_state, ImageDisplayWindow, microcontroller, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		# self.setTitle('Tracking Controller')
		self.base_path_is_set = False
		self.streamHandler = streamHandler
		self.trackingController = trackingController
		self.trackingDataSaver = trackingDataSaver
		self.internal_state = internal_state
		self.ImageDisplayWindow = ImageDisplayWindow
		self.microcontroller = microcontroller

		# self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)
		self.add_components()

		# Initialize states in underlying objects
		self.update_tracker_init_method()
		self.update_invert_image_flag()
		self.sliders_move()

	def add_components(self):
		# Image Tracking Button
		self.btn_track = QPushButton("Start Tracking")
		# self.btn_track.setStyleSheet('QPushButton {color: red;}')
		self.btn_track.setCheckable(True)
		self.btn_track.setChecked(False)
		self.btn_track.setDefault(False)
		self.btn_track.setIcon(QIcon('icon/track_white.png'))

		self.checkbox_enable_stage_tracking = QCheckBox(' Enable Stage Tracking')
		if USE_HARDWARE_SWITCH:
			self.checkbox_enable_stage_tracking.setEnabled(False)
		else:
			self.checkbox_enable_stage_tracking.setEnabled(True)
			self.checkbox_enable_stage_tracking.setChecked(True)
		self.checkbox_enable_stage_tracking.stateChanged.connect(self.toggle_stage_tracking)

		# Image Tracker Dropdown
		self.dropdown_TrackerSelection = QComboBox()
		self.dropdown_TrackerSelection.addItems(TRACKERS)
		self.dropdown_TrackerSelection.setCurrentText(DEFAULT_TRACKER)
		self.trackingController.tracker_image.update_tracker_type(self.dropdown_TrackerSelection.currentText())

		# Invert thresholded image checkbox (useful when switching between BF and DF)
		self.invert_image_checkbox = QCheckBox('Invert image')
		self.invert_image_checkbox.setChecked(False)

		self.tracking_init_threshold = QRadioButton("Threshold")
		self.tracking_init_roi = QRadioButton("ROI")
		self.tracking_init_threshold.setChecked(False)
		if DEFAULT_INIT_METHOD:
			self.tracking_init_roi.setChecked(True)

		# Layout
		self.tracking_init_group = QGroupBox('Tracking init method')
		self.tracking_init_layout = QGridLayout()
		self.tracking_init_layout.addWidget(self.tracking_init_threshold,0,0)
		self.tracking_init_layout.addWidget(self.tracking_init_roi,1,0)
		self.tracking_init_layout.addWidget(self.invert_image_checkbox,0,1)
		self.tracking_init_group.setLayout(self.tracking_init_layout)

		# Image offset settings
		self.label_x = QLabel('x (px)')
		# Image tracking offset - X axis
		self.tracking_setPoint_offset_x = QSpinBox()
		# self.tracking_setPoint_offset_x.setMinimum(-round(self.trackingController.image_width/4)) 
		# self.tracking_setPoint_offset_x.setMaximum(round(self.trackingController.image_width/4)) 
		self.tracking_setPoint_offset_x.setMinimum(-2000) # to change
		self.tracking_setPoint_offset_x.setMaximum(2000) # to change
		self.tracking_setPoint_offset_x.setSingleStep(1)
		self.tracking_setPoint_offset_x.setValue(0)

		# Image tracking offset - Z axis
		self.label_y = QLabel('z (px)')
		self.tracking_setPoint_offset_y = QSpinBox()
		# self.tracking_setPoint_offset_y.setMinimum(-round(self.trackingController.image_width/4)) 
		# self.tracking_setPoint_offset_y.setMaximum(round(self.trackingController.image_width/4))
		self.tracking_setPoint_offset_y.setMinimum(-2000) 
		self.tracking_setPoint_offset_y.setMaximum(2000)  
		self.tracking_setPoint_offset_y.setSingleStep(1)
		self.tracking_setPoint_offset_y.setValue(0)

		# Sliders for image segmentation
		self.label_Hue = QLabel('H')
		self.range_slider1 = rangeslider.QRangeSlider()
		self.range_slider1.setMax(255)
		self.label_Saturation=QLabel('S')
		self.range_slider2=rangeslider.QRangeSlider()
		self.range_slider2.setMax(255)
		self.label_Vibrance=QLabel('V')
		self.range_slider3=rangeslider.QRangeSlider()
		self.range_slider3.setMax(255)
		
		# Track button connection
		self.btn_track.clicked.connect(self.do_track_button_tasks)

		# Choose tracker
		self.dropdown_TrackerSelection.currentIndexChanged.connect(self.update_tracker)

		# Image tracking setpoint
		self.tracking_setPoint_offset_x.valueChanged.connect(self.update_tracking_setPoints)
		self.tracking_setPoint_offset_y.valueChanged.connect(self.update_tracking_setPoints)

		self.invert_image_checkbox.clicked.connect(self.update_invert_image_flag)

		self.tracking_init_threshold.clicked.connect(self.update_tracker_init_method)
		self.tracking_init_roi.clicked.connect(self.update_tracker_init_method)

		self.range_slider1.startValueChanged.connect(self.sliders_move)
		self.range_slider2.startValueChanged.connect(self.sliders_move)
		self.range_slider3.startValueChanged.connect(self.sliders_move)
		self.range_slider1.endValueChanged.connect(self.sliders_move)
		self.range_slider2.endValueChanged.connect(self.sliders_move)
		self.range_slider3.endValueChanged.connect(self.sliders_move)

		# Sub-blocks layout
		tracking_group_layout = QHBoxLayout()
		tracking_group_layout.addWidget(QLabel('Tracker selection'))
		tracking_group_layout.addWidget(self.dropdown_TrackerSelection)
		
		self.tracking_setPoint_group = QGroupBox('Tracking set-point offset', alignment = Qt.AlignCenter)
		tracking_setPoint_layout = QGridLayout()
		tracking_setPoint_layout.addWidget(self.label_x,0,0)
		tracking_setPoint_layout.addWidget(self.tracking_setPoint_offset_x,0,1)
		tracking_setPoint_layout.addWidget(self.label_y, 1,0)
		tracking_setPoint_layout.addWidget(self.tracking_setPoint_offset_y, 1,1)
		self.tracking_setPoint_group.setLayout(tracking_setPoint_layout)

		# Range sliders for image color thresholding
		self.group_sliders = QGroupBox('Threshold', alignment = Qt.AlignCenter)
		layout_sliders = QGridLayout()
		
		# layout_sliders.addWidget(self.label_Hue,0,0,1,1)
		# layout_sliders.addWidget(self.range_slider1,0,1,1,1)
		# layout_sliders.addWidget(self.label_Saturation,1,0,1,1)
		# layout_sliders.addWidget(self.range_slider2,1,1,1,1)
		layout_sliders.addWidget(self.label_Vibrance,2,0,1,1)
		layout_sliders.addWidget(self.range_slider3,2,1,1,1)
		self.group_sliders.setLayout(layout_sliders)
		self.group_sliders.setEnabled(True)

		# Overall layout
		groupbox_track_layout = QGridLayout()
		groupbox_track_layout.addWidget(self.btn_track, 0,0,2,2)
		# groupbox_track_layout.addWidget(self.dropdown_TrackerSelection, 0,1,1,1)
		groupbox_track_layout.addLayout(tracking_group_layout,2,0)
		groupbox_track_layout.addWidget(self.tracking_init_group,3,0)
		groupbox_track_layout.addWidget(self.checkbox_enable_stage_tracking,2,1)
		groupbox_track_layout.addWidget(self.tracking_setPoint_group,3,1)
		groupbox_track_layout.addWidget(self.group_sliders,4,0,1,2)
		self.setLayout(groupbox_track_layout)

	def do_track_button_tasks(self):
		if self.btn_track.isChecked():
			if self.internal_state.data['stage_tracking_enabled'] == True:
				# disable offset velocity when stage tracking is enabled
				# self.microcontroller.set_off_set_velocity_y(0) # to do: connect this to the offset velocity control widget
				self.signal_disable_offset_velocity.emit()
			self.internal_state.data['image_tracking_enabled'] = True
			if(self.tracking_init_roi.isChecked()):
				self.trackingController.update_roi_bbox()
			self.trackingController.reset_track()
			self.trackingDataSaver.start_new_track()
			self.streamHandler.start_tracking()
			self.btn_track.setText('Stop Tracking')
		else:
			self.btn_track.setText('Start Tracking')
			self.streamHandler.stop_tracking()
			self.internal_state.data['image_tracking_enabled'] = False
			print('stop tracking')

	def slot_start_tracking(self):
		# called after start recording is pressed
		if self.btn_track.isChecked():
			pass
		else:
			self.btn_track.setChecked(True)
			self.do_track_button_tasks()

	def slot_joystick_button_pressed(self):
		self.btn_track.toggle()
		self.do_track_button_tasks()

	def update_invert_image_flag(self):
		if(self.invert_image_checkbox.isChecked()):
			self.streamHandler.update_invert_image_flag(True)
		else:
			self.streamHandler.update_invert_image_flag(False)

	def update_tracker_init_method(self):
		if(self.tracking_init_threshold.isChecked()):
			self.trackingController.tracker_image.update_init_method("threshold")
			self.show_roi.emit(False)
		elif(self.tracking_init_roi.isChecked()):
			self.trackingController.tracker_image.update_init_method("roi")
			self.show_roi.emit(True)

	def update_tracker(self, index):
		self.trackingController.tracker_image.update_tracker_type(self.dropdown_TrackerSelection.currentText())

	def update_tracking_setPoints(self):
		value_x = self.tracking_setPoint_offset_x.value()
		value_y = self.tracking_setPoint_offset_y.value()

		self.trackingController.update_image_offset((value_x, value_y))
		'''
		Changing the tracking set point also changes the cross-hair location 
		displayed on the window (so a user can position the object precisely 
		where they want in the frame)
		'''
		self.ImageDisplayWindow.update_image_offset((value_x, value_y))

	def set_slider_defaults(self, LOWER =[0,0,0], UPPER = [255,255,255]):
		LOWER=np.array(LOWER,dtype="uint8")
		UPPER=np.array(UPPER,dtype="uint8")
		self.range_slider1.setRange(LOWER[0],UPPER[0])
		self.range_slider2.setRange(LOWER[1],UPPER[1])
		self.range_slider3.setRange(LOWER[2],UPPER[2])

	def sliders_move(self):
		LOWER=np.array([0,0,0],dtype="uint8")
		UPPER=np.array([255,255,255],dtype="uint8")
		LOWER[0],UPPER[0]=self.range_slider1.getRange()
		LOWER[1],UPPER[1]=self.range_slider2.getRange()
		LOWER[2],UPPER[2]=self.range_slider3.getRange()
		self.streamHandler.set_image_thresholds(np.uint8(LOWER), np.uint8(UPPER))	

	def slot_stop_tracking(self):
		self.btn_track.setChecked(False) # note that this will not cause the clicked signal to emit
		self.btn_track.setText('Start Tracking')

	def slot_update_stage_tracking_status(self):
		self.checkbox_enable_stage_tracking.setChecked(self.internal_state.data['stage_tracking_enabled'])

	def toggle_stage_tracking(self,enabled):
		self.internal_state.data['stage_tracking_enabled'] = enabled


class NavigationWidget(QFrame):
	def __init__(self, navigationController, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.navigationController = navigationController
		self.microcontroller = self.navigationController.microcontroller
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		self.label_Xpos = QLabel()
		self.label_Xpos.setNum(0)
		self.label_Xpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.entry_dX = QDoubleSpinBox()
		self.entry_dX.setMinimum(0) 
		self.entry_dX.setMaximum(5) 
		self.entry_dX.setSingleStep(0.2)
		self.entry_dX.setValue(0)
		self.btn_moveX_forward = QPushButton('Forward')
		self.btn_moveX_forward.setDefault(False)
		self.btn_moveX_backward = QPushButton('Backward')
		self.btn_moveX_backward.setDefault(False)
		self.btn_home_X = QPushButton('Home')
		self.btn_home_X.setDefault(False)
		self.btn_home_X.setEnabled(HOMING_ENABLED_X)
		self.btn_zero_X = QPushButton('Zero')
		self.btn_zero_X.setDefault(False)
		
		self.label_Ypos = QLabel()
		self.label_Ypos.setNum(0)
		self.label_Ypos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.entry_dY = QDoubleSpinBox()
		self.entry_dY.setMinimum(0)
		self.entry_dY.setMaximum(5)
		self.entry_dY.setSingleStep(0.2)
		self.entry_dY.setValue(0)
		self.btn_moveY_forward = QPushButton('Forward')
		self.btn_moveY_forward.setDefault(False)
		self.btn_moveY_backward = QPushButton('Backward')
		self.btn_moveY_backward.setDefault(False)
		self.btn_home_Y = QPushButton('Home')
		self.btn_home_Y.setDefault(False)
		self.btn_home_Y.setEnabled(HOMING_ENABLED_Y)
		self.btn_zero_Y = QPushButton('Zero')
		self.btn_zero_Y.setDefault(False)

		self.label_Zpos = QLabel()
		self.label_Zpos.setNum(0)
		self.label_Zpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.entry_dZ = QDoubleSpinBox()
		self.entry_dZ.setMinimum(0)
		self.entry_dZ.setMaximum(5)
		self.entry_dZ.setSingleStep(0.2)
		self.entry_dZ.setValue(0)
		self.btn_moveZ_forward = QPushButton('Forward')
		self.btn_moveZ_forward.setDefault(False)
		self.btn_moveZ_backward = QPushButton('Backward')
		self.btn_moveZ_backward.setDefault(False)
		self.btn_home_Z = QPushButton('Home')
		self.btn_home_Z.setDefault(False)
		self.btn_home_Z.setEnabled(HOMING_ENABLED_Z)
		self.btn_zero_Z = QPushButton('Zero')
		self.btn_zero_Z.setDefault(False)

		self.label_Thetapos = QLabel()
		self.label_Thetapos.setNum(0)
		self.label_Thetapos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.entry_dTheta = QDoubleSpinBox()
		self.entry_dTheta.setMinimum(0) 
		self.entry_dTheta.setMaximum(2*np.pi) 
		self.entry_dTheta.setSingleStep(0.01)
		self.entry_dTheta.setValue(0)
		self.btn_moveTheta_forward = QPushButton('Forward')
		self.btn_moveTheta_forward.setDefault(False)
		self.btn_moveTheta_backward = QPushButton('Backward')
		self.btn_moveTheta_backward.setDefault(False)
		self.btn_home_Theta = QPushButton('Home')
		self.btn_home_Theta.setDefault(False)
		self.btn_home_Theta.setEnabled(HOMING_ENABLED_THETA)
		self.btn_zero_Theta = QPushButton('Zero')
		self.btn_zero_Theta.setDefault(False)

		self.label_x_limit_pos = QLabel()
		self.label_x_limit_pos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_x_limit_pos.setText('+inf')
		self.label_x_limit_neg = QLabel()
		self.label_x_limit_neg.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_x_limit_neg.setText('-inf')
		self.label_y_limit_pos = QLabel()
		self.label_y_limit_pos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_y_limit_pos.setText('+inf')
		self.label_y_limit_neg = QLabel()
		self.label_y_limit_neg.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_y_limit_neg.setText('-inf')
		self.label_z_limit_pos = QLabel()
		self.label_z_limit_pos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_z_limit_pos.setText('+inf')
		self.label_z_limit_neg = QLabel()
		self.label_z_limit_neg.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_z_limit_neg.setText('-inf')

		self.btn_set_x_limit_pos = QPushButton('Set X Limit +')
		self.btn_set_x_limit_neg = QPushButton('Set X Limit -')
		self.btn_set_y_limit_pos = QPushButton('Set Y Limit +')
		self.btn_set_y_limit_neg = QPushButton('Set Y Limit -')
		self.btn_set_z_limit_pos = QPushButton('Set Y Limit +')
		self.btn_set_z_limit_neg = QPushButton('Set Y Limit -')
		self.btn_clear_x_limits = QPushButton('Clear')
		self.btn_clear_y_limits = QPushButton('Clear')
		self.btn_clear_z_limits = QPushButton('Clear')
		
		grid_line0 = QGridLayout()
		grid_line0.addWidget(QLabel('X (mm)'), 0,0)
		grid_line0.addWidget(self.label_Xpos, 0,1)
		grid_line0.addWidget(self.entry_dX, 0,2)
		grid_line0.addWidget(self.btn_moveX_forward, 0,3)
		grid_line0.addWidget(self.btn_moveX_backward, 0,4)
		grid_line0.addWidget(self.btn_home_X, 0,5)
		grid_line0.addWidget(self.btn_zero_X, 0,6)

		grid_line1 = QGridLayout()
		grid_line1.addWidget(QLabel('Y (mm)'), 0,0)
		grid_line1.addWidget(self.label_Ypos, 0,1)
		grid_line1.addWidget(self.entry_dY, 0,2)
		grid_line1.addWidget(self.btn_moveY_forward, 0,3)
		grid_line1.addWidget(self.btn_moveY_backward, 0,4)
		grid_line1.addWidget(self.btn_home_Y, 0,5)
		grid_line1.addWidget(self.btn_zero_Y, 0,6)

		if TRACKING_CONFIG == 'XTheta_Y':
			grid_line2 = QGridLayout()
			grid_line2.addWidget(QLabel('Theta (degree)'), 0,0)
			grid_line2.addWidget(self.label_Thetapos, 0,1)
			grid_line2.addWidget(self.entry_dTheta, 0,2)
			grid_line2.addWidget(self.btn_moveTheta_forward, 0,3)
			grid_line2.addWidget(self.btn_moveTheta_backward, 0,4)
			grid_line2.addWidget(self.btn_home_Theta, 0,5)
			grid_line2.addWidget(self.btn_zero_Theta, 0,6)
		else:
			grid_line2 = QGridLayout()
			grid_line2.addWidget(QLabel('Z (mm)'), 0,0)
			grid_line2.addWidget(self.label_Zpos, 0,1)
			grid_line2.addWidget(self.entry_dZ, 0,2)
			grid_line2.addWidget(self.btn_moveZ_forward, 0,3)
			grid_line2.addWidget(self.btn_moveZ_backward, 0,4)
			grid_line2.addWidget(self.btn_home_Z, 0,5)
			grid_line2.addWidget(self.btn_zero_Z, 0,6)

		grid_line4 = QGridLayout()
		grid_line4.addWidget(QLabel('X Limit -'), 0,0)
		grid_line4.addWidget(self.label_x_limit_neg, 0,1)
		grid_line4.addWidget(QLabel('X Limit +'), 0,2)
		grid_line4.addWidget(self.label_x_limit_pos, 0,3)
		grid_line4.addWidget(self.btn_set_x_limit_neg, 0,4)
		grid_line4.addWidget(self.btn_set_x_limit_pos, 0,5)
		grid_line4.addWidget(self.btn_clear_x_limits, 0,6)
		grid_line4.addWidget(QLabel('Y Limit -'), 1,0)
		grid_line4.addWidget(self.label_y_limit_neg, 1,1)
		grid_line4.addWidget(QLabel('Y Limit +'), 1,2)
		grid_line4.addWidget(self.label_y_limit_pos, 1,3)
		grid_line4.addWidget(self.btn_set_y_limit_neg, 1,4)
		grid_line4.addWidget(self.btn_set_y_limit_pos, 1,5)
		grid_line4.addWidget(self.btn_clear_y_limits, 1,6)
		if TRACKING_CONFIG == 'XZ_Y' or TRACKING_CONFIG == 'XY_Z':
			grid_line4.addWidget(QLabel('Z Limit -'), 2,0)
			grid_line4.addWidget(self.label_z_limit_neg, 2,1)
			grid_line4.addWidget(QLabel('Z Limit +'), 2,2)
			grid_line4.addWidget(self.label_z_limit_pos, 2,3)
			grid_line4.addWidget(self.btn_set_z_limit_neg, 2,4)
			grid_line4.addWidget(self.btn_set_z_limit_pos, 2,5)
			grid_line4.addWidget(self.btn_clear_z_limits, 2,6)

		# add offset velocity
		if TRACKING_CONFIG == 'XTheta_Y' or TRACKING_CONFIG == 'XZ_Y':
			grid_line5 = QHBoxLayout()
			self.entry_vz = QDoubleSpinBox()
			self.entry_vz.setMinimum(-25)
			self.entry_vz.setMaximum(25)
			self.entry_vz.setSingleStep(0.001)
			self.entry_vz.setValue(0)
			self.entry_vz.setDecimals(4)
			self.entry_vz.setKeyboardTracking(False)
			self.entry_vz.valueChanged.connect(self.update_offset_velocity_y)
			self.btn_vz_enable = QPushButton('Enable/Disable Offset Velocity')
			self.btn_vz_enable.setDefault(False)
			self.btn_vz_enable.setCheckable(True)
			self.btn_vz_enable.clicked.connect(self.enable_offset_velocity)
			grid_line5.addWidget(QLabel('Z offset velocity (mm/s)'))
			grid_line5.addWidget(self.entry_vz)
			grid_line5.addWidget(self.btn_vz_enable)

		self.grid = QGridLayout()
		self.grid.addLayout(grid_line0,0,0)
		self.grid.addLayout(grid_line1,1,0)
		self.grid.addLayout(grid_line2,2,0)
		self.grid.addLayout(grid_line4,4,0)
		# add offset velocity
		if TRACKING_CONFIG == 'XTheta_Y':
			self.grid.addLayout(grid_line5,5,0)
		self.setLayout(self.grid)

		self.btn_moveX_forward.clicked.connect(self.move_x_forward)
		self.btn_moveX_backward.clicked.connect(self.move_x_backward)
		self.btn_moveY_forward.clicked.connect(self.move_y_forward)
		self.btn_moveY_backward.clicked.connect(self.move_y_backward)
		self.btn_moveZ_forward.clicked.connect(self.move_z_forward)
		self.btn_moveZ_backward.clicked.connect(self.move_z_backward)
		self.btn_moveTheta_forward.clicked.connect(self.move_theta_forward)
		self.btn_moveTheta_backward.clicked.connect(self.move_theta_backward)

		self.btn_home_X.clicked.connect(self.home_x)
		self.btn_home_Y.clicked.connect(self.home_y)
		self.btn_home_Z.clicked.connect(self.home_z)
		self.btn_home_Theta.clicked.connect(self.home_theta)
		self.btn_zero_X.clicked.connect(self.zero_x)
		self.btn_zero_Y.clicked.connect(self.zero_y)
		self.btn_zero_Z.clicked.connect(self.zero_z)
		self.btn_zero_Theta.clicked.connect(self.zero_theta)

		self.navigationController.signal_x_mm.connect(self.update_x_display)
		self.navigationController.signal_y_mm.connect(self.update_y_display)
		self.navigationController.signal_z_mm.connect(self.update_z_display)
		self.navigationController.signal_theta_degree.connect(self.update_theta_display)

		self.btn_set_x_limit_neg.clicked.connect(self.set_x_limit_neg)
		self.btn_set_x_limit_pos.clicked.connect(self.set_x_limit_pos)
		self.btn_set_y_limit_neg.clicked.connect(self.set_y_limit_neg)
		self.btn_set_y_limit_pos.clicked.connect(self.set_y_limit_pos)
		self.btn_set_z_limit_neg.clicked.connect(self.set_z_limit_neg)
		self.btn_set_z_limit_pos.clicked.connect(self.set_z_limit_pos)

		self.btn_clear_x_limits.clicked.connect(self.clear_x_limits)
		self.btn_clear_y_limits.clicked.connect(self.clear_y_limits)
		self.btn_clear_z_limits.clicked.connect(self.clear_z_limits)
		
	def move_x_forward(self):
		dx_mm = self.entry_dX.value()
		dx_usteps = dx_mm/(SCREW_PITCH_X_MM/(self.navigationController.x_microstepping*FULLSTEPS_PER_REV_X))
		self.navigationController.move_x_usteps(dx_usteps)
	def move_x_backward(self):
		dx_mm = self.entry_dX.value()
		dx_usteps = dx_mm/(SCREW_PITCH_X_MM/(self.navigationController.x_microstepping*FULLSTEPS_PER_REV_X))
		self.navigationController.move_x_usteps(-dx_usteps)
	def home_x(self):
		msg = QMessageBox()
		msg.setIcon(QMessageBox.Information)
		msg.setText("Confirm your action")
		msg.setInformativeText("Click OK to run homing")
		msg.setWindowTitle("Confirmation")
		msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		msg.setDefaultButton(QMessageBox.Cancel)
		retval = msg.exec_()
		if QMessageBox.Ok == retval:
			self.navigationController.home_x()
	def zero_x(self):
		msg = QMessageBox()
		msg.setIcon(QMessageBox.Information)
		msg.setText("Confirm your action")
		msg.setInformativeText("Click OK to zero")
		msg.setWindowTitle("Confirmation")
		msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		msg.setDefaultButton(QMessageBox.Cancel)
		retval = msg.exec_()
		if QMessageBox.Ok == retval:
			self.navigationController.zero_x()
	
	def move_y_forward(self):
		dy_mm = self.entry_dY.value()
		if TRACKING_CONFIG == 'XY_Z':
			dy_usteps = dy_mm/(SCREW_PITCH_Y_MM/(self.navigationController.y_microstepping*FULLSTEPS_PER_REV_Y))
			self.navigationController.move_y_usteps(dy_usteps)
		else: # 'XZ_Y' or 'XTheta_Y'
			dz_usteps = dy_mm/(SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z))
			self.navigationController.move_z_usteps(dz_usteps)
	def move_y_backward(self):
		dy_mm = self.entry_dY.value()
		if TRACKING_CONFIG == 'XY_Z':
			dy_usteps = dy_mm/(SCREW_PITCH_Y_MM/(self.navigationController.y_microstepping*FULLSTEPS_PER_REV_Y))
			self.navigationController.move_y_usteps(-dy_usteps)
		else: # 'XZ_Y' or 'XTheta_Y'
			dz_usteps = dy_mm/(SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z))
			self.navigationController.move_z_usteps(-dz_usteps)
	def home_y(self):
		msg = QMessageBox()
		msg.setIcon(QMessageBox.Information)
		msg.setText("Confirm your action")
		msg.setInformativeText("Click OK to run homing")
		msg.setWindowTitle("Confirmation")
		msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		msg.setDefaultButton(QMessageBox.Cancel)
		retval = msg.exec_()
		if QMessageBox.Ok == retval:
			if TRACKING_CONFIG == 'XY_Z':
				self.navigationController.home_y()
			else:
				self.navigationController.home_z() # y is the focus axis (driver stack z)
	def zero_y(self):
		msg = QMessageBox()
		msg.setIcon(QMessageBox.Information)
		msg.setText("Confirm your action")
		msg.setInformativeText("Click OK to zero")
		msg.setWindowTitle("Confirmation")
		msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		msg.setDefaultButton(QMessageBox.Cancel)
		retval = msg.exec_()
		if QMessageBox.Ok == retval:
			if TRACKING_CONFIG == 'XY_Z':
				self.navigationController.zero_y()
			else:
				self.navigationController.zero_z() # y is the focus axis (driver stack z)

	def move_z_forward(self):
		dz_mm = self.entry_dZ.value()
		if TRACKING_CONFIG == 'XY_Z':
			dz_usteps = self.entry_dZ.value()/(SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z))
			self.navigationController.move_z_usteps(dz_usteps)
		elif TRACKING_CONFIG == 'XZ_Y':
			dy_usteps = self.entry_dZ.value()/(SCREW_PITCH_Y_MM/(self.navigationController.y_microstepping*FULLSTEPS_PER_REV_Y))
			self.navigationController.move_y_usteps(dy_usteps)
	def move_z_backward(self):
		dz_mm = self.entry_dZ.value()
		if TRACKING_CONFIG == 'XY_Z':
			dz_usteps = self.entry_dZ.value()/(SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z))
			self.navigationController.move_z_usteps(-dz_usteps)
		elif TRACKING_CONFIG == 'XZ_Y':
			dy_usteps = self.entry_dZ.value()/(SCREW_PITCH_Y_MM/(self.navigationController.y_microstepping*FULLSTEPS_PER_REV_Y))
			self.navigationController.move_y_usteps(-dy_usteps)
	def home_z(self):
		msg = QMessageBox()
		msg.setIcon(QMessageBox.Information)
		msg.setText("Confirm your action")
		msg.setInformativeText("Click OK to run homing")
		msg.setWindowTitle("Confirmation")
		msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		msg.setDefaultButton(QMessageBox.Cancel)
		retval = msg.exec_()
		if QMessageBox.Ok == retval:
			if TRACKING_CONFIG == 'XY_Z':
				self.navigationController.home_z()
			else:
				self.navigationController.home_y() # z is the in-plane axis (driver stack y)
	def zero_z(self):
		msg = QMessageBox()
		msg.setIcon(QMessageBox.Information)
		msg.setText("Confirm your action")
		msg.setInformativeText("Click OK to zero")
		msg.setWindowTitle("Confirmation")
		msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		msg.setDefaultButton(QMessageBox.Cancel)
		retval = msg.exec_()
		if QMessageBox.Ok == retval:
			if TRACKING_CONFIG == 'XY_Z':
				self.navigationController.zero_z()
			else:
				self.navigationController.zero_y() # z is the in-plane axis (driver stack y)

	def move_theta_forward(self):
		dTheta_degree = self.entry_dTheta.value()
		dTheta_rad = (dTheta_degree/360)*2*np.pi
		dTheta_usteps = dTheta_rad/((2*np.pi)/(FULLSTEPS_PER_REV_THETA*self.navigationController.theta_microstepping*GEAR_RATIO_THETA))
		self.navigationController.move_y_usteps(dTheta_usteps)
	def move_theta_backward(self):
		dTheta_degree = self.entry_dTheta.value()
		dTheta_rad = (dTheta_degree/360)*2*np.pi
		dTheta_usteps = dTheta_rad/((2*np.pi)/(FULLSTEPS_PER_REV_THETA*self.navigationController.theta_microstepping*GEAR_RATIO_THETA))
		self.navigationController.move_y_usteps(-dTheta_usteps)
	def home_theta(self):
		msg = QMessageBox()
		msg.setIcon(QMessageBox.Information)
		msg.setText("Confirm your action")
		msg.setInformativeText("Click OK to run homing")
		msg.setWindowTitle("Confirmation")
		msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		msg.setDefaultButton(QMessageBox.Cancel)
		retval = msg.exec_()
		if QMessageBox.Ok == retval:
			self.navigationController.home_y() # theta is the in-plane axis (driver stack y)
	def zero_theta(self):
		msg = QMessageBox()
		msg.setIcon(QMessageBox.Information)
		msg.setText("Confirm your action")
		msg.setInformativeText("Click OK to zero")
		msg.setWindowTitle("Confirmation")
		msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		msg.setDefaultButton(QMessageBox.Cancel)
		retval = msg.exec_()
		if QMessageBox.Ok == retval:
			self.navigationController.zero_y() # theta is the in-plane axis (driver stack y)

	def update_x_display(self,value):
		self.label_Xpos.setText('{:.03f}'.format(value))
	def update_y_display(self,value):
		self.label_Ypos.setText('{:.03f}'.format(value))
	def update_z_display(self,value):
		self.label_Zpos.setText('{:.03f}'.format(value))
	def update_theta_display(self,value):
		self.label_Thetapos.setText('{:.03f}'.format(value))

	def set_x_limit_neg(self):
		if STAGE_MOVEMENT_SIGN_X > 0:
			self.microcontroller.set_lim(LIMIT_CODE.X_NEGATIVE,self.microcontroller.x_pos)
			print('x negative limit: ' + str(self.microcontroller.x_pos))
		else:
			self.microcontroller.set_lim(LIMIT_CODE.X_POSITIVE,self.microcontroller.x_pos)
			print('x positive limit: ' + str(self.microcontroller.x_pos))
		self.label_x_limit_neg.setNum(float(self.label_Xpos.text()))

	def set_x_limit_pos(self):
		if STAGE_MOVEMENT_SIGN_X > 0:
			self.microcontroller.set_lim(LIMIT_CODE.X_POSITIVE,self.microcontroller.x_pos)
			print('x positive limit: ' + str(self.microcontroller.x_pos))
		else:
			self.microcontroller.set_lim(LIMIT_CODE.X_NEGATIVE,self.microcontroller.x_pos)
			print('x negative limit: ' + str(self.microcontroller.x_pos))
		self.label_x_limit_pos.setNum(float(self.label_Xpos.text()))

	def set_y_limit_neg(self):
		if TRACKING_CONFIG == 'XY_Z':
			if STAGE_MOVEMENT_SIGN_Y > 0:
				self.microcontroller.set_lim(LIMIT_CODE.Y_NEGATIVE,self.microcontroller.y_pos)
			else:
				self.microcontroller.set_lim(LIMIT_CODE.Y_POSITIVE,self.microcontroller.y_pos)
		else: # XZ_Y and XTheta_Y
			if STAGE_MOVEMENT_SIGN_Z > 0:
				self.microcontroller.set_lim(LIMIT_CODE.Z_NEGATIVE,self.microcontroller.z_pos)
			else:
				self.microcontroller.set_lim(LIMIT_CODE.Z_POSITIVE,self.microcontroller.z_pos)
		self.label_y_limit_neg.setNum(float(self.label_Ypos.text()))

	def set_y_limit_pos(self):
		if TRACKING_CONFIG == 'XY_Z':
			if STAGE_MOVEMENT_SIGN_Y > 0:
				self.microcontroller.set_lim(LIMIT_CODE.Y_POSITIVE,self.microcontroller.y_pos)
			else:
				self.microcontroller.set_lim(LIMIT_CODE.Y_NEGATIVE,self.microcontroller.y_pos)
		else: # XZ_Y and XTheta_Y
			if STAGE_MOVEMENT_SIGN_Z > 0:
				self.microcontroller.set_lim(LIMIT_CODE.Z_POSITIVE,self.microcontroller.z_pos)
			else:
				self.microcontroller.set_lim(LIMIT_CODE.Z_NEGATIVE,self.microcontroller.z_pos)
		self.label_y_limit_pos.setNum(float(self.label_Ypos.text()))

	def set_z_limit_neg(self):
		if TRACKING_CONFIG == 'XY_Z':
			if STAGE_MOVEMENT_SIGN_Z > 0:
				self.microcontroller.set_lim(LIMIT_CODE.Z_NEGATIVE,self.microcontroller.z_pos)
			else:
				self.microcontroller.set_lim(LIMIT_CODE.Z_POSITIVE,self.microcontroller.z_pos)
		else: # 'XZ_Y'
			if STAGE_MOVEMENT_SIGN_Y > 0:
				self.microcontroller.set_lim(LIMIT_CODE.Y_NEGATIVE,self.microcontroller.y_pos)
			else:
				self.microcontroller.set_lim(LIMIT_CODE.Y_POSITIVE,self.microcontroller.y_pos)
		self.label_z_limit_neg.setNum(float(self.label_Zpos.text()))

	def set_z_limit_pos(self):
		if TRACKING_CONFIG == 'XY_Z':
			if STAGE_MOVEMENT_SIGN_Z > 0:
				self.microcontroller.set_lim(LIMIT_CODE.Z_POSITIVE,self.microcontroller.z_pos)
			else:
				self.microcontroller.set_lim(LIMIT_CODE.Z_NEGATIVE,self.microcontroller.z_pos)
		else:
			if STAGE_MOVEMENT_SIGN_Y > 0:
				self.microcontroller.set_lim(LIMIT_CODE.Y_POSITIVE,self.microcontroller.y_pos)
			else:
				self.microcontroller.set_lim(LIMIT_CODE.Y_NEGATIVE,self.microcontroller.y_pos)
		self.label_z_limit_pos.setNum(float(self.label_Zpos.text()))

	def clear_x_limits(self):
		self.microcontroller.set_lim(LIMIT_CODE.X_NEGATIVE,-2147483648)
		self.microcontroller.set_lim(LIMIT_CODE.X_POSITIVE,2147483647)
		self.label_x_limit_neg.setText('-inf')
		self.label_x_limit_pos.setText('+inf')

	def clear_y_limits(self):
		if TRACKING_CONFIG == 'XY_Z':
			self.microcontroller.set_lim(LIMIT_CODE.Y_NEGATIVE,-2147483648)
			self.microcontroller.set_lim(LIMIT_CODE.Y_POSITIVE,2147483647)
		else: # 'XZ_Y' or 'XTheta_Y'
			self.microcontroller.set_lim(LIMIT_CODE.Z_NEGATIVE,-2147483648)
			self.microcontroller.set_lim(LIMIT_CODE.Z_POSITIVE,2147483647)
		self.label_y_limit_neg.setText('-inf')
		self.label_y_limit_pos.setText('+inf')

	def clear_z_limits(self):
		if TRACKING_CONFIG == 'XY_Z':
			self.microcontroller.set_lim(LIMIT_CODE.Z_NEGATIVE,-2147483648)
			self.microcontroller.set_lim(LIMIT_CODE.Z_POSITIVE,2147483647)
		else: # 'XZ_Y'
			self.microcontroller.set_lim(LIMIT_CODE.Y_NEGATIVE,-2147483648)
			self.microcontroller.set_lim(LIMIT_CODE.Y_POSITIVE,2147483647)
		self.label_z_limit_neg.setText('-inf')
		self.label_z_limit_pos.setText('+inf')

	def update_offset_velocity_y(self,offset_velocity):
		if self.btn_vz_enable.isChecked():
			self.microcontroller.set_off_set_velocity_y(offset_velocity)

	def enable_offset_velocity(self,pressed):
		if pressed:
			self.microcontroller.set_off_set_velocity_y(self.entry_vz.value())
		else:
			self.microcontroller.set_off_set_velocity_y(0)

	def slot_disable_offset_velocity(self):
		print('disable offset velocity')
		self.microcontroller.set_off_set_velocity_y(0)
		self.btn_vz_enable.setChecked(False)

class PID_Group_Widget(QFrame):
	def __init__(self, trackingController):
		super().__init__()
		# self.setTitle('PID settings')
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)
		self.trackingController = trackingController
		self.add_components()

	def add_components(self):
		self.PID_widget_x = PID_Widget('X')
		self.PID_widget_z = PID_Widget('Z')
		self.PID_widget_y = PID_Widget('Y')

		layout = QGridLayout()
		layout.addWidget(QLabel('X'),0,0,1,1)
		layout.addWidget(self.PID_widget_x,0,1,1,16)
		layout.addWidget(QLabel('Y'),1,0,1,1)
		layout.addWidget(self.PID_widget_y,1,1,1,16)
		layout.addWidget(QLabel('Z'),2,0,1,1)
		layout.addWidget(self.PID_widget_z,2,1,1,16)
		self.setLayout(layout)

		# Connections
		# X
		self.PID_widget_x.spinboxP.valueChanged.connect(self.trackingController.pid_controller_x.update_P)
		self.PID_widget_x.spinboxI.valueChanged.connect(self.trackingController.pid_controller_x.update_I)
		self.PID_widget_x.spinboxD.valueChanged.connect(self.trackingController.pid_controller_x.update_D)
		# Y
		self.PID_widget_y.spinboxP.valueChanged.connect(self.trackingController.pid_controller_y.update_P)
		self.PID_widget_y.spinboxI.valueChanged.connect(self.trackingController.pid_controller_y.update_I)
		self.PID_widget_y.spinboxD.valueChanged.connect(self.trackingController.pid_controller_y.update_D)
		# Theta
		self.PID_widget_z.spinboxP.valueChanged.connect(self.trackingController.pid_controller_z.update_P)
		self.PID_widget_z.spinboxI.valueChanged.connect(self.trackingController.pid_controller_z.update_I)
		self.PID_widget_z.spinboxD.valueChanged.connect(self.trackingController.pid_controller_z.update_D)

class PID_Widget(QFrame):
	
	def __init__(self,name,Pmax=2,Dmax=1,Imax=1):
		super().__init__()

		# Slider Groupe P
		defaultP = PID_parameters.P_DEFAULT
		stepP = Pmax/200

		self.labelP = QLabel('P')
		self.hsliderP = QSlider(Qt.Vertical)
		self.hsliderP.setRange(0,int(Pmax*100))
		self.hsliderP.setValue(int(defaultP*100))
		self.spinboxP = QDoubleSpinBox()
		self.spinboxP.setRange(0,round(Pmax,2))
		self.spinboxP.setSingleStep(round(stepP,3))
		self.spinboxP.setValue(round(defaultP,2))
		self.hsliderP.valueChanged.connect(self.spinBoxP_setValue)
		self.spinboxP.valueChanged.connect(self.hsliderP_setValue)
		sliderP_layout=QGridLayout()
		sliderP_layout.addWidget(self.labelP,0,0)
		sliderP_layout.addWidget(self.hsliderP,0,1,2,1)
		sliderP_layout.addWidget(self.spinboxP,1,0)
		group_sliderP=QWidget()
		group_sliderP.setLayout(sliderP_layout)

		defaultI = PID_parameters.I_DEFAULT
		stepI = Imax/200
		# Slider Groupe I
		self.labelI = QLabel('I')
		self.hsliderI = QSlider(Qt.Vertical)
		self.hsliderI.setRange(0,int(Imax*100))
		self.hsliderI.setValue(int(defaultI*100))
		self.spinboxI=QDoubleSpinBox()
		self.spinboxI.setSingleStep(round(stepI,2))
		self.spinboxI.setRange(0,int(Imax))
		self.spinboxI.setValue(round(defaultI,2))
		self.hsliderI.valueChanged.connect(self.spinBoxI_setValue)
		self.spinboxI.valueChanged.connect(self.hsliderI_setValue)
		sliderI_layout=QGridLayout()
		sliderI_layout.addWidget(self.labelI,0,0)
		sliderI_layout.addWidget(self.hsliderI,0,1,2,1)
		sliderI_layout.addWidget(self.spinboxI,1,0)
		group_sliderI=QWidget()
		group_sliderI.setLayout(sliderI_layout)
		
		# Slider Groupe D
		defaultD = PID_parameters.D_DEFAULT
		stepD = Dmax/200

		self.labelD = QLabel('D')
		self.hsliderD = QSlider(Qt.Vertical)
		self.hsliderD.setRange(0,int(Dmax*100))
		self.hsliderD.setValue(int(defaultD*100))
		self.spinboxD=QDoubleSpinBox()
		self.spinboxD.setRange(0,int(Dmax))
		self.spinboxD.setSingleStep(round(stepD,3))
		self.spinboxD.setValue(round(defaultD,2))
		self.hsliderD.valueChanged.connect(self.spinBoxD_setValue)
		self.spinboxD.valueChanged.connect(self.hsliderD_setValue)
		sliderD_layout=QGridLayout()
		sliderD_layout.addWidget(self.labelD,0,0)
		sliderD_layout.addWidget(self.hsliderD,0,1,2,1)
		sliderD_layout.addWidget(self.spinboxD,1,0)
		group_sliderD=QWidget()
		group_sliderD.setLayout(sliderD_layout)
		
		# Big PID group
		groupbox_layout_PID = QHBoxLayout()
		# groupbox_layout_PID.addWidget(QLabel(name))
		groupbox_layout_PID.addWidget(group_sliderP)
		groupbox_layout_PID.addWidget(group_sliderI)
		groupbox_layout_PID.addWidget(group_sliderD)
		self.setLayout(groupbox_layout_PID)
		self.setFrameStyle(QFrame.StyledPanel | QFrame.Plain)
	
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

class PDAFControllerWidget(QFrame):
	def __init__(self, PDAFController, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.PDAFController = PDAFController
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		self.entry_x_offset = QSpinBox()
		self.entry_x_offset.setMinimum(-1000) 
		self.entry_x_offset.setMaximum(1000) 
		self.entry_x_offset.setValue(PDAF.x_offset_default)

		self.entry_y_offset = QSpinBox()
		self.entry_y_offset.setMinimum(-1000) 
		self.entry_y_offset.setMaximum(1000)
		self.entry_y_offset.setValue(PDAF.y_offset_default)

		self.entry_ROI_ratio_width = QDoubleSpinBox()
		self.entry_ROI_ratio_width.setMinimum(0.5) 
		self.entry_ROI_ratio_width.setMaximum(10) 
		self.entry_ROI_ratio_width.setSingleStep(0.1)
		self.entry_ROI_ratio_width.setValue(PDAF.ROI_ratio_width_default)

		self.entry_ROI_ratio_height = QDoubleSpinBox()
		self.entry_ROI_ratio_height.setMinimum(0.5) 
		self.entry_ROI_ratio_height.setMaximum(10) 
		self.entry_ROI_ratio_height.setSingleStep(0.1)
		self.entry_ROI_ratio_height.setValue(PDAF.ROI_ratio_height_default)

		self.entry_shift_to_distance_um = QDoubleSpinBox()
		self.entry_shift_to_distance_um.setMinimum(-10) 
		self.entry_shift_to_distance_um.setMaximum(10) 
		self.entry_shift_to_distance_um.setSingleStep(0.1)
		self.entry_shift_to_distance_um.setValue(PDAF.shift_to_distance_um_default)

		self.entry_tracking_range_min_um = QDoubleSpinBox()
		self.entry_tracking_range_min_um.setMinimum(-10000) 
		self.entry_tracking_range_min_um.setMaximum(10000) 
		self.entry_tracking_range_min_um.setSingleStep(0.1)
		self.entry_tracking_range_min_um.setValue(-10000)

		self.entry_tracking_range_max_um = QDoubleSpinBox()
		self.entry_tracking_range_max_um.setMinimum(-10000) 
		self.entry_tracking_range_max_um.setMaximum(10000) 
		self.entry_tracking_range_max_um.setSingleStep(0.1)
		self.entry_tracking_range_max_um.setValue(10000)

		self.btn_enable_calculation = QPushButton('Enable Calculation')
		self.btn_enable_calculation.setCheckable(True)
		self.btn_enable_calculation.setChecked(False)
		self.btn_enable_calculation.setDefault(False)
		
		self.btn_enable_tracking = QPushButton('Enable Focus Tracking')
		self.btn_enable_tracking.setCheckable(True)
		self.btn_enable_tracking.setChecked(False)
		self.btn_enable_tracking.setDefault(False)

		# self.label_Shift = QLabel()
		# self.label_Shift.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		# self.label_Error = QLabel()
		# self.label_Error.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.display_defocus_um = QLCDNumber()
		self.display_defocus_um.setNumDigits(4)
		self.display_error = QLCDNumber()
		self.display_error.setNumDigits(4)
		
		grid_line0 = QGridLayout()
		grid_line0.addWidget(QLabel('X Crop Offset'), 0,0)
		grid_line0.addWidget(self.entry_x_offset, 0,1)
		grid_line0.addWidget(QLabel('Y Crop Offset'), 0,2)
		grid_line0.addWidget(self.entry_y_offset, 0,3)
		grid_line0.addWidget(QLabel('ROI Width Scaling'), 1,0)
		grid_line0.addWidget(self.entry_ROI_ratio_width, 1,1)
		grid_line0.addWidget(QLabel('ROI Height Scaling'), 1,2)
		grid_line0.addWidget(self.entry_ROI_ratio_height, 1,3)
		grid_line0.addWidget(self.btn_enable_calculation, 2,0,1,2)
		grid_line0.addWidget(self.btn_enable_tracking, 2,2,1,2)
		grid_line0.addWidget(QLabel('Defocus (um)'), 3,0,1,1)
		grid_line0.addWidget(self.display_defocus_um, 3,1,1,1)
		grid_line0.addWidget(QLabel('Error'), 3,2,1,1)
		grid_line0.addWidget(self.display_error, 3,3,1,1)

		self.grid = QGridLayout()
		self.grid.addLayout(grid_line0,0,0)

		self.grid2 = QGridLayout()
		self.grid2.addWidget(QLabel('tracking range min (um)'),0,0)
		self.grid2.addWidget(self.entry_tracking_range_min_um,0,1)
		self.grid2.addWidget(QLabel('tracking range max (um)'),0,2)
		self.grid2.addWidget(self.entry_tracking_range_max_um,0,3)
		
		vbox = QVBoxLayout()
		vbox.addLayout(self.grid)
		vbox.addStretch()
		vbox.addLayout(self.grid2)
		self.setLayout(vbox)

		# self.btn_enable_calculation.clicked.connect(self.PDAFController.enable_caculation)
		# self.btn_enable_tracking.clicked.connect(self.PDAFController.enable_tracking)
		self.entry_x_offset.valueChanged.connect(self.PDAFController.set_x_offset)
		self.entry_y_offset.valueChanged.connect(self.PDAFController.set_y_offset)
		self.entry_ROI_ratio_width.valueChanged.connect(self.PDAFController.set_ROI_ratio_width)
		self.entry_ROI_ratio_height.valueChanged.connect(self.PDAFController.set_ROI_ratio_height)
		self.entry_shift_to_distance_um.valueChanged.connect(self.PDAFController.set_ROI_ratio_height)

		self.btn_enable_calculation.clicked.connect(self.enable_caculation)
		self.btn_enable_tracking.clicked.connect(self.enable_tracking)

		self.PDAFController.signal_defocus_um_display.connect(self.display_defocus_um.display)
		self.PDAFController.signal_error.connect(self.display_error.display)

		self.entry_tracking_range_min_um.valueChanged.connect(self.PDAFController.set_defocus_um_for_enable_tracking_min)
		self.entry_tracking_range_max_um.valueChanged.connect(self.PDAFController.set_defocus_um_for_enable_tracking_max)

	def enable_caculation(self,pressed):
		if pressed:
			self.PDAFController.enable_caculation(True)
		else:
			if self.btn_enable_tracking.isChecked():
				self.btn_enable_calculation.setChecked(True)
			else:
				self.PDAFController.enable_caculation(False)

	def enable_tracking(self,pressed):
		if pressed:
			if self.btn_enable_calculation.isChecked() == False:
				self.btn_enable_calculation.setChecked(True)
				self.PDAFController.enable_caculation(True)
			self.PDAFController.enable_tracking(True)
		else:
			self.PDAFController.enable_tracking(False)
			