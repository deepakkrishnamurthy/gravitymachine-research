# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

import pyqtgraph as pg
import pyqtgraph.dockarea as dock
from pyqtgraph.dockarea.Dock import DockLabel

import control.utils.dockareaStyle as dstyle
import numpy as np
from collections import deque

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *

class CameraSettingsWidget(QFrame):
	
	def __init__(self, camera, liveController, main=None, *args, **kwargs):

		super().__init__(*args, **kwargs)
		self.camera = camera
		self.liveController = liveController
		self.fps_trigger = FPS['trigger_software']['default']

		# add components to self.grid
		self.add_components()		
		# set frame style
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		# add buttons and input fields
		self.entry_exposureTime = QDoubleSpinBox()
		self.entry_exposureTime.setMinimum(self.camera.EXPOSURE_TIME_MS_MIN) 
		self.entry_exposureTime.setMaximum(self.camera.EXPOSURE_TIME_MS_MAX) 
		self.entry_exposureTime.setSingleStep(1)
		self.entry_exposureTime.setValue(5)
		self.camera.set_exposure_time(5)

		self.entry_analogGain = QDoubleSpinBox()
		self.entry_analogGain.setMinimum(self.camera.GAIN_MIN) 
		self.entry_analogGain.setMaximum(self.camera.GAIN_MAX) 
		self.entry_analogGain.setSingleStep(self.camera.GAIN_STEP)
		self.entry_analogGain.setValue(0)
		self.camera.set_analog_gain(0)

		self.entry_exposureTime_Preset = QDoubleSpinBox()
		self.entry_exposureTime_Preset.setMinimum(self.camera.EXPOSURE_TIME_MS_MIN)
		self.entry_exposureTime_Preset.setMaximum(self.camera.EXPOSURE_TIME_MS_MAX)
		self.entry_exposureTime_Preset.setSingleStep(1)
		self.entry_exposureTime_Preset.setValue(20)
		# self.liveController.set_exposure_time_preset(20)

		self.entry_analogGain_Preset = QDoubleSpinBox()
		self.entry_analogGain_Preset.setMinimum(self.camera.GAIN_MIN) 
		self.entry_analogGain_Preset.setMaximum(self.camera.GAIN_MAX) 
		self.entry_analogGain_Preset.setSingleStep(self.camera.GAIN_STEP)
		self.entry_analogGain_Preset.setValue(0)
		# self.liveController.set_analog_gain_preset(0)

		self.btn_Preset = QPushButton("Preset")
		self.btn_Preset.setDefault(False)

		# Trigger Mode
		self.triggerMode = None
		self.dropdown_triggerMode = QComboBox()
		self.dropdown_triggerMode.addItems([TriggerMode.SOFTWARE,TriggerMode.HARDWARE,TriggerMode.CONTINUOUS])
		self.dropdown_triggerMode.setCurrentText(TriggerMode.SOFTWARE)

		# Trigger FPS
		self.entry_triggerFPS = QDoubleSpinBox()
		self.entry_triggerFPS.setMinimum(FPS['trigger_software']['min']) 
		self.entry_triggerFPS.setMaximum(FPS['trigger_software']['max']) 
		self.entry_triggerFPS.setSingleStep(1)
		self.entry_triggerFPS.setValue(self.fps_trigger)

		# Trigger FPS actual
		self.actual_streamFPS = QLCDNumber()
		self.actual_streamFPS.setNumDigits(4)
		self.actual_streamFPS.display(0.0)

		# connection
		self.btn_Preset.clicked.connect(self.load_preset)
		self.entry_exposureTime.valueChanged.connect(self.camera.set_exposure_time)
		self.entry_analogGain.valueChanged.connect(self.camera.set_analog_gain)
		self.entry_exposureTime_Preset.valueChanged.connect(self.liveController.set_exposure_time_bfdf_preset)
		self.entry_analogGain_Preset.valueChanged.connect(self.liveController.set_analog_gain_bfdf_preset)
		self.entry_triggerFPS.valueChanged.connect(self.liveController.set_trigger_fps)
		self.dropdown_triggerMode.currentIndexChanged.connect(self.update_trigger_mode)

		# Sub-blocks layout
		grid_ctrl = QGridLayout()
		grid_ctrl.addWidget(QLabel('Exposure Time (ms)'), 0,0)
		grid_ctrl.addWidget(self.entry_exposureTime, 0,1)
		grid_ctrl.addWidget(QLabel('Analog Gain'), 0,2)
		grid_ctrl.addWidget(self.entry_analogGain, 0,3)

		grid_ctrl_preset = QGridLayout()
		grid_ctrl_preset.addWidget(self.entry_exposureTime_Preset, 0,1)
		grid_ctrl_preset.addWidget(self.entry_analogGain_Preset, 0,2)
		grid_ctrl_preset.addWidget(self.btn_Preset, 0,0)
	  
		trigger_fps_group = QGroupBox('Trigger FPS')
		trigger_fps_layout = QGridLayout()
		
		trigger_fps_layout.addWidget(QLabel('Set'),0,0)
		trigger_fps_layout.addWidget(self.entry_triggerFPS, 0,1)
		trigger_fps_layout.addWidget(QLabel('Actual'),0,2)
		trigger_fps_layout.addWidget(self.actual_streamFPS, 0,3)
		trigger_fps_group.setLayout(trigger_fps_layout)

		triggerMode_layout = QHBoxLayout()
		triggerMode_layout.addWidget(QLabel('Trigger mode'))
		triggerMode_layout.addWidget(self.dropdown_triggerMode)

		# Overall layout
		self.grid = QVBoxLayout()
		self.grid.addLayout(grid_ctrl)
		# self.grid.addLayout(grid_ctrl_preset)
		self.grid.addLayout(triggerMode_layout)
		self.grid.addWidget(trigger_fps_group)
		self.grid.addStretch()
		self.setLayout(self.grid)

	def load_preset(self):
		self.entry_exposureTime.setValue(self.entry_exposureTime_Preset.value())
		self.entry_exposureTime.repaint() # update doesn't work
		self.entry_analogGain.setValue(self.entry_analogGain_Preset.value())
		self.entry_analogGain.repaint()

	def update_trigger_mode(self):
		self.liveController.set_trigger_mode(self.dropdown_triggerMode.currentText())

	def set_trigger_mode(self,trigger_mode):
		self.dropdown_triggerMode.setCurrentText(trigger_mode)

	# Slot connected to signal from streamHandler.
	def update_stream_fps(self, value):
		self.actual_streamFPS.display(value)

class LiveControlWidget(QFrame):
	'''
	Widget controls salient microscopy parameters such as:
		- Live Button
		- Checkboxes to choose active image-streams.
		- Objective
		- Display resolution slider
	'''
	signal_update_pixel_size = Signal(float) # Pixel size based on calibration image
	signal_update_image_resizing_factor = Signal(float)
	show_window = Signal(bool)

	def __init__(self, streamHandler, liveController, internalState, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.liveController = liveController
		self.streamHandler = streamHandler
		self.internal_state = internalState
		self.imaging_channels = CAMERAS.keys()
		self.objective = DEFAULT_OBJECTIVE
		self.fps_display = FPS['display']['default']
		self.streamHandler.set_display_fps(self.fps_display)
		self.add_components()
		self._update_pixel_size()
		# self.setTitle('Live Controller')
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		# Live button (0,0)
		self.btn_live = QPushButton("Live")
		self.btn_live.setCheckable(True)
		self.btn_live.setChecked(False)
		self.btn_live.setDefault(False)

		# Microscope objective (0, 1)
		self.dropdown_objectiveSelection = QComboBox()
		self.dropdown_objectiveSelection.addItems(list(OBJECTIVES.keys()))
		self.dropdown_objectiveSelection.setCurrentText(DEFAULT_OBJECTIVE)

		# Checkboxes for enabling image streams (0,2)
		self.checkbox = {}
		for channel in self.imaging_channels:
			self.checkbox[channel] = QCheckBox(channel)
			self.checkbox[channel].setChecked(True)

		# Display FPS (1,0)
		# Entry display fps 
		self.entry_displayFPS = QDoubleSpinBox()
		self.entry_displayFPS.setMinimum(FPS['display']['min']) 
		self.entry_displayFPS.setMaximum(FPS['display']['max']) 
		self.entry_displayFPS.setSingleStep(1)
		self.entry_displayFPS.setValue(FPS['display']['default'])

		 # Display fps actual
		self.actual_displayFPS = QLCDNumber()
		self.actual_displayFPS.setNumDigits(4)
		self.actual_displayFPS.display(0.0)

		# Trigger FPS actual
		self.actual_streamFPS = QLCDNumber()
		self.actual_streamFPS.setNumDigits(4)
		self.actual_streamFPS.display(0.0)

		# Display resolution slider (2,0)
		self.slider_resolutionScaling = QSlider(Qt.Horizontal)
		self.slider_resolutionScaling.setTickPosition(QSlider.TicksBelow)
		self.slider_resolutionScaling.setMinimum(10)
		self.slider_resolutionScaling.setMaximum(100)
		self.slider_resolutionScaling.setValue(50)
		self.slider_resolutionScaling.setSingleStep(10)

		self.display_workingResolution = QLCDNumber()
		self.display_workingResolution.setNumDigits(6)
		self.display_workingResolution.display(0.0)

		# connections
		self.slider_resolutionScaling.valueChanged.connect(self.streamHandler.set_working_resolution_scaling)
		self.slider_resolutionScaling.valueChanged.connect(self._update_image_resizing_factor)
		self.dropdown_objectiveSelection.currentIndexChanged.connect(self._update_pixel_size)
		self.btn_live.clicked.connect(self.toggle_live)

		for channel in self.imaging_channels:
			self.checkbox[channel].clicked.connect(self.update_active_channels)

		self.entry_displayFPS.valueChanged.connect(self.streamHandler.set_display_fps)

		# sub blocks Layout

		# checkbox layout
		checkbox_layout = QGridLayout()
		for column, channel in enumerate(self.imaging_channels):
			checkbox_layout.addWidget(self.checkbox[channel],0,column)

		objective_layout = QHBoxLayout()
		objective_layout.addWidget(QLabel('Objective'))
		objective_layout.addWidget(self.dropdown_objectiveSelection)
		
		working_resolution_group = QGroupBox('Display resolution')
		working_resolution_layout = QGridLayout()
		working_resolution_layout.addWidget(self.slider_resolutionScaling, 0,0)
		working_resolution_layout.addWidget(self.display_workingResolution, 0,1)
		working_resolution_group.setLayout(working_resolution_layout)

		stream_fps_group = QGroupBox('Tracking FPS')
		stream_fps_layout = QHBoxLayout()
		stream_fps_layout.addWidget(QLabel('Measured'))
		stream_fps_layout.addWidget(self.actual_streamFPS)
		stream_fps_group.setLayout(stream_fps_layout)

		display_fps_group = QGroupBox('Display FPS')
		display_fps_layout = QGridLayout()
		display_fps_layout.addWidget(QLabel('Set'),0,0)
		display_fps_layout.addWidget(self.entry_displayFPS, 0,1)
		display_fps_layout.addWidget(QLabel('Actual'),0,2)
		display_fps_layout.addWidget(self.actual_displayFPS, 0,3)
		display_fps_group.setLayout(display_fps_layout)

		# Overall Layout
		top_box_layout = QHBoxLayout()
		top_box_layout.addWidget(self.btn_live)
		top_box_layout.addLayout(objective_layout)
		top_box_layout.addLayout(checkbox_layout)

		middle_box_layout = QHBoxLayout()
		middle_box_layout.addWidget(stream_fps_group)
		middle_box_layout.addWidget(display_fps_group)
		
		self.grid = QVBoxLayout()
		self.grid.addLayout(top_box_layout)
		self.grid.addLayout(middle_box_layout)
		self.grid.addWidget(working_resolution_group)
		self.grid.addStretch()
		self.setLayout(self.grid)
	
	# Slot connected to signal from trackingController.
	def update_working_resolution(self, value):
		self.display_workingResolution.display(value)

	def _update_pixel_size(self):
		self.objective = self.dropdown_objectiveSelection.currentText()
		self.internal_state.data['Objective'] = self.objective
		# only do the calculation for the tracking camera
		pixel_size_um = CAMERA_PIXEL_SIZE_UM[CAMERAS[TRACKING]['sensor']] / ( TUBE_LENS_MM[TRACKING] / (OBJECTIVES[self.objective]['tube_lens_f_mm']/OBJECTIVES[self.objective]['magnification']) )
		self.signal_update_pixel_size.emit(pixel_size_um)
		print('pixel size is ' + str(pixel_size_um) + ' um')

	def _update_image_resizing_factor(self):
		self.signal_update_image_resizing_factor.emit(self.slider_resolutionScaling.value()/100)

	def toggle_live(self,pressed):
		if pressed:
			for channel in self.imaging_channels:
				self.checkbox[channel].setEnabled(False)
				if(self.checkbox[channel].isChecked()):
					if(type(self.liveController) is dict):
						self.liveController[channel].start_live()
					else:
						self.liveController.start_live()					
		else:
			for channel in self.imaging_channels:
				if(type(self.liveController) is dict):
					self.liveController[channel].stop_live()
				else:
					self.liveController.stop_live()
				self.checkbox[channel].setEnabled(True)

	def update_active_channels(self):
		# @@@ TO DO: Convert these to slots and remove dependency on low level objects
		print('Updating active channels')
		for channel in self.imaging_channels:
			if(self.checkbox[channel].isChecked()):
				# Make window active/Show
				self.show_window.emit(True)
			elif (self.checkbox[channel].isChecked()==False):
				# Hide the window.
				self.show_window.emit(False)

	# Slot connected to signal from streamHandler.
	def update_display_fps(self, value):
		self.actual_displayFPS.display(value)

	# Slot connected to signal from streamHandler.
	def update_stream_fps(self, value):
		self.actual_streamFPS.display(value)

# @@@ This widget has been merged with live control and camera settings widget
# class StreamControlWidget(QFrame):
# 	'''
# 	Widget controls image/video-stream parameters:
# 		- Trigger mode (Hardware, Software, Continuous acquisition)
# 		- Trigger FPS (Set and Actual). Set value only matters during Software trigger. 
# 		- Display fps (Set and Actual).
# 	'''

# 	def __init__(self, streamHandler, liveController, camera, main=None, *args, **kwargs):
# 		super().__init__(*args, **kwargs)
# 		self.liveController = liveController
# 		self.streamHandler = streamHandler
# 		self.camera = camera

# 		self.liveController.set_trigger_fps(self.fps_trigger)

		
# 		self.add_components()

# 	def add_components(self):

# 		pass  

class RecordingWidget(QGroupBox):

	start_tracking_signal = Signal()

	def __init__(self, streamHandler, imageSaver, internal_state, trackingDataSaver = None, imaging_channels = TRACKING, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		
		# In general imageSaver, streamHandler are dicts corresponding to each image_channel
		self.imageSaver = imageSaver # for saving path control
		self.streamHandler = streamHandler
		self.internal_state = internal_state 
		self.trackingDataSaver = trackingDataSaver
		self.imaging_channels = imaging_channels

		self.tracking_flag = False
		self.recordingOnly_flag = False

		self.save_dir_base = DEFAULT_SAVE_FOLDER
		self.base_path_is_set = False
		self.add_components()

		self.setTitle('Acquisition')
		# self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		self.btn_setSavingDir = QPushButton('Browse')
		self.btn_setSavingDir.setDefault(False)
		self.btn_setSavingDir.setIcon(QIcon('icon/folder.png'))
		
		self.lineEdit_savingDir = QLineEdit()
		self.lineEdit_savingDir.setReadOnly(True)
		self.lineEdit_savingDir.setText('Choose a base saving directory')

		self.lineEdit_experimentID = QLineEdit()

		self.checkbox = {}
		self.entry_saveFPS = {}
		self.actual_saveFPS = {}
		self.entry_timeLimit = {}

		# Check-boxes to select the image channels to save
		for channel in self.imaging_channels:

			self.checkbox[channel] = QCheckBox(channel)
			self.checkbox[channel].setChecked(True)

			# SpinBox for specifying save FPS of each stream
			self.entry_saveFPS[channel] = QDoubleSpinBox()
			self.entry_saveFPS[channel].setMinimum(0.01) 
			self.entry_saveFPS[channel].setMaximum(200) 
			self.entry_saveFPS[channel].setSingleStep(1)
			self.entry_saveFPS[channel].setValue(100)
			self.streamHandler[channel].set_save_fps(100)

			# LCD for displaying the actual save FPS
			self.actual_saveFPS[channel] = QLCDNumber()
			self.actual_saveFPS[channel].setNumDigits(4)
			self.actual_saveFPS[channel].display(0.0)

			# SpinBox for specifying recording time limit for each stream
			self.entry_timeLimit[channel] = QSpinBox()
			self.entry_timeLimit[channel].setMinimum(-1) 
			self.entry_timeLimit[channel].setMaximum(60*60*24*30) 
			self.entry_timeLimit[channel].setSingleStep(1)
			self.entry_timeLimit[channel].setValue(-1)

		self.radioButton_tracking = QRadioButton("Track+Record")
		self.radioButton_tracking.setChecked(True)
		self.radioButton_recording = QRadioButton("Record")

		self.btn_record = QPushButton("Start Acquisition")
		self.btn_record.setCheckable(True)
		self.btn_record.setChecked(False)
		self.btn_record.setDefault(False)
		self.btn_record.setIcon(QIcon('icon/record.png'))

		grid_line1 = QGridLayout()
		grid_line1.addWidget(QLabel('Saving Path'))
		grid_line1.addWidget(self.lineEdit_savingDir, 0,1)
		grid_line1.addWidget(self.btn_setSavingDir, 0,2)

		grid_line2 = QGridLayout()
		grid_line2.addWidget(QLabel('Experiment ID'), 0,0)
		grid_line2.addWidget(self.lineEdit_experimentID,0,1)

		tracking_recording_layout = QHBoxLayout()
		tracking_recording_layout.addWidget(self.radioButton_tracking)
		tracking_recording_layout.addWidget(self.radioButton_recording)
		tracking_recording_layout.addWidget(self.btn_record)

		imaging_channel_box = QGroupBox('Imaging channels')

		box_layout = QGridLayout()
		box_layout.addWidget(QLabel('Channel'), 0,0,1,1)
		box_layout.addWidget(QLabel('Save FPS'), 0,1,1,1)
		box_layout.addWidget(QLabel('Actual FPS'), 0,2,1,1)
		box_layout.addWidget(QLabel('Time limit'), 0,3,1,1)

		for row, channel in enumerate(self.imaging_channels):
			if channel == 'volumetric imaging':
				self.checkbox[channel].setChecked(False)
				continue # skip volumetric imaging acquisition control in this widget as it's controlled separately
			box_layout.addWidget(self.checkbox[channel], row+1, 0, 1, 1)
			box_layout.addWidget(self.entry_saveFPS[channel], row+1, 1, 1, 1)
			box_layout.addWidget(self.actual_saveFPS[channel], row+1, 2, 1, 1)
			box_layout.addWidget(self.entry_timeLimit[channel], row+1, 3, 1, 1)


		self.grid = QGridLayout()
		self.grid.addLayout(box_layout,0,0,1,1)
		self.grid.addLayout(grid_line1,1,0,1,1)
		self.grid.addLayout(grid_line2,2,0,1,1)
		# self.grid.addWidget(self.btn_record,3,0,1,1)
		self.grid.addLayout(tracking_recording_layout,3,0,1,1)
		self.setLayout(self.grid)

		# add and display a timer - to be implemented
		# self.timer = QTimer()

		# connections
		self.btn_setSavingDir.clicked.connect(self.set_saving_dir)
		self.btn_record.clicked.connect(self.toggle_recording)

		self.radioButton_recording.clicked.connect(self.set_tracking_recording_flag)
		self.radioButton_tracking.clicked.connect(self.set_tracking_recording_flag)
		self.lineEdit_experimentID.textEdited.connect(self.trackingDataSaver.update_experiment_ID)

		for channel in self.imaging_channels:
			self.entry_saveFPS[channel].valueChanged.connect(self.streamHandler[channel].set_save_fps)
			self.entry_timeLimit[channel].valueChanged.connect(self.imageSaver[channel].set_recording_time_limit)
			self.imageSaver[channel].stop_recording.connect(self.stop_recording)

	def set_saving_dir(self, use_default_dir = False):
		if(use_default_dir is False):
			dialog = QFileDialog()
			self.save_dir_base = dialog.getExistingDirectory(None, "Select Folder")

		# Set base path for image saver
		for channel in self.imaging_channels:
			if(self.checkbox[channel].isChecked()):
				self.imageSaver[channel].set_base_path(self.save_dir_base)
		
		# set the base path for the data saver
		if(self.trackingDataSaver is not None):
			self.trackingDataSaver.set_base_path(self.save_dir_base)
		
		self.lineEdit_savingDir.setText(self.save_dir_base)
		self.base_path_is_set = True

	def toggle_recording(self,pressed):
		if self.base_path_is_set == False and self.save_dir_base is None:
			self.btn_record.setChecked(False)
			msg = QMessageBox()
			msg.setText("Please choose base saving directory first")
			msg.exec_()
			return
		elif self.save_dir_base is not None:
			self.set_saving_dir(use_default_dir = True)

		if pressed:
			''' Start Acquisition
			'''
			self.internal_state.data['Acquisition'] = True
			self.lineEdit_experimentID.setEnabled(False)
			self.btn_setSavingDir.setEnabled(False)
			for channel in self.imaging_channels:
				self.checkbox[channel].setEnabled(False)

			self.trackingDataSaver.start_new_experiment()

			if(self.trackingDataSaver is not None and self.recordingOnly_flag==False):
				self.start_tracking_signal.emit()
			else:
				pass

			for channel in self.imaging_channels:
				if(self.checkbox[channel].isChecked()):
					self.imageSaver[channel].start_saving_images()
					self.streamHandler[channel].start_recording()
		else:
			''' Stop Acquisition
			'''
			self.internal_state.data['Acquisition'] = False
			for channel in self.imaging_channels:
				self.streamHandler[channel].stop_recording()
				self.checkbox[channel].setEnabled(True)
			
			self.lineEdit_experimentID.setEnabled(True)
			self.btn_setSavingDir.setEnabled(True)

	# stop_recording can be called by imageSaver
	def stop_recording(self):
		self.lineEdit_experimentID.setEnabled(True)
		self.btn_record.setChecked(False)
		for channel in self.imaging_channels:
				self.streamHandler[channel].stop_recording()
				self.checkbox[channel].setEnabled(True)
		self.btn_setSavingDir.setEnabled(True)

	def set_tracking_recording_flag(self):
		if(self.radioButton_tracking.isChecked()):
			self.recordingOnly_flag = False
			print('Set mode to Tracking+Rec')
		elif(self.radioButton_recording.isChecked()):
			self.recordingOnly_flag = True
			print('Set mode to Recording only')

	def update_save_fps(self, channel, real_fps):
		self.actual_saveFPS[channel].display(real_fps)

'''
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                              Plot widget
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''
class dockAreaPlot(dock.DockArea):

	def __init__(self, internal_state, parent=None):
		super().__init__(parent)
		self.internal_state = internal_state
		DockLabel.updateStyle = dstyle.updateStylePatched
		self.plots = {key:PlotWidget(key, self.internal_state) for key in PLOT_VARIABLES.keys()}
		self.docks = {key:dock.Dock(key) for key in PLOT_VARIABLES.keys()}
		for key in PLOT_VARIABLES.keys():
			self.docks[key].addWidget(self.plots[key])
		
		# Layout of the plots
		self.addDock(self.docks['X'])
		self.addDock(self.docks['Z'],'above',self.docks['X'])

		prev_key = 'Z'
		for key in PLOT_VARIABLES:
			if key not in DEFAULT_PLOTS:
				self.addDock(self.docks[key],'above',self.docks[prev_key])
				prev_key = key

		self.initialise_plot_area()

	def initialise_plot_area(self):
		for key in self.plots.keys():
			self.plots[key].initialise_plot()

	def update_plots(self):
		for key in self.plots.keys():
			self.plots[key].update_plot()


class PlotWidget(pg.GraphicsLayoutWidget):

	def __init__(self,title, internal_state, parent=None):
		super().__init__(parent)
		self.title=title
		self.key = PLOT_VARIABLES[self.title]
		self.internal_state = internal_state
		#plot Zobj
		self.Abscissa=deque(maxlen=20)
		self.Ordinate=deque(maxlen=20)
		self.Abs=[]
		self.Ord=[]
		self.plot1=self.addPlot(title=title)
		self.curve=self.plot1.plot(self.Abs,self.Ord, pen=pg.mkPen(PLOT_COLORS[self.title], width=3))
		self.curve.setClipToView(True)
		self.plot1.enableAutoRange('xy', True)
		self.plot1.showGrid(x=True, y=True)
		
	def update_plot(self):
		data = np.zeros(2)
		# For now the x-axis is always time
		data[0] = self.internal_state.data['Time']
		data[1] = self.internal_state.data[self.key]
		self.Abscissa.append(data[0])
		self.Ordinate.append(data[1])
		self.Abs=list(self.Abscissa)
		self.Ord=list(self.Ordinate)
		self.curve.setData(self.Abs,self.Ord)

	def initialise_plot(self):
		self.Abscissa=deque(maxlen=20)
		self.Ordinate=deque(maxlen=20)
		self.Abs=[]
		self.Ord=[]
		self.label = PLOT_UNITS[self.title]
		self.curve.setData(self.Abs,self.Ord)

class LEDMatrixControlWidget(QFrame):

	def __init__(self,microcontroller):
		super().__init__()
		self.microcontroller = microcontroller
		self.illumination_source = LED_MATRIX_PATTERN['LED matrix left half']
		self.add_components()		
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		self.dropdown_LED_matrix_pattern = QComboBox()
		for pattern in LED_MATRIX_PATTERN.keys():
			self.dropdown_LED_matrix_pattern.addItems([pattern])
		self.dropdown_LED_matrix_pattern.setCurrentText('LED matrix left half')

		self.slider_R = QSlider(Qt.Horizontal)
		self.slider_R.setTickPosition(QSlider.TicksBelow)
		self.slider_R.setMinimum(0)
		self.slider_R.setMaximum(100)
		self.slider_R.setSingleStep(1)
		self.slider_R.setValue(LED_MATRIX_R_FACTOR*100)
		self.entry_R = QDoubleSpinBox()
		self.entry_R.setMinimum(0) 
		self.entry_R.setMaximum(1) 
		self.entry_R.setSingleStep(0.01)
		self.entry_R.setValue(LED_MATRIX_R_FACTOR)

		self.slider_G = QSlider(Qt.Horizontal)
		self.slider_G.setTickPosition(QSlider.TicksBelow)
		self.slider_G.setMinimum(0)
		self.slider_G.setMaximum(100)
		self.slider_G.setSingleStep(1)
		self.slider_G.setValue(LED_MATRIX_G_FACTOR*100)
		self.entry_G = QDoubleSpinBox()
		self.entry_G.setMinimum(0) 
		self.entry_G.setMaximum(1) 
		self.entry_G.setSingleStep(0.01)
		self.entry_G.setValue(LED_MATRIX_G_FACTOR)

		self.slider_B = QSlider(Qt.Horizontal)
		self.slider_B.setTickPosition(QSlider.TicksBelow)
		self.slider_B.setMinimum(0)
		self.slider_B.setMaximum(100)
		self.slider_B.setSingleStep(1)
		self.slider_B.setValue(LED_MATRIX_B_FACTOR*100)
		self.entry_B = QDoubleSpinBox()
		self.entry_B.setMinimum(0) 
		self.entry_B.setMaximum(100) 
		self.entry_B.setSingleStep(0.01)
		self.entry_B.setValue(LED_MATRIX_B_FACTOR)

		self.slider_intensity = QSlider(Qt.Horizontal)
		self.slider_intensity.setTickPosition(QSlider.TicksBelow)
		self.slider_intensity.setMinimum(0)
		self.slider_intensity.setMaximum(100)
		self.slider_intensity.setSingleStep(1)
		self.slider_intensity.setValue(20)
		self.entry_intensity = QDoubleSpinBox()
		self.entry_intensity.setMinimum(0) 
		self.entry_intensity.setMaximum(100) 
		self.entry_intensity.setSingleStep(1)
		self.entry_intensity.setValue(10)

		self.btn_toggle = QPushButton('LED Matrix On/Off')
		self.btn_toggle.setCheckable(True)
		self.btn_toggle.setDefault(False)

		grid = QGridLayout()
		grid.addWidget(QLabel('LED matrix pattern'),0,0)
		grid.addWidget(self.dropdown_LED_matrix_pattern,0,1,1,2)
		grid.addWidget(QLabel('LED matrix R'),1,0)
		grid.addWidget(self.slider_R,1,1)
		grid.addWidget(self.entry_R,1,2)
		grid.addWidget(QLabel('LED matrix G'),2,0)
		grid.addWidget(self.slider_G,2,1)
		grid.addWidget(self.entry_G,2,2)
		grid.addWidget(QLabel('LED matrix B'),3,0)
		grid.addWidget(self.slider_B,3,1)
		grid.addWidget(self.entry_B,3,2)
		grid.addWidget(QLabel('Intensity'),4,0)
		grid.addWidget(self.slider_intensity,4,1)
		grid.addWidget(self.entry_intensity,4,2)
		grid.addWidget(self.btn_toggle,5,0,1,3)
		grid.setRowStretch(grid.rowCount(), 1)

		self.setLayout(grid)

		# connections
		self.slider_R.valueChanged.connect(lambda x: self.entry_R.setValue(x/100.0))
		self.entry_R.valueChanged.connect(lambda x: self.slider_R.setValue(x*100))
		self.entry_R.valueChanged.connect(self.update_illumination)
		self.slider_G.valueChanged.connect(lambda x: self.entry_G.setValue(x/100.0))
		self.entry_G.valueChanged.connect(lambda x: self.slider_G.setValue(x*100))
		self.entry_G.valueChanged.connect(self.update_illumination)
		self.slider_B.valueChanged.connect(lambda x: self.entry_B.setValue(x/100.0))
		self.entry_B.valueChanged.connect(lambda x: self.slider_B.setValue(x*100))
		self.entry_B.valueChanged.connect(self.update_illumination)
		self.slider_intensity.valueChanged.connect(self.entry_intensity.setValue)
		self.entry_intensity.valueChanged.connect(self.slider_intensity.setValue)
		self.entry_intensity.valueChanged.connect(self.update_illumination)
		self.btn_toggle.clicked.connect(self.toggle_illumination)

	def toggle_illumination(self,on):
		if on == True:
			self.microcontroller.turn_on_illumination()
		else:
			self.microcontroller.turn_off_illumination()

	def set_illumination_pattern(self):
		self.illumination_source = LED_MATRIX_PATTERN[self.dropdown_LED_matrix_pattern.currentText()]

	def update_illumination(self):
		if self.illumination_source < 10: # LED matrix
			self.microcontroller.set_illumination_led_matrix(self.illumination_source,
				r=(self.entry_intensity.value()/100.0)*(self.entry_R.value()),
				g=(self.entry_intensity.value()/100.0)*(self.entry_G.value()),
				b=(self.entry_intensity.value()/100.0)*(self.entry_B.value()))
		else:
			self.microcontroller.set_illumination(self.illumination_source,self.intensity)

class LEDControlWidget(QFrame):

	def __init__(self,microcontroller):
		super().__init__()
		self.microcontroller = microcontroller
		self.illumination_source = 11
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		self.slider_intensity = QSlider(Qt.Horizontal)
		self.slider_intensity.setTickPosition(QSlider.TicksBelow)
		self.slider_intensity.setMinimum(0)
		self.slider_intensity.setMaximum(100)
		self.slider_intensity.setSingleStep(1)
		self.slider_intensity.setValue(20)
		self.entry_intensity = QDoubleSpinBox()
		self.entry_intensity.setMinimum(0)
		self.entry_intensity.setMaximum(100)
		self.entry_intensity.setSingleStep(1)
		self.entry_intensity.setValue(10)

		self.btn_toggle = QPushButton('LED Matrix On/Off')
		self.btn_toggle.setCheckable(True)
		self.btn_toggle.setDefault(False)

		grid = QGridLayout()
		grid.addWidget(QLabel('Intensity'),4,0)
		grid.addWidget(self.slider_intensity,4,1)
		grid.addWidget(self.entry_intensity,4,2)
		grid.addWidget(self.btn_toggle,5,0,1,3)
		grid.setRowStretch(grid.rowCount(), 1)
		self.setLayout(grid)

		# connections
		self.slider_intensity.valueChanged.connect(self.entry_intensity.setValue)
		self.entry_intensity.valueChanged.connect(self.slider_intensity.setValue)
		self.entry_intensity.valueChanged.connect(self.update_illumination)
		self.btn_toggle.clicked.connect(self.toggle_illumination)

	def toggle_illumination(self,on):
		if on == True:
			self.microcontroller.turn_on_illumination()
		else:
			self.microcontroller.turn_off_illumination()

	def update_illumination(self):
		self.microcontroller.set_illumination(self.illumination_source,self.entry_intensity.value())

