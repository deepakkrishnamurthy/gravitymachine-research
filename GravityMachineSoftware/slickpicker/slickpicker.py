from PyQt5 import QtCore, QtGui, QtWidgets
import re

COLOR_RE = re.compile(r'(#[0-9a-fA-F]{3})([0-9a-fA-F]{3})?')
COLOR_NAMES = QtGui.QColor.colorNames()

class QColorValidator(QtGui.QValidator):
	def __init__(self, parent=None):
		super().__init__(parent)

	def validate(self, text, pos):
		portion = text
		matchy = COLOR_RE.match(portion)
		text_len=len(text)
		#print(matchy, text)
		if matchy is not None:
			if matchy.group(1) == portion and text_len == 4:
				return (QtGui.QValidator.Acceptable, text, pos)
			elif matchy.group(1) and matchy.group(2) is None:
				return (QtGui.QValidator.Intermediate, text, pos)
			elif matchy.group(1) and matchy.group(2) and text_len == 7:
				return (QtGui.QValidator.Acceptable, text, pos)
			else:
				return (QtGui.QValidator.Invalid, text, pos)
		else:
			if text_len == 0:
				return (QtGui.QValidator.Intermediate, text, pos)
			elif text_len < 4 and text[0] == '#':
				return (QtGui.QValidator.Intermediate, text, pos)
			partial = re.compile(re.escape(text[:pos]))
			for named_col in COLOR_NAMES:
				match = partial.match(named_col)
				if match is not None:
					if match.string == text:
						return (QtGui.QValidator.Acceptable, text, pos)
					else:
						return (QtGui.QValidator.Intermediate, text, pos)
		return (QtGui.QValidator.Invalid, text, pos)

class QSpinSlider(QtWidgets.QWidget):
	def __init__(self, parent=None, slider=None, spinner=None):
		super().__init__(parent)

		if slider is None:
			self.slider = QtWidgets.QSlider()
			self.slider.setOrientation(QtCore.Qt.Horizontal)
		else:
			self.slider = slider

		if spinner is None:
			self.spinner = QtWidgets.QSpinBox()
		else:
			self.spinner = spinner

		if self.slider.orientation() == QtCore.Qt.Horizontal:
			layout = QtWidgets.QHBoxLayout(self)
			layout.addWidget(self.slider, alignment = QtCore.Qt.AlignVCenter)
		else:
			layout = QtWidgets.QVBoxLayout(self)
			layout.addWidget(self.slider, alignment = QtCore.Qt.AlignHCenter)

		layout.addWidget(self.spinner, alignment = QtCore.Qt.AlignHCenter)
		
		self.slider.valueChanged.connect(self.spinner.setValue)
		self.spinner.valueChanged.connect(self.slider.setValue)
		self.slider.rangeChanged.connect(self.spinner.setRange)

class QColorLineEdit(QtWidgets.QLineEdit):
	colorChanged = QtCore.pyqtSignal('QColor')

	def __init__(self, color=None, parent=None):
		super().__init__(parent)
		#self.clicked.connect(lambda x: self.adjustColor())
		self.textChanged.connect(self.adjustColor)
		self._cmpl = QtWidgets.QCompleter()
		self._model = QtCore.QStringListModel(COLOR_NAMES)
		self._cmpl.setModel(self._model)
		self.setCompleter(self._cmpl)

		if isinstance(color,QtGui.QColor):
			self.setColor(color)
		else:
			self.setColor(QtGui.QColor())
		self.setValidator(QColorValidator())
	
	def adjustColor(self, text):
		if self.hasAcceptableInput():
			#print("Updating color display with", text)
			self.setColor(text)
		#self.color = QtGui.QColorDialog.getColor(initial=self.color,parent=self)

	def color(self):
		return self._color
	
	def setColor(self,color):
		if isinstance(color,QtGui.QColor):
			#print(color.name())
			self._color = color
			self.setText(color.name())
			self.colorChanged.emit(color)
		elif isinstance(color, str):
			self.setText(color)
			self._color = QtGui.QColor(color)
			self.colorChanged.emit(self._color)
		else:
			raise ValueError("That isn't a QColor!")
	
	color = QtCore.pyqtProperty('QColor', fget=color, fset=setColor)

class QColorSpinEdit(QtWidgets.QWidget):
	colorChanged = QtCore.pyqtSignal('QColor')
	def __init__(self, color=None, parent=None):
		super().__init__(parent)
		layout = QtWidgets.QFormLayout(self)

		self.HEdit = QSpinSlider()
		self.SEdit = QSpinSlider()
		self.VEdit = QSpinSlider()
		
		self.REdit = QSpinSlider()
		self.GEdit = QSpinSlider()
		self.BEdit = QSpinSlider()
		
		self.HEdit.slider.setRange(0,359)
		self.SEdit.slider.setRange(0,255)
		self.VEdit.slider.setRange(0,255)
		
		self.REdit.slider.setRange(0,255)
		self.GEdit.slider.setRange(0,255)
		self.BEdit.slider.setRange(0,255)

		self.HEdit.slider.valueChanged.connect(self._makeHSVColor)
		self.SEdit.slider.valueChanged.connect(self._makeHSVColor)
		self.VEdit.slider.valueChanged.connect(self._makeHSVColor)

		self.REdit.slider.valueChanged.connect(self._makeRGBColor)
		self.GEdit.slider.valueChanged.connect(self._makeRGBColor)
		self.BEdit.slider.valueChanged.connect(self._makeRGBColor)

		self.colorChanged.connect(self._syncSliders)

		layout.addRow("Hue", self.HEdit)
		layout.addRow("Saturation", self.SEdit)
		layout.addRow("Value", self.VEdit)

		layout.addRow("Red", self.REdit)
		layout.addRow("Green", self.GEdit)
		layout.addRow("Blue", self.BEdit)
		self._bup = False

	def _makeRGBColor(self, val):
		if self._bup is False:
			self._bup = True
			color = QtGui.QColor.fromRgb(self.REdit.slider.value(),
						self.GEdit.slider.value(),
						self.BEdit.slider.value())
			self.HEdit.slider.setValue(color.hue())
			self.SEdit.slider.setValue(color.saturation())
			self.VEdit.slider.setValue(color.value())
			self.setColor(color)
			self._bup = False

	def _makeHSVColor(self, val):
		if self._bup is False:
			self._bup = True
			color = QtGui.QColor.fromHsv(self.HEdit.slider.value(),
							self.SEdit.slider.value(),
							self.VEdit.slider.value())
			self.REdit.slider.setValue(color.red())
			self.GEdit.slider.setValue(color.green())
			self.BEdit.slider.setValue(color.blue())
			self.setColor(color)
			self._bup = False

	def _syncSliders(self, color):
		self.HEdit.slider.setValue(color.hue())
		self.SEdit.slider.setValue(color.saturation())
		self.VEdit.slider.setValue(color.value())

		self.REdit.slider.setValue(color.red())
		self.GEdit.slider.setValue(color.green())
		self.BEdit.slider.setValue(color.blue())

	def color(self):
		return self._color
	
	def setColor(self, color):
		self._color = color
		self.colorChanged.emit(color)

	color = QtCore.pyqtProperty('QColor', fget=color, fset=setColor)

class QColorEdit(QtWidgets.QWidget):
	colorChanged = QtCore.pyqtSignal('QColor')

	def __init__(self, color=None, parent=None, useQColorDialog=True):
		super().__init__(parent)
		layout = QtWidgets.QGridLayout(self)
		
		self.previewPanel = QtWidgets.QPushButton()
		self.picky = QtWidgets.QPushButton()
		self.picky.setText("Pick from screen")
		self.previewPanel.setCheckable(True)
		self.previewPanel.setText("▶")
		self.lineEdit = QColorLineEdit()
		if not useQColorDialog:
			self.spinColEdit = QColorSpinEdit(self)
		else:
			self.spinColEdit = QtWidgets.QColorDialog(self)
			self.spinColEdit.setOption(QtWidgets.QColorDialog.DontUseNativeDialog)
			self.spinColEdit.setOption(QtWidgets.QColorDialog.NoButtons)
		self.spinColEdit.setWindowFlags(QtCore.Qt.Popup)
		if isinstance(color, QtGui.QColor):
			self._color = color
		elif isinstance(color, str):
			self._color = QtGui.QColor(color)
		else:
			self._color = QtGui.QColor()
		self._picking = False

		self.previewPanel.clicked.connect(self._toggleHsv)
		self.picky.clicked.connect(self._preparePicking)

		self.lineEdit.colorChanged.connect(self._syncWidgets)
		self.lineEdit.colorChanged.connect(self.setColor)
		
		if not useQColorDialog:
			self.spinColEdit.colorChanged.connect(self._syncWidgets)
			self.spinColEdit.colorChanged.connect(self.setColor)
		else:
			self.spinColEdit.currentColorChanged.connect(self._syncWidgets)
			self.spinColEdit.currentColorChanged.connect(self.setColor)
		#self.spinColEdit.colorChanged.connect(self.lineEdit.setColor)

		self.colorChanged.connect(self._updatePreview)
		self.spinColEdit.installEventFilter(self)

		layout.addWidget(self.lineEdit,0,0,1,1)
		layout.addWidget(self.picky,0,1,1,1)
		layout.addWidget(self.previewPanel,0,2,1,1)
		if not useQColorDialog:
			self._updatePreview(self.color)
		else:
			self.spinColEdit.setCurrentColor(self.color)

	def _syncWidgets(self, c):
		#print(self.sender())
		if self.sender() is self.lineEdit:
			if isinstance(self.spinColEdit, QColorSpinEdit):
				self.spinColEdit._syncSliders(c)
				self.spinColEdit._color=c
			else:
				self.spinColEdit.setCurrentColor(c)
		elif self.sender() is self.spinColEdit:
			self.lineEdit.setColor(c)

	def _preparePicking(self, event):
		self._picking = True
		self.grabKeyboard()
		self.grabMouse(QtCore.Qt.CrossCursor)

	def mouseMoveEvent(self, event):
		if self._picking:
			self._pickColor(event.globalPos())
			return
		return QtWidgets.QWidget.mouseMoveEvent(self, event)

	def keyPressEvent(self, event):
		if self._picking:
			if event.key() == QtCore.Qt.Key_Escape:
				self._picking = False
				self.releaseKeyboard()
				self.releaseMouse()
			event.accept()
			return
		return QtWidgets.QWidget.keyPressEvent(self, event)

	def mouseReleaseEvent(self, event):
		if self._picking:
			self._picking = False
			self.releaseKeyboard()
			self.releaseMouse()
			self._pickColor(event.globalPos())
			return
		return QtWidgets.QWidget.mouseReleaseEvent(self, event)

	def _pickColor(self, pos):
		desktop = QtWidgets.QApplication.desktop().winId()
		screen = QtWidgets.QApplication.primaryScreen()
		pixmap = screen.grabWindow(
			desktop,
			pos.x(), pos.y(),
			1, 1
		)
		img = pixmap.toImage()
		col = QtGui.QColor(img.pixel(0,0))
		self.lineEdit.setColor(col)
		if isinstance(self.spinColEdit,QColorSpinEdit):
			self.spinColEdit.setColor(col)
		else:
			self.spinColEdit.setCurrentColor(col)
		self.setColor(col)

	def eventFilter(self, source, event):
		if source is self.spinColEdit and event.type() in (QtCore.QEvent.Close, QtCore.QEvent.Hide, QtCore.QEvent.HideToParent):
			self.previewPanel.setChecked(False)
			self.previewPanel.setText("▶")
		return QtWidgets.QWidget.eventFilter(self, source, event)

	def _toggleHsv(self, checked):
		if checked:
			self.spinColEdit.show()
			really_here = self.mapToGlobal(self.rect().bottomRight())
			bottom_margin = self.layout().contentsMargins().bottom()
			bottom_corner= really_here.x()-self.spinColEdit.width()
			pop_here = really_here.y()-bottom_margin
			#print(bottom_corner, pop_here, self.height(), really_here.y(), bottom_margin)
			self.spinColEdit.move(bottom_corner, pop_here)
			self.previewPanel.setText("▼")
		else:
			#print("e")
			self.spinColEdit.hide()
			self.previewPanel.setText("▶")

	def _updatePreview(self, color):
		px=QtGui.QPixmap(128,128)
		px.fill(color)
		self.previewPanel.setIcon(QtGui.QIcon(px))

	def color(self):
		return self._color
	
	def setColor(self, color):
		self._color = color
		self.colorChanged.emit(color)

	color = QtCore.pyqtProperty('QColor', fget=color, fset=setColor)
