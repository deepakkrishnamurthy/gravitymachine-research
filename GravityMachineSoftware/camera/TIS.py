import numpy
import gi
from collections import namedtuple
from time import sleep
import sys
import time #@@@

gi.require_version("Gst", "1.0")
gi.require_version("Tcam", "0.1")

from gi.repository import Tcam, Gst, GLib, GObject


DeviceInfo = namedtuple("DeviceInfo", "status name identifier connection_type")
CameraProperty = namedtuple("CameraProperty", "status value min max default step type flags category group")


def on_new_buffer(appsink):
    print("Hallo")
    return False


class TIS:
    'The Imaging Source Camera'

    def __init__(self,serial, width, height, framerate, color):
        Gst.init(sys.argv)
        self.height = height
        self.width = width
        self.sample = None
        self.samplelocked = False
        self.newsample = False
        self.gotimage = False
        self.img_mat = None
        self.c = 0
        #@@@
        self.newImageCallbackFunction = None

        format = "BGRx"
        if(color == False):
            format="GRAY8"

        if(framerate == 2500000):    
            p = 'tcambin serial="%s" name=source ! video/x-raw,format=%s,width=%d,height=%d,framerate=%d/10593' % (serial,format,width,height,framerate,)
        else:
            p = 'tcambin serial="%s" name=source ! video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1' % (serial,format,width,height,framerate,)

        p += ' ! videoconvert ! appsink name=sink'

        print(p)
        try:
            self.pipeline = Gst.parse_launch(p)
        except GLib.Error as error:
            print("Error creating pipeline: {0}".format(err))
            raise

        self.pipeline.set_state(Gst.State.READY)
        self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
        # Query a pointer to our source, so we can set properties.
        self.source = self.pipeline.get_by_name("source")

        # Query a pointer to the appsink, so we can assign the callback function.
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.set_property("max-buffers",5)
        self.appsink.set_property("drop", True)
        self.appsink.set_property("emit-signals", True)
        self.appsink.connect('new-sample', self.on_new_buffer)

    def on_new_buffer(self, appsink):
        # Function that is called when a new sample from camera is available
        self.newsample = True
        # print('new buffer received: ' + str(time.time())) #@@@
        if self.samplelocked is False:
            self.samplelocked = True
            try:
                self.sample = self.appsink.get_property('last-sample')
                self.gstbuffer_to_opencv()
                # print('new buffer read into RAM: ' + str(time.time())) #@@@
                self.samplelocked = False
                self.newsample = False
                # gotimage reflects if a new image was triggered
                self.gotimage = True
                #@@@
                if self.newImageCallbackFunction is not None:
                    self.newImageCallbackFunction(self.newImageCallback_CallingObject)
            except GLib.Error as error:
                print("Error on_new_buffer pipeline: {0}".format(err)) #error
                self.img_mat = None
        return Gst.FlowReturn.OK

    def start(self):
        try:
            self.pipeline.set_state(Gst.State.PLAYING)
            self.pipeline.get_state(Gst.CLOCK_TIME_NONE)

        except GLib.Error as error:
            print("Error starting pipeline: {0}".format(err))#error?
            raise

    def triggered(self):
        # print('Got image: {}'.format(self.gotimage))
        return self.gotimage

    def read(self):
        self.gotimage = False
        # print('In TIS read function')
        return self.img_mat

    def gstbuffer_to_opencv(self):
        # Sample code from https://gist.github.com/cbenhagen/76b24573fa63e7492fb6#file-gst-appsink-opencv-py-L34
        buf = self.sample.get_buffer()
        caps = self.sample.get_caps()
        bpp = 4;
        if caps.get_structure(0).get_value('format') == "BGRx":
            bpp = 4;

        if caps.get_structure(0).get_value('format') == "GRAY8":
            bpp = 1;

        self.img_mat = numpy.ndarray(
            (caps.get_structure(0).get_value('height'),
             caps.get_structure(0).get_value('width'),
             bpp),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=numpy.uint8)


    def stop(self):
        
        # self.pipeline.set_state(Gst.State.PAUSED)
        # print("pipeline paused")
        self.pipeline.set_state(Gst.State.NULL)
        print("pipeline stopped")
        self.pipeline.set_state(Gst.State.READY)
        print("pipeline ready")
        # self.pipeline.set_state(Gst.State.NULL)
        # print("pipeline stopped")


    def Pause_pipeline(self):
        self.pipeline.set_state(Gst.State.PAUSED)

    def Resume_pipeline(self):
        self.pipeline.set_state(Gst.State.PLAYING)

    #@@@
    def Set_NewImage_Callback(self,function,*data):
        self.newImageCallbackFunction = function
        self.newImageCallback_CallingObject = data

    def List_Properties(self):
        for name in self.source.get_tcam_property_names():
            print( name )

    def Get_Property(self, PropertyName):
        try:
            return CameraProperty(*self.source.get_tcam_property(PropertyName))
        except GLib.Error as error:
            print("Error get Property {0}: {1}",PropertyName, format(err))
            raise

    def Set_Property(self, PropertyName, value):
        try:
            self.source.set_tcam_property(PropertyName,GObject.Value(type(value),value))
        except GLib.Error as error:
            print("Error set Property {0}: {1}",PropertyName, format(err))
            raise
