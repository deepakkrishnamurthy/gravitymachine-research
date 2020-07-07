import cv2
from camera.WebcamVideoStream import WebcamVideoStream
import camera.TIS as TIS

class VideoStream:
    def __init__(self, src=0, CAMERA=3, Serial = None, resolution=(1440, 1080),framerate=120, color = False):
        # check to see if the picamera module should be used
        self.Camera=CAMERA
        self.color = color
        self.serial = Serial
        
        if self.Camera==1: #pi camera
            # only import the picamera packages unless we are
            # explicity told to do so -- this helps remove the
            # requirement of `picamera[array]` from desktops or
            # laptops that still want to use the `imutils` package
            from .pivideostream import PiVideoStream
            # initialize the picamera stream and allow the camera
            # sensor to warmup
            self.stream = PiVideoStream(resolution=resolution,framerate=framerate)

        # otherwise, we are using OpenCV so initialize the webcam
        # stream
        elif self.Camera==2: #WebCam
            self.stream = WebcamVideoStream(src=src, camResolution=resolution,camFPS=framerate)
        
        elif self.Camera==3: #Tis camera
            # self.stream = TIS.TIS("07810322",resolution[0],resolution[1],framerate,True)
            # DMK33UX252 Camera "08910100"
            self.stream = TIS.TIS(self.serial,resolution[0],resolution[1],framerate,self.color)


        
    def start(self):
        # start the threaded video stream
        return self.stream.start()
    
    def update(self):
        # grab the next frame from the stream
        #no update for the TIS Cam
        self.stream.update()
    
    
    def read(self):
        # return the current frame
        image=self.stream.read()

        if self.Camera==3 and self.color==True:
            image=cv2.cvtColor(image,cv2.COLOR_BGRA2BGR)

   
        return image

    def stop(self):
        # stop the thread and release any resources
        self.stream.stop()

    def set_property(self,property_name,property_value):
        # set camera properties
        if self.Camera==3:
            self.stream.Set_Property(property_name, property_value)

    def triggered(self):
        # check if camera is triggered && image is ready
        if self.Camera==3:
            # print('Triggered: {}'.format(self.stream.triggered()))
            return self.stream.triggered()
        else:
            return False

    def set_newImage_callback(self,function,*data):
        # set camera properties
        if self.Camera==3:
            self.stream.Set_NewImage_Callback(function, data)

