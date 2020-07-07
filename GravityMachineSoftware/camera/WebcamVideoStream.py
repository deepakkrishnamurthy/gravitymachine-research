import cv2
from threading import Thread

class WebcamVideoStream:
    def __init__(self, src=0,camResolution=(1280,720),camFPS=30):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, camResolution[0])
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, camResolution[1])
        self.stream.set(cv2.CAP_PROP_FPS, camFPS)
        #self.stream.set(15, camExposure)
        #self.stream.set(cv2.CAP_PROP_GAIN, camGain)
        (self.grabbed, self.frame) = self.stream.read()
        # initialize the variable used to indicate if the thread should be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self
    
    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
        self.stream.release()
        cv2.destroyAllWindows()
        
        