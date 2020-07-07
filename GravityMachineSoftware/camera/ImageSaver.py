# -*- coding: utf-8 -*-
"""
Created on Sun Jun 17 21:24:38 2018

@author: Francois
"""

from threading import Thread
from queue import Queue
import cv2 as cv2


class ImageSaver():
    
    def __init__(self):
        Thread.__init__(self)
        self.queue = Queue()
        self.stopped = False
        self.start()
           
            
    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.run, args=())
        t.daemon = True
        t.start()
        return self
    
    def run(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # otherwise, read the next frame from the stream
            [path,data_bgr]=self.queue.get()
            cv2.imwrite(path,data_bgr) 
            self.queue.task_done()
            
    def register(self,path,data_bgr):
        self.queue.put([path,data_bgr]) 
        
    def wait(self):
        self.queue.join()
            
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True