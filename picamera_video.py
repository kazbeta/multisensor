import picamera
import os
import time

def capture(fps, duration):
    filename = "temporary"

    camera = picamera.PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = fps
    camera.start_recording(filename + ".h264")
    camera.wait_recording(duration)
    camera.stop_recording()

#    os.system("MP4Box -add %s.h264" % filename + " %s.mp4" % filename)
#    os.system("rm %s.h264" % filename)
