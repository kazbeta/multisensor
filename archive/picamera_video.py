import picamera
import os
import time

filename = str(time.time())

camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
camera.start_recording(filename + ".h264")
camera.wait_recording(30)
camera.stop_recording()

os.system("MP4Box -add %s.h264" % filename + " %s.mp4" % time.time())
os.system("rm %s.h264" % filename)
