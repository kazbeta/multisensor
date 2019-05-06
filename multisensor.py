import os
import threading
import time
import RPi.GPIO as GPIO
#import smbus2 as smbus

##local references
import picamera_video as video
import thermal_AMG8833 as grideye
import temperature_BME280 as temperature
import getCPUserial as serial
#import someaudio as audio
import airquality_SGP30 as airquality

##GPIO setting up for PIR and LED
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.IN)
GPIO.setup(24, GPIO.OUT)

##Specify the functoin in each ref
str_video = "video.capture(25,60)"
#str_audio =
#ge = grideye.GridEye(i2c_bus=smbus.SMBusWrapper(1))
#str_grideye = "ge.get_thermistor_temp()"
str_grideye = "GridEye.get_thermistor_temp()"
str_temperature = "temperature.readBME280All()"
str_airquality = "airquality.getvalue()"

##generating threads
thread_video = threading.Thread(target = str_video)
#thread_audio = threading.Thread(target = str_audio)
thread_grideye = threading.Thread(target = str_grideye)
thread_temperature = threading.Thread(target = str_temperature)
thread_airquality = threading.Thread(target = str_airquality)

##main
try:
    time.sleep(2) # to stabilize sensor
    SerialNumber = serial.getserial()
    while True:
        if GPIO.input(4):
            Startingtime = time.time()
            Log = open("%s" % Startingtime + "_%s.txt" % SerialNumber,'a')
            GPIO.output(24, True)
#            time.sleep(0.5) #LED turns on for 0.5 sec
#            GPIO.output(24, False)
            
            print("Motion Detected. Data gatehring in progress...")
            
            ##Capturing video clip. output will be "temporary.h264" and be converted later
            thread_video.start()
            
            for i in range(750):
                ##?format/dimension? Getting data from grideye in 25fps
                value0 = thread_grideye.start()
                ##?format/dimension? Getting data from BMP280 and SGP30 in 1 fps (putting NA for most points)
                if i % 25 == 0:
                    value1 = thread_temperature.start()
                    value2 = thread_airquality.start()
                else:
                    value1 = "NA"
                    value2 = "NA"
                ##Writing gatehred values
                Log.write("%s," % value0)
                Log.write("%s," % value1)
                Log.write("%s\n" % value2)                
                time.sleep(1/25)
            
            ##converting video clip to mp4
            os.system("MP4Box -add temporary.h264" + " %s.mp4" % Startingtime)
            os.system("rm temporary.h264")
            
            ##For PIR
            time.sleep(2) #to avoid multiple detection
            
        time.sleep(0.1) #loop delay, should be less than detection delay

except:
    GPIO.cleanup()
