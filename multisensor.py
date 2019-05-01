import threading
import time
import RPi.GPIO as GPIO

import picamera_video as video
import thermal_AMG8833 as grideye
import temperature_BME280 as temperature
import getCPUserial as serial
#import someaudio as audio
import airquality_Sgp30 as airquality

GPIO.setmode(GPIO.BCM)

GPIO.setup(4, GPIO.IN) #PIR
GPIO.setup(24, GPIO.OUT) #LED

str_video = "video.capture(25,60)"
#str_audio =
ge = thermal_AMG8833.GridEye(i2c_bus=bus)
str_grideye = "ge.get_thermistor_temp()"
str_temperature = "temperature.readBME280All()"
str_airquality = "airquality.getvalue()"

thread_video = threading.Thread(target = str_video)
#thread_audio = threading.Thread(target = str_audio)
thread_grideye = threading.Thread(target = str_grideye)
thread_temperature = threading.Thread(target = str_temperature)
thread_airquality = threading.Thread(target = str_airquality)

try:
    time.sleep(2) # to stabilize sensor
    
    while True:
        if GPIO.input(4):
            Startingtime = time.time()
            GPIO.output(24, True)
#            time.sleep(0.5) #Buzzer turns on for 0.5 sec
#            GPIO.output(24, False)
            brah
            
            print("Motion Detected...")
            time.sleep(2) #to avoid multiple detection
        time.sleep(0.1) #loop delay, should be less than detection delay

except:
    GPIO.cleanup()
