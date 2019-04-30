Multi sensor works with following conditions:
Only when a motion is detected, start gatehring multiple data:

Video:
 Picam XX "picamera_video.py": 640x480 resolution, 25 fps, 1 minute per file; stored in "Startingtime(UT).mp4"

Audio:
 Microphone XX "": XYZ; 1 minute per file; stored in "Startingtime(UT).wav"

Thermal:
 AMG8833 Grideye "thermal_AMG8833.py": 8x8 matrix, 25fps, 1 minute per file
Thermal sensor data will be stored in "Startingtime(UT)_grideye.txt"

Temperature:
 BME280 "temperature_BME280.py": temperature, pressue, humidity; every minutes
Air quality:
 SGP30 "airquality_SGP30.py": eCO2 (ppm), VOC (bpm); every minutes
Temperature and Air quality data will be stored in "Startingtime(UT)_air.txt"
