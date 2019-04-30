Multi sensor works with following conditions:
Only when a motion is detected, start gatehring multiple data:

<h3> Video:</h3>
 Picam XX "picamera_video.py": 640x480 resolution, 25 fps, 1 minute per file; stored in "Startingtime(UT).mp4"<br>

<h3>Audio:</h3>
 Microphone XX "": XYZ; 1 minute per file; stored in "Startingtime(UT).wav"<br>

<h3>Thermal:</h3>
 AMG8833 Grideye "thermal_AMG8833.py": 8x8 matrix, 25fps, 1 minute per file<br>
Thermal sensor data will be stored in "Startingtime(UT)_grideye.txt"<br>

<h3>Temperature:</h3>
 BME280 "temperature_BME280.py": temperature, pressue, humidity; every minutes<br>
<h3>Air quality:</h3>
 SGP30 "airquality_SGP30.py": eCO2 (ppm), VOC (bpm); every minutes<br>
Temperature and Air quality data will be stored in "Startingtime(UT)_air.txt"<br>
