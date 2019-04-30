Multi sensor works only when a motion is detected, start gatehring multiple data<br>

<h3> Video:</h3>
Picam XX <br>
 <I>"picamera_video.py"</I>: 640x480 resolution, 25 fps, 1 minute per file; stored in <I>"Startingtime(UT).mp4"</I><br>

<h3>Audio:</h3>
Microphone XX <br>
 <I>"abcdef"</I>: XYZ; 1 minute per file; stored in <I>"Startingtime(UT).wav"</I><br>

<h3>Thermal:</h3>
AMG8833 Grideye <br>
<I>"thermal_AMG8833.py"</I>: 8x8 matrix, 25fps, 1 minute per file<br>
Thermal sensor data will be stored in <I>"Startingtime(UT)_grideye.txt"</I><br>

<h3>Temperature:</h3>
BME280<br>
<I>"temperature_BME280.py"</I>: temperature, pressue, humidity; every minutes<br>
<h3>Air quality:</h3>
SGP30<br>
<I>"airquality_SGP30.py"</I>: eCO2 (ppm), VOC (bpm); every minutes<br>
Temperature and Air quality data will be stored in <I>"Startingtime(UT)_air.txt"</I><br>
