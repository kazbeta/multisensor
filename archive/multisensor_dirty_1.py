import RPi.GPIO as GPIO
import time
import threading

GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.IN) #PIR
GPIO.setup(6, GPIO.OUT) #LED

########BME280##########
import smbus
#import time
from ctypes import c_short
from ctypes import c_byte
from ctypes import c_ubyte

DEVICE = 0x77 # Default device I2C address
bus = smbus.SMBus(1)

def getShort(data, index):
    # return two bytes from data as a signed 16-bit value
    return c_short((data[index+1] << 8) + data[index]).value

def getUShort(data, index):
    # return two bytes from data as an unsigned 16-bit value
    return (data[index+1] << 8) + data[index]

def getChar(data,index):
    # return one byte from data as a signed char
    result = data[index]
    if result > 127:
        result -= 256
    return result

def getUChar(data,index):
    # return one byte from data as an unsigned char
    result =  data[index] & 0xFF
    return result

def readBME280ID(addr=DEVICE):
    # Chip ID Register Address
    REG_ID     = 0xD0
    (chip_id, chip_version) = bus.read_i2c_block_data(addr, REG_ID, 2)
    return (chip_id, chip_version)

def readBME280All(addr=DEVICE):
    # Register Addresses
    REG_DATA = 0xF7
    REG_CONTROL = 0xF4
    REG_CONFIG  = 0xF5
                
    REG_CONTROL_HUM = 0xF2
    REG_HUM_MSB = 0xFD
    REG_HUM_LSB = 0xFE
                            
    # Oversample setting - page 27
    OVERSAMPLE_TEMP = 2
    OVERSAMPLE_PRES = 2
    MODE = 1
                                        
    # Oversample setting for humidity register - page 26
    OVERSAMPLE_HUM = 2
    bus.write_byte_data(addr, REG_CONTROL_HUM, OVERSAMPLE_HUM)
    control = OVERSAMPLE_TEMP<<5 | OVERSAMPLE_PRES<<2 | MODE
    bus.write_byte_data(addr, REG_CONTROL, control)

    # Read blocks of calibration data from EEPROM
    # See Page 22 data sheet
    cal1 = bus.read_i2c_block_data(addr, 0x88, 24)
    cal2 = bus.read_i2c_block_data(addr, 0xA1, 1)
    cal3 = bus.read_i2c_block_data(addr, 0xE1, 7)
                                                
    # Convert byte data to word values
    dig_T1 = getUShort(cal1, 0)
    dig_T2 = getShort(cal1, 2)
    dig_T3 = getShort(cal1, 4)

    dig_P1 = getUShort(cal1, 6)
    dig_P2 = getShort(cal1, 8)
    dig_P3 = getShort(cal1, 10)
    dig_P4 = getShort(cal1, 12)
    dig_P5 = getShort(cal1, 14)
    dig_P6 = getShort(cal1, 16)
    dig_P7 = getShort(cal1, 18)
    dig_P8 = getShort(cal1, 20)
    dig_P9 = getShort(cal1, 22)
        
    dig_H1 = getUChar(cal2, 0)
    dig_H2 = getShort(cal3, 0)
    dig_H3 = getUChar(cal3, 2)
            
    dig_H4 = getChar(cal3, 3)
    dig_H4 = (dig_H4 << 24) >> 20
    dig_H4 = dig_H4 | (getChar(cal3, 4) & 0x0F)
                
    dig_H5 = getChar(cal3, 5)
    dig_H5 = (dig_H5 << 24) >> 20
    dig_H5 = dig_H5 | (getUChar(cal3, 4) >> 4 & 0x0F)
                    
    dig_H6 = getChar(cal3, 6)
        
    # Wait in ms (Datasheet Appendix B: Measurement time and current calculation)
    wait_time = 1.25 + (2.3 * OVERSAMPLE_TEMP) + ((2.3 * OVERSAMPLE_PRES) + 0.575) + ((2.3 * OVERSAMPLE_HUM)+0.575)
    time.sleep(wait_time/1000)  # Wait the required time

    # Read temperature/pressure/humidity
    data = bus.read_i2c_block_data(addr, REG_DATA, 8)
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw = (data[6] << 8) | data[7]
    
    #Refine temperature
    var1 = ((((temp_raw>>3)-(dig_T1<<1)))*(dig_T2)) >> 11
    var2 = (((((temp_raw>>4) - (dig_T1)) * ((temp_raw>>4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
    t_fine = var1+var2
    temperature = float(((t_fine * 5) + 128) >> 8);
            
    # Refine pressure and adjust for temperature
    var1 = t_fine / 2.0 - 64000.0
    var2 = var1 * var1 * dig_P6 / 32768.0
    var2 = var2 + var1 * dig_P5 * 2.0
    var2 = var2 / 4.0 + dig_P4 * 65536.0
    var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * dig_P1
    if var1 == 0:
        pressure=0
    else:
        pressure = 1048576.0 - pres_raw
        pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
        var1 = dig_P9 * pressure * pressure / 2147483648.0
        var2 = pressure * dig_P8 / 32768.0
        pressure = pressure + (var1 + var2 + dig_P7) / 16.0
    # Refine humidity
    humidity = t_fine - 76800.0
    humidity = (hum_raw - (dig_H4 * 64.0 + dig_H5 / 16384.0 * humidity)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * humidity * (1.0 + dig_H3 / 67108864.0 * humidity)))
    humidity = humidity * (1.0 - dig_H1 * humidity / 524288.0)
    if humidity > 100:
        humidity = 100
    elif humidity < 0:
        humidity = 0

    return temperature/100.0,pressure/100.0,humidity

#def main():
#
#    (chip_id, chip_version) = readBME280ID()
#    print "Chip ID     :", chip_id
#    print "Version     :", chip_version
#
#    temperature,pressure,humidity = readBME280All()
#
#    print "Temperature : ", temperature, "C"
#    print "Pressure : ", pressure, "hPa"
#    print "Humidity : ", humidity, "%"
#
#if __name__=="__main__":
#    main()

########BME280##########

########SGP30##########
from smbus2 import SMBusWrapper
from sgp30 import Sgp30

#with SMBusWrapper(1) as bus:
#    sgp=Sgp30(bus,baseline_filename="/tmp/mySGP30_baseline") #things thing with the baselinefile is dumb and will be changed in the future
#    sgp.i2c_geral_call() #WARNING: Will reset any device on teh i2cbus that listens for general call
#    sgp.init_sgp()

#while True:
#    time.sleep(1)
#    print(sgp.read_measurements())

########SGP30##########

########Picamera##########
import picamera
import os

#filename = str(time.time())
#
#camera = picamera.PiCamera()
#camera.resolution = (640, 480)
#camera.framerate = 25
#camera.start_recording(filename + ".h264")
#camera.wait_recording(30)
#camera.stop_recording()
#
#os.system("MP4Box -add %s.h264" % filename + " %s.mp4" % time.time())
#os.system("rm %s.h264" % filename)


########Picamera##########

########AMG8833##########
class GridEye():
    def __init__(self, i2c_address=0x69, i2c_bus=smbus.SMBus(1)):
        self.i2c = {"bus": i2c_bus, "address": i2c_address}
        self.read_byte = self.i2c["bus"].read_byte_data
        self.write_byte = self.i2c["bus"].write_byte_data
        self.modes = {  0x00: "NORM",
                        0x10: "SLEEP",
                        0x20: "STANDBY60",
                        0x21: "STANDBY10"
                    }

    def get_mode(self):
        mode = self.read_byte(self.i2c["address"], 0x00)
        return self.modes.get(mode)
    
    def set_mode(self, mode="NORM"):
        if isinstance(mode, str):
            mode = {v: k for k, v in self.modes.items()}.get(mode)
        self.write_byte(self.i2c["address"], 0x00, mode)
    
    def reset(self, flags_only=False):
        if flags_only:
            reset = 0x30
        else:
            reset = 0x3F
        self.write_byte(self.i2c["address"], 0x01, reset)
    
    def get_fps(self):
        fps = self.read_byte(self.i2c["address"], 0x02)
        if fps is 0:
            return 10
        else:
            return 1

    def set_fps(self, fps=10):
        if fps is not 1:
            fps = 0x00
        self.write_byte(self.i2c["address"], 0x02, fps)

    def get_interrupt_ctrl(self):
        intc = self.read_byte(self.i2c["address"], 0x03)
        return (1 & intc is 1), (2 & intc is 2)

    def set_interupt_ctrl(self, enabled=False, mode=False):
        """
        mode = False -> difference mode
        mode = True  -> absolute mode
        """
        intc = 0x0
        if enabled:
            intc += 1
        if mode:
            intc += 2
        self.write_byte(self.i2c["address"], 0x03, intc)

    def get_states(self):
        """
        returns a tuple of (
            Interrupt Outbreak,
            Temperature Output Overflow,
            Thermistor Temperature Output Overflow
            )
        from 0x04
        """
        state = self.read_byte(self.i2c["address"], 0x04)
        return (1 & state is 1), (2 & state is 2), (4 & state is 4)

    def clear_states(self, interrupt=False, temp_overflow=False, thermistor_overflow=False):
        clear = 0x00
        if interrupt:
            clear += 1
        if temp_overflow:
            clear += 2
        if thermistor_overflow:
            clear += 4
        self.write_byte(self.i2c["address"], 0x05, clear)

    def set_moving_average(self, twice=False):
        # not quite sure how this works.
        if twice:
            value = 0x20
        else:
            value = 0
        self.write_byte(self.i2c["address"], 0x07, value)

    def set_interrupt_limits(self, lower_limit, upper_limit, hysteresis_level):
        lower_limit = split_in_2bytes(int2twoscomplement(lower_limit))
        upper_limit = split_in_2bytes(int2twoscomplement(upper_limit))
        hysteresis_level = split_in_2bytes(int2twoscomplement(hysteresis_level))
        self.write_byte(self.i2c["address"], 0x08, upper_limit[1])
        self.write_byte(self.i2c["address"], 0x09, upper_limit[0])
    
        self.write_byte(self.i2c["address"], 0x0A, lower_limit[1])
        self.write_byte(self.i2c["address"], 0x0B, lower_limit[0])
        
        self.write_byte(self.i2c["address"], 0x0C, hysteresis_level[1])
        self.write_byte(self.i2c["address"], 0x0D, hysteresis_level[0])
    
    def get_interrupts(self, reset=False):
        """
        Returns current interrupts and optionally resets the interrupt table.
        Format is a list of tuples (line, pixel in line)
        """
        interrupts = []
        data = self.i2c["bus"].read_i2c_block_data(self.i2c["address"], 0x10, 8)
        for i in range(8):
            for bit in range(8):
                if data[i] & 2**bit != 0:
                    interrupts.append((i, bit))
    
        if reset:
            self.clear_states(interrupt=True)
        return interrupts
    
    def get_thermistor_temp(self, raw=False):
        """
        returns the thermistor temperature in .25C resolution
        TODO: high res option with possible 0.0625C resolution
        """
        upper = self.read_byte(self.i2c["address"], 0x0F) << 6
        lower = self.read_byte(self.i2c["address"], 0x0E) >> 2
        complete = upper + lower
        if not raw:
            if complete > 512:
                complete -= 1024
                complete = -complete
            return complete / 4
        else:
            return complete

def get_sensor_data(self, mode="TEMP", remap=True):
    """
    returns the sensor data, supporting different modes
    "TEMP" -> [8][8] Array of temp values
    "GRAYIMAGE" -> a 8x8 pixel image with optional remapping
    +
    min, max values as [value, x,y]
    NOTE: READ is done per line. the raspberry pi doesn't like reading 128 bytes at once.
    """
    lines = []
    minv = [500, 0, 0]
    maxv = [-500, 0, 0]
    for line in range(8):
        offset = 0x80+line*16
        block = self.i2c["bus"].read_i2c_block_data(self.i2c["address"], offset, 16)
        values = []
        for i in range(0, 16, 2):
            upper = block[i+1] << 8
            lower = block[i]
            val = upper + lower
            if 2048 & val == 2048:
                val -= 4096
                val = val/4
            if val < minv[0]:
                minv = [val, i//2, line]
            if val > maxv[0]:
                maxv = [val, i//2, line]
            values.append(val)
        lines.append(values)
    if mode is "TEMP":
        return lines, minv, maxv
    elif mode is "GRAYIMAGE":
        img = Image.new("L", (8, 8))
        pixel = img.load()
        for i in range(8):
            for j in range(8):
                if remap:
                    value = maprange((minv[0], maxv[0]), (0, 255), lines[i][j])
                    value = (int(value),)
                else:
                    value = (int(lines[i][j]), )
                pixel[i, j] = value
        return img, minv, maxv


def int2twoscomplement(value, bits=12):
    """returning a integer which is equal to value as two's complement"""
    if value > 0:
        return value
    else:
        return (1 << bits) + num
def split_in_2bytes(value):
    """
    Returns a tuple with 2 integers (upper,lower) matching the according bytes
    The AMG88 usually uses 2 byte to store 12bit values.
    """
    upper = value >> 9
    lower = 0b011111111 & value
    return (upper, lower)

def maprange(a, b, s):
    """remap values linear to a new range"""
    (a1, a2), (b1, b2) = a, b
    return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

#if __name__ == '__main__':
#    from time import sleep
#    from pprint import pprint
#
#    with smbus.SMBusWrapper(1) as bus:
#        print("Init GridEye88xx")
#        ge = GridEye(i2c_bus=bus)
#        print("Save Sensor Data As heatmap.png")
#        image = ge.get_sensor_data("GRAYIMAGE")[0]
#        image.save("heatmap.png", "PNG")
#        while True:
#            print("Thermistor Temperature is: %fC" % ge.get_thermistor_temp())
#            print("Current Sensor Data:")
#            pprint(ge.get_sensor_data()[0])
#            sleep(0.5)

########AMG8833##########

##generating threads
#thread_video = threading.Thread(target = 
#                                filename = str(time.time())
#                                camera = picamera.PiCamera()
#                                camera.resolution = (640, 480)
#                                camera.framerate = 25
#                                camera.start_recording(filename + ".h264")
#                                camera.wait_recording(30)
#                                camera.stop_recording()
#                                os.system("MP4Box -add %s.h264" % filename + " %s.mp4" % time.time())
#                                os.system("rm %s.h264" % filename)
#                               )
#thread_grideye = threading.Thread(target = str_grideye)
#thread_temperature = threading.Thread(target = str_temperature)
#thread_airquality = threading.Thread(target = str_airquality)

try:
    time.sleep(3) # to stabilize sensor
    GPIO.output(6, False)
    
    while True:
        if GPIO.input(4):
            Startingtime = time.time()
            Log = open("%s" % Startingtime + "_%s.txt" % SerialNumber,'a')
            #Turn on LED while getting data
            GPIO.output(6, True)
            print(Startingtime),
            print("Motion Detected. Data collection in progress...")
            time.sleep(0.001)
            #Picamera
            #thread_video
            with smbus.SMBusWrapper(1) as bus:
                for i in range(750):
                    #Getting data from grideye in 25fps
                    GPIO.output(6, True)
                    ge = GridEye(i2c_bus=bus)
                    AMG8833_value = ge.get_thermistor_temp()
                    #Getting data from BMP280 and SGP30 in 1 fps (putting NA for most points)
                    if i % 25 == 0:
                        #BME280
                        temperature,pressure,humidity = readBME280All()
                        #SGP30
                        sgp=Sgp30(bus,baseline_filename="/tmp/mySGP30_baseline")
                        sgp.init_sgp()
                        sgp30_value = sgp.read_measurements()
                    else:
                        temperature,pressure,humidity = "NA", "NA", "NA"
                        sgp30_value = "NA"
                    ##Writing gatehred values
                    Log.write("%s," % AMG8833_value)
                    Log.write("%s," % temperature,pressure,humidity)
                    Log.write("%s\n" % sgp30_value)
                    time.sleep(1/25)
                    GPIO.output(6, False)
        GPIO.output(6, False)

except KeyboardInterrupt:
    GPIO.cleanup()
