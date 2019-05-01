from smbus2 import SMBusWrapper
from sgp30 import Sgp30
import time

def getvalue():
    with SMBusWrapper(1) as bus:
        sgp=Sgp30(bus,baseline_filename="/tmp/mySGP30_baseline")
        #    sgp.i2c_geral_call() #WARNING: Will reset any device on teh i2cbus that listens for general call
        sgp.init_sgp()
    value = sgp.read_measurements()
    return value
