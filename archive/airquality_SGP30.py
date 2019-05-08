from smbus2 import SMBusWrapper
from sgp30 import Sgp30
import time
with SMBusWrapper(1) as bus:
    sgp=Sgp30(bus,baseline_filename="/tmp/mySGP30_baseline") #things thing with the baselinefile is dumb and will be changed in the future
    sgp.i2c_geral_call() #WARNING: Will reset any device on teh i2cbus that listens for general call
    sgp.init_sgp()
    
while True:
    time.sleep(1)
    print(sgp.read_measurements())
#comment

