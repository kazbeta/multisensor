import time

def getserial():
    # Extract serial from cpuinfo file
    #    cpuserial = "0000000000000000"
    cpuserial = "00000000"
    try:
        f = open('/proc/cpuinfo','r')
        for line in f:
            if line[0:6]=='Serial':
                #                cpuserial = line[10:26]
                cpuserial = line[19:26]
        f.close()
    except:
        cpuserial = "ERROR000"
    return cpuserial
