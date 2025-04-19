import time
import RPyLIDAR
from utils import getRandomPoint

def fakeScan():
    scanIndex = 0
    index = 1
    while True:
        index +=1
        time.sleep(0.001)
        point = getRandomPoint((1, 1), 0.24, 0.25)
        yield (scanIndex, point[0], point[1], 0, 47)
        if index >= 360:
            index = 0
            scanIndex += 1

def oneScan():
    
    try:
        sensor = RPyLIDAR.RPyLIDAR('COM3',    115200,       1,         700,      False,       True)
        sensor.connect()
        sensor.stop_scan()
        sensor.stop_motor()

        sensor.get_device_info()
        sensor.get_device_health()
        sensor.get_sample_rate()

        #sensor.start_motor_slow()
        #sensor.start_motor_14()
        #sensor.start_motor_half()
        sensor.start_motor_34()
        #sensor.start_motor_max()

        yield sensor.start_standard_scan() # Count, X, Y, Z, quality


        sensor.stop_scan()
        sensor.stop_motor()
        sensor.disconnect()

    except:
        print(" *** AN UNKNOWN ERROR OCCURRED *** ")
        sensor.stop_scan()
        sensor.stop_motor()
        sensor.disconnect()
    