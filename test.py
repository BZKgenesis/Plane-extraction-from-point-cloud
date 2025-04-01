import RPyLIDAR
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

    sensor.start_standard_scan()
    #sensor.start_express_scan()

    sensor.stop_scan()
    sensor.stop_motor()
    sensor.disconnect()

except:
    print(" *** AN UNKNOWN ERROR OCCURRED *** ")
    sensor.stop_scan()
    sensor.stop_motor()
    sensor.disconnect()