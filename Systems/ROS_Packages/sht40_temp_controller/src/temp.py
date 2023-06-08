#!/usr/bin/env python3

import time
import board
import adafruit_sht4x
import rospy

if __name__ == '__main__':
    rospy.init_node('temp_sensor')
    rospy.loginfo('Temperature sensor node has started!')

    i2c = board.I2C()   # uses board.SCL and board.SDA
    sht = adafruit_sht4x.SHT4x(i2c)
    print("Found SHT4x with serial number", hex(sht.serial_number))

    sht.mode = adafruit_sht4x.Mode.NOHEAT_HIGHPRECISION
    # Can also set the mode to enable heater
    # sht.mode = adafruit_sht4x.Mode.LOWHEAT_100MS
    print("Current mode is: ", adafruit_sht4x.Mode.string[sht.mode])
    
    rate = rospy.Rate(0.2)

    while not rospy.is_shutdown():
        temperature, relative_humidity = sht.measurements
        print("Temperature: %0.1f C" % temperature)
        print("Humidity: %0.1f %%" % relative_humidity)
        print("")
        rate.sleep()
    rospy.logerr('node has been terminated!')
