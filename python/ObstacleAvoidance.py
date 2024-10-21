import time

import busio
import adafruit_vl53l0x

# Initialize I2C bus and sensor.
i2c = busio.I2C(3, 2)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

# Main loop will read the range and print it every second.
while True:
    distance = vl53.range
    stop = False
    if 1000 > distance > 100: #mm
        stop = True
    print("Distance: {}mm, stop = {}".format(distance, stop))
    time.sleep(1.0)