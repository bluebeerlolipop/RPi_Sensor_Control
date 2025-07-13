import board
import adafruit_bno055
import time

i2c = board.I2C() # uses board.SCL and board.SDA
sensor = adafruit_bno055.BNO055_I2C(i2c)
# sensor.mode = adafruit_bno055.M4G_MODE

while True:
    print('Temperature: {} degrees C'.format(sensor.temperature))
    print('Accelerometer (m/s^2): {}'.format(sensor.acceleration))
    print('Magnetometer (microteslas): {}'.format(sensor.magnetic))
    print('Gyroscope (deg/sec): {}'.format(sensor.gyro))
    print('Euler angle: {}'.format(sensor.euler))
    print('Quaternion: {}'.format(sensor.quaternion))
    print('Linear acceleration (m/s^2): {}'.format(sensor.linear_acceleration))
    print('Gravity (m/s^2): {}'.format(sensor.gravity))
    print('Sensor mode : {}'.format(sensor.mode))
    print(type(sensor.quaternion))
    print()

    time.sleep(1)