HOST = "localhost"
PORT = 4223
UID = "6xgbpL"

from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu import IMU

ipcon = IPConnection() # Create IP connection
imu = IMU(UID, ipcon) # Create device object

ipcon.connect(HOST, PORT) # Connect to brickd
# Don't use device before ipcon is connected

imu.orientation_calculation_off()

imu.leds_off()

imu.set_convergence_speed(5)
