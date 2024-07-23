#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, InfraredSensor
from pybricks.parameters import Port, Direction
from structures import Robot

from modules import *

# Initialize the color sensors
sensor_left = test_port(ColorSensor, Port.S2)
sensor_right = test_port(ColorSensor, Port.S4)
sensor_mid = test_port(ColorSensor, Port.S3)
ir_sensor = test_port(InfraredSensor, Port.S1)

# Initialize the drive base
left_motor = test_port(Motor, Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = test_port(Motor, Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
door = test_port(Motor, Port.C)
wheel_diameter = 32 # mm
axle_track = 190 # mm
robot = Robot(
  left_motor,
  right_motor,
  wheel_diameter,
  axle_track,
  sensor_left,
  sensor_right,
  ir_sensor
)

# Constants for the PID configuration
KP = 3.5  # Proportional gain
KI = 0.0005  # Integral gain
KD = 0.01  # Derivative gain
robot.configure(KP, KI, KD) # Configure the PID

if __name__ == "__main__":
  try:
    robot.calibrate()
    robot.start()
    # print(sensor_left.color(), sensor_right.color())
  except KeyboardInterrupt:
    # Exit the program when Ctrl + C is pressed
    robot.stop()
