from pybricks.parameters import Direction, Port, Button
from pybricks.robotics import DriveBase
from pybricks.ev3devices import ColorSensor, Motor, InfraredSensor
from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import ImageFile
from pybricks.tools import wait
from modules import *
import _thread

# Initialize the EV3 brick
ev3 = EV3Brick()

# Constants for line following and turning
MAX_SPEED = 75
SLOW_SPEED = 50
REDUCE_SPEED = 1.25

class Robot(DriveBase):
  kp, ki, kd = 0, 0, 0
  black, mid, white = [0, 0], [0, 0], [0, 0]

  # Initial variables for PID controller
  integral, derivative, last_error = 0, 0, 0

  # Constructor
  def __init__(self, left_motor: Motor, right_motor: Motor, wheel_diameter: int | float, axle_track: int | float, left_sensor: ColorSensor, right_sensor: ColorSensor, ir_sensor: InfraredSensor):
    super().__init__(left_motor, right_motor, wheel_diameter, axle_track)
    self.left_sensor = left_sensor
    self.right_sensor = right_sensor
    self.left_motor = left_motor
    self.right_motor = right_motor
    self.ir_sensor = ir_sensor

  # Centralize the robot on the line using PID algorithm
  def center_line(self):
    integral, derivative, last_error = 0, 0, 0
    while self.left_sensor.reflection() - self.right_sensor.reflection() != 0:
      error = self.left_sensor.reflection() - self.right_sensor.reflection()
      integral += error
      derivative = error - last_error
      control = (self.kp * error) + (self.ki * integral) + (self.kd * derivative)

      self.drive(0, control)
      last_error = error
      wait(10)
    self.stop()

  def intersection(self):
    Logger.info("Intersection\ndetected")
    if self.right_sensor.reflection() < self.mid[1]:
      self.straight(50) # Drive distance
                        # TODO: Drive until the color sensor pass green indicator

      # Turn right until left sensor reflection is greater than constant
      while self.left_sensor.reflection() > (self.mid[0] - 15):
        self.drive(0, 50)
    if self.left_sensor.reflection() < (self.mid[0] - 15):
      self.straight(50) # Drive distance
                        # TODO: Drive until the color sensor pass green indicator

      # Turn left until right sensor reflection is greater than constant
      while self.right_sensor.reflection() > self.mid[1]:
        self.drive(0, -50)

    self.center_line() # Center on the line
    ev3.screen.clear()

  def detect_green(self):
    wait(5) # Small delay so it detects the green

    color_left, color_right = self.left_sensor.color(), self.right_sensor
    if is_green(color_left) and is_green(color_right.color()): # Turn backwards
      ev3.screen.load_image(ImageFile.BACKWARD)
      self.straight(125)
      self.turn(180)
    elif is_green(color_left): # Turn left
      ev3.screen.load_image(ImageFile.LEFT)
      self.straight(125)
      self.turn(-90)
    elif is_green(color_right.color()): # Turn right
      ev3.screen.load_image(ImageFile.RIGHT)
      self.straight(125)
      self.turn(90)

    ev3.screen.clear()

  def avoid_obstacle(self):
    self.straight(-5)

    self.turn(90)
    self.straight(130)

    self.turn(-90)
    self.straight(280) # Object distance to drive

    self.turn(-90)

    # Drive forward until finds a line
    while self.left_sensor.reflection() > self.mid[0]:
      self.drive(SLOW_SPEED, 0)

    self.straight(90) # Drive the diff of the sensor to motor distance
    self.turn(90) # Turn 90 degrees to the line direction
    self.center_line() # Center line

  def calculate_pid(self):
    # Calculates the PID algorithm
    error = self.left_sensor.reflection() - self.right_sensor.reflection()
    self.integral += error
    derivative = error - self.last_error

    control = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
    reduced_speed = max(min(MAX_SPEED, MAX_SPEED - (abs(control) * REDUCE_SPEED)), 5) # Use the control var to calculate the speed

    self.drive(reduced_speed, control)
    last_error = error

  def start(self):
    while True in [all(value == 0 for value in self.black), all(value == 0 for value in self.white), all(value == 0 for value in self.mid)]:
      Logger.warning("Robot isn't\ncalibrated")
      wait(5000)
      self.calibrate()

    while True:
      # Detect obstacle using IR sensor
      if self.ir_sensor.distance() <= 20:
          self.stop()
          self.avoid_obstacle()
      elif is_green(self.left_sensor.color()) or is_green(self.right_sensor.color()):
              self.stop()
              self.detect_green()
      # Verfies for green indicator or ignore intersection
      elif (self.left_sensor.reflection() < self.mid[0] or self.right_sensor.reflection() < self.mid[1]) and not (is_green(self.left_sensor.color()) or is_green(self.right_sensor.color())):
          self.intersection()
      else:
        self.calculate_pid()

  def calibrate(self):
    ev3.screen.load_image(ImageFile.WINKING)
    while True:
      wait(150)
      if Button.LEFT in ev3.buttons.pressed():
        self.black = [self.left_sensor.reflection(), self.right_sensor.reflection()]
        Logger.info("Black value\n\tupdated to:\n\t\t{} {}".format(self.black[0], self.black[1]))

      elif Button.CENTER in ev3.buttons.pressed():
        self.mid = [self.left_sensor.reflection(), self.right_sensor.reflection()]
        Logger.info("Middle value\n\tupdated to:\n\t\t{} {}".format(self.mid[0], self.mid[1]))

      elif Button.RIGHT in ev3.buttons.pressed():
        self.white = [self.left_sensor.reflection(), self.right_sensor.reflection()]
        Logger.info("White value\n\tupdated to:\n\t\t{} {}".format(self.white[0], self.white[1]))

      elif Button.DOWN in ev3.buttons.pressed():
        Logger.info("Calibration finished")
        wait(500)
        ev3.screen.clear()
        break

  # Configure KP, KI and KD values for PID algorithm
  def configure(self, kp: float | int, ki: float | int, kd: float | int):
    self.kp = kp
    self.ki = ki
    self.kd = kd
