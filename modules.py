from pybricks.ev3devices import ColorSensor, Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Color
from pybricks.tools import wait

from logger import Logger

# Initialize the EV3 brick
ev3 = EV3Brick()

def is_green(color: Color | None):
    return (color == Color.GREEN)

def test_port(type, port: Port, **kwargs):
    try:
        type(port, **kwargs)
    except OSError as e:
        # If an OSError was raised, we can check what
        # kind of error this was, like ENODEV.
        # ENODEV = 19, which means that there is no device
        if e.args[0] == 19:
            Logger.warning("No device on\n", port)
            wait(5000)
        else:
            Logger.error("Another error occurred: ", e)
    return type(port, **kwargs)

def angle_error(angle: int):
	angle = angle % 90
	if (angle >= 45): angle -= 2 * (angle - 45)
	return angle
