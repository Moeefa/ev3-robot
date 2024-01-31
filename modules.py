from pybricks.ev3devices import ColorSensor, Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Color
from pybricks.tools import wait

# Initialize the EV3 brick
ev3 = EV3Brick()

def rgb_to_hsv(r: float, g: float, b: float):
    maxc = max(r, g, b)
    minc = min(r, g, b)
    rangec = (maxc-minc)
    v = maxc
    if minc == maxc:
        return 0.0, 0.0, v
    s = rangec / maxc
    rc = (maxc-r) / rangec
    gc = (maxc-g) / rangec
    bc = (maxc-b) / rangec
    if r == maxc:
        h = bc-gc
    elif g == maxc:
        h = 2.0+rc-bc
    else:
        h = 4.0+gc-rc
    h = (h/6.0) % 1.0
    return h, s, v

def is_green(color: Color):
    # rgb = sensor.rgb()
    return (color == Color.GREEN) # (((rgb[2] < 0.8 * rgb[1]) and (rgb[0] < .64 * rgb[1])) or

def test_port(type, port: Port, **kwargs):
    try:
        type(port, **kwargs)
    except OSError as e:
        # If an OSError was raised, we can check what
        # kind of error this was, like ENODEV.
        if e.args[0] == 19:
            ev3.screen.print("No device on\n", port)
            print("No device on", port)
            ev3.speaker.beep(50, 150)
            wait(5000)
        else:
            print("Another error occurred: ", e)
    return type(port, **kwargs)

def angle_error(angle: int):
	angle = angle % 90
	if (angle >= 45): angle -= 2 * (angle - 45)
	return angle