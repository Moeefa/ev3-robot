from pybricks.hubs import EV3Brick
from _typeshed import SupportsWrite
from typing_extensions import Literal
from pybricks.media.ev3dev import ImageFile, SoundFile

# Initialize the EV3 brick
ev3 = EV3Brick()

class Logger:
  @staticmethod
  def info(*values: object, sep: str | None = " ", end: str | None = "\n", file: SupportsWrite[str] | None = None, flush: Literal[False] = False):
    ev3.screen.print(values)
    print(values)

  @staticmethod
  def warning(*values: object, sep: str | None = " ", end: str | None = "\n", file: SupportsWrite[str] | None = None, flush: Literal[False] = False):
    ev3.screen.load_image(ImageFile.WARNING)
    ev3.speaker.play_file(SoundFile.GENERAL_ALERT)
    Logger.info(values)

  @staticmethod
  def error(*values: object, sep: str | None = " ", end: str | None = "\n", file: SupportsWrite[str] | None = None, flush: Literal[False] = False):
    ev3.screen.load_image(ImageFile.DECLINE)
    ev3.speaker.beep(90, 250)
    Logger.info(values)
