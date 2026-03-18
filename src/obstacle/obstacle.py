from classes.Robot import Robot
from src.constants import FRONT_THRESH


def detect_obstacle():
    r = Robot()

    return all(x <= FRONT_THRESH for x in r.front_distances)
