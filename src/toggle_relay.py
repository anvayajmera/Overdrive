import os
import time

import cv2

from classes.Robot import Robot
from constants import GREEN_CHECK_PERIOD_FRAMES, GUI, TIMESTEP, X11
from gap.gap import gap
from green_square.green_square import green_square
from linetrace.linetrace import init, linetrace
from obstacle.obstacle import obstacle
from victims.victim import victim, victim_init

if GUI and not X11:
    os.environ.setdefault("DISPLAY", ":0")

r = Robot()
r.set_relay(False)
