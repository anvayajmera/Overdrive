import cv2, time, os
from classes.Robot import Robot
from classes.linetrace import linetrace, init
from classes.constants import GUI, TIMESTEP

if GUI:
    os.environ.setdefault("DISPLAY", ":0")

r = Robot()

init();

try:
    while True:
        time.sleep(TIMESTEP)

        linetrace()
        
        if GUI:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
except KeyboardInterrupt:
    print("Stopping bc of ctrl + C.")
    r.stop()    

r.cleanup()
