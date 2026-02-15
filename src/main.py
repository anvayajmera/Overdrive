import cv2, time, os
from classes.Robot import Robot
from classes.linetrace import linetrace, init
from classes.constants import GUI, TIMESTEP, FPS

if GUI:
    os.environ.setdefault("DISPLAY", ":0")

r = Robot()

init()

next_t = time.perf_counter()

try:
    while True:
        now = time.perf_counter()
        if now < next_t:
            time.sleep(next_t - now)
        next_t += TIMESTEP

        linetrace()
        
        if GUI:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
except KeyboardInterrupt:
    print("Stopping bc of ctrl + C.")
    r.stop()    

r.cleanup()
