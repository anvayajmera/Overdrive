import cv2, time
from classes.Robot import Robot
from classes.linetrace import linetrace, init
from classes.constants import GUI

r = Robot()

init();

try:
    while True:
        time.sleep(0.05)
        
        linetrace()
        
        if GUI:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
except KeyboardInterrupt:
    print("Stopping bc of ctrl + C.")
    r.stop()    

r.cleanup()
