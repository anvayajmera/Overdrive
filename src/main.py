import cv2, time
from lib.Robot import Robot
from lib.linetrace import linetrace, init
from lib.constants import GUI

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
