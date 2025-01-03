import time
import serial
import threading
import uart
import constants
from identify import VideoCapture, show_mission, str_int
from control import Motor,StepMotor
import cv2
if __name__=='__main__':
    cap = VideoCapture(1)
    while True:
        center = cap.detect_circle(1)
        cap.cv_imshow()
        if cv2.waitKey(100)  == ord('q'):
            break
        if center:
            print(f"x_delt{center[0]-320},y_delt{center[1]-240}")
    cap.release()
    cv2.destroyAllWindows()
            