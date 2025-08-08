import cv2

camera1 = cv2.VideoCapture(0, cv2.CAP_DSHOW)

while camera1.isOpened():
    # ret indicates available frame
    ret, frame = camera1.read()
    if ret:
        cv2.imshow("frame", frame)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break
    else:
        break

camera1.release()
cv2.destroyAllWindows()