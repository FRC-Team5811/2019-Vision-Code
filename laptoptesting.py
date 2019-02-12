import pipeline
import cv2 as cv
import os
import threading
from networktables import NetworkTables

# setting camera exposure settings
os.system("v4l2-ctl -d /dev/video1 -c exposure_auto=1")
os.system("v4l2-ctl -d /dev/video1 -c exposure_absolute=0")

pipe = pipeline.GripPipeline()
cap = cv.VideoCapture(1)

cond = threading.Condition()
notified = [False]

CONTOUR_AREA_THRESHOLD = 300
RED = (0, 0, 255)
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)
BLACK = (0, 0, 0)
WIDTH = cap.get(3)
HEIGHT = cap.get(4)


def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()


def get_center(rect):  # gets center from bounding box
    x, y, w, h = rect
    cx = int((2 * x + w) / 2)
    cy = int((2 * y + h) / 2)
    return [cx, cy]


NetworkTables.initialize(server='10.58.11.88')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

print("Connected!")


while True:
    retval, img = cap.read()

    pipe.process(img)  # processes image via GRIP pipeline
    img = cv.cvtColor(pipe.cv_erode_output, cv.COLOR_GRAY2BGR)  # converts to color
    contours = pipe.find_contours_output  # contour data

    filtered_contours = []

    left_target = None
    right_target = None

    print("23/33 lmao")

    for c in contours:
        area = cv.contourArea(c)

        if area > CONTOUR_AREA_THRESHOLD:  # filter out contours by size
            perimeter = cv.arcLength(c, True)  # calculates corners of targets
            approx = cv.approxPolyDP(c, 0.04 * perimeter, True)

            cv.drawContours(img, approx, -1, (0, 0, 255), 3)

            filtered_contours.append(approx)


    cv.imshow("Vision", img)
    cv.waitKey(25)

    sd = NetworkTables.getTable('SmartDashboard')
    sd.putNumber('yeet', filtered_contours[0])

