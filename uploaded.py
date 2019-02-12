import pipeline_laptop
import cv2 as cv

pipe = pipeline_laptop.GripPipeline()
cap = cv.VideoCapture(1)

CONTOUR_AREA_THRESHOLD = 300
RED = (0, 0, 255)
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)
BLACK = (0, 0, 0)
WIDTH = cap.get(3)
HEIGHT = cap.get(4)


def get_center(rect):  # gets center from bounding box
    x, y, w, h = rect
    cx = int((2 * x + w) / 2)
    cy = int((2 * y + h) / 2)
    return [cx, cy]


while True:
    retval, img = cap.read()

    pipe.process(img)  # processes image via GRIP pipeline
    img = cv.cvtColor(pipe.cv_erode_output, cv.COLOR_GRAY2BGR)  # converts to color
    contours = pipe.find_contours_output  # contour data

    filtered_contours = []

    left_target = None
    right_target = None

    for c in contours:
        area = cv.contourArea(c)

        if area > CONTOUR_AREA_THRESHOLD:  # filter out contours by size
            perimeter = cv.arcLength(c, True)  # calculates corners of targets
            approx = cv.approxPolyDP(c, 0.04 * perimeter, True)

            cv.drawContours(img, approx, -1, (0, 0, 255), 3)

            filtered_contours.append(approx)

            print(approx)
