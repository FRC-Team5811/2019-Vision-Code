#!/usr/bin/env python3

import numpy
import cv2 as cv
import os
import threading
from networktables import NetworkTables

DEBUG = True

if DEBUG:
    import pipeline_laptop as pipeline
else:
    import pipeline


CAMERA_PORT = 1
os.system("v4l2-ctl -d /dev/video{} -c exposure_auto=1".format(CAMERA_PORT))
os.system("v4l2-ctl -d /dev/video{} -c exposure_absolute=0".format(CAMERA_PORT))
CONTOUR_AREA_THRESHOLD = 250
RED = (0, 0, 255)
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)
BLACK = (0, 0, 0)
cap = cv.VideoCapture(CAMERA_PORT)
WIDTH = int(cap.get(3))
HEIGHT = int(cap.get(4))
CROPPED_HEIGHT = 320
MID_X = int(WIDTH / 2)
ROLLING_AVG_LENGTH = 20
NETWORK_TABLES_ON = False

pipe = pipeline.GripPipeline()


cond = threading.Condition()
notified = [False]

roll_avg = []


def connection_listener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()


def target_merger(left_target, right_target):  # merges data from two targets into a goal
    l_area = left_target['area']
    r_area = right_target['area']
    t_area = l_area + r_area
    ratio = l_area / r_area
    center = midpoint(left_target['center'], right_target['center'])
    xc = center[0]
    yc = center[1]

    if ratio > 1.1:
        side = "left"
    elif ratio < 0.90909090909:
        side = "right"
    else:
        side = "straight"

    if xc < mid_x:
        aimed = "left"
    else:
        aimed = "right"


    data = {
        'side': side,
        'aimed': aimed,
        'left_area': l_area,
        'right_area': r_area,
        'total_area': t_area,
        'area_ratio': ratio,
        'center': center,
        'contour': numpy.concatenate((left_target['contour'], right_target['contour']))
    }

    # print(side, aimed)

    return data


def update_list(val):  # adds new and removes old val from rolling avg
    if len(roll_avg) > ROLLING_AVG_LENGTH:
        del roll_avg[0]
    roll_avg.append(val)


def get_avg():  # calculates avg from list
    if roll_avg:
        return sum(roll_avg) / len(roll_avg)
    else:
        return None


def midpoint(p1, p2):  # calculates midpoint between two points
    x1, y1 = p1
    x2, y2 = p2
    mx = int((x1 + x2) / 2)
    my = int((y1 + y2) / 2)
    return mx, my


def slope(p1, p2):  # calculates slope of two points
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1

    if dx == 0:
        return 0
    else:
        return dy / dx


if NETWORK_TABLES_ON:
    NetworkTables.initialize(server='10.58.11.2')
    NetworkTables.addConnectionListener(connection_listener, immediateNotify=True)

    with cond:
        print("Waiting")
        if not notified[0]:
            cond.wait()

    print("Connected!")


while True:
    retval, img = cap.read()

    crop_img = img[HEIGHT - CROPPED_HEIGHT:HEIGHT, 0:WIDTH]  # cropping image

    pipe.process(crop_img)  # processes image via GRIP pipeline
    img = cv.cvtColor(pipe.cv_erode_output, cv.COLOR_GRAY2BGR)  # converts to color
    contours = pipe.find_contours_output  # contour data

    width = img.shape[1]
    height = img.shape[0]
    mid_x = int(width / 2)

    filtered_contours = []
    targets = []
    goals = []

    for c in contours:  # filters and simplifies contours
        area = cv.contourArea(c)

        if area > CONTOUR_AREA_THRESHOLD:  # filter out contours by size
            perimeter = cv.arcLength(c, True)  # calculates corners of targets
            approx = cv.approxPolyDP(c, 0.04 * perimeter, True)

            filtered_contours.append(approx)

    for contour in filtered_contours:  # determines target data
        fixed_contour = []
        for c in contour:
            fixed_contour.append(c[0])

        # sorts points by increasing y value
        s = sorted(fixed_contour, key=lambda k: [k[1], k[0]])

        if len(contour) >= 4:  # at least 4 points detecting in contour
            # detecting target side
            upper_midpoint = midpoint(s[0], s[1])
            lower_midpoint = midpoint(s[2], s[3])
            m = slope(upper_midpoint, lower_midpoint)
            side = ""

            if m > 0:
                side = "right"
            else:
                side = "left"

            # calculating center point
            center = midpoint(upper_midpoint, lower_midpoint)

            # calculating area
            area = cv.contourArea(contour)  # fyi this is less accurate after contours filtered to 4 points

            cv.circle(img, center, 1, RED)

            # data saved per contour
            data = {
                'area': area,
                'side': side,
                'center': center,
                'contour': contour
            }

            targets.append(data)

    x_sorted_targets = sorted(targets, key=lambda k: [k['center']])
    left_target = None

    for t in x_sorted_targets:  # merges individual targets into goal units
        if t['side'] == "left":
            if left_target is None:
                left_target = t

        elif t['side'] == "right":
            if left_target is not None:
                merged_target = target_merger(left_target, t)  # merging target
                goals.append(merged_target)
                left_target = None

    final_target = None
    close_dist = width

    for g in goals:  # selects the final target based on proximity to camera center line
        cx = g['center'][0]
        dist = abs(MID_X - cx)

        if dist < close_dist:
            close_dist = dist
            final_target = g

        if DEBUG:
            cv.circle(img, g['center'], 4, RED)
            rect = cv.boundingRect(g['contour'])
            x, y, w, h = rect
            cv.rectangle(img, (x, y), (x + w, y + h), RED)

    sd = NetworkTables.getTable('SmartDashboard')

    if final_target:
        if DEBUG:
            cv.circle(img, final_target['center'], 4, GREEN)
            rect = cv.boundingRect(final_target['contour'])
            x, y, w, h = rect
            cv.rectangle(img, (x, y), (x + w, y + h), GREEN)
        update_list(final_target['area_ratio'])
        print(final_target['side'], final_target['aimed'])


    if NETWORK_TABLES_ON:
        if final_target:
            sd.putNumber("center_x", final_target['center'][0])
            sd.putNumber("center_y", final_target['center'][1])
            sd.putNumber("left_area", final_target['left_area'])
            sd.putNumber("right_area", final_target['right_area'])
        else:
            sd.putNumber("center_x", 0)
            sd.putNumber("center_y", 0)
            sd.putNumber("left_area", 0)
            sd.putNumber("right_area", 0)


    if DEBUG:
        cv.line(img, (mid_x, height), (mid_x, 0), RED)
        cv.imshow("Window", img)
        cv.waitKey(25)
