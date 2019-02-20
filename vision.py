#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import os
import threading
import time
from networktables import NetworkTables

DEBUG = False
NETWORK_TABLES_ON = True
STREAM_VISION = True
CAMERA_PORT = 0

if DEBUG:
    import pipeline_laptop as pipeline
else:
    import pipeline
    from cscore import CameraServer, CvSource, VideoMode

    inst = CameraServer.getInstance()
    camera = CvSource("CvCam", VideoMode.PixelFormat.kBGR, 320, 160, 15)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)

os.system("v4l2-ctl -d /dev/video{} -c exposure_auto=1".format(CAMERA_PORT))
os.system("v4l2-ctl -d /dev/video{} -c exposure_absolute=0".format(CAMERA_PORT))
os.system("v4l2-ctl -d /dev/video{} -c brightness=30".format(CAMERA_PORT))
os.system("v4l2-ctl -d /dev/video{} -c contrast=10".format(CAMERA_PORT))

cap = cv.VideoCapture(CAMERA_PORT)
WIDTH = int(cap.get(3))
HEIGHT = int(cap.get(4))
MID_X = int(WIDTH / 2)

CROPPED_HEIGHT = 320
HFOV = 61

CONTOUR_AREA_THRESHOLD = 0  # min area to be recognized as a target
GOAL_PERCENT_DISTANCE_THRESHOLD = 0.6  # percent of screen in middle where area trumps distance
GOAL_DISTANCE_THRESHOLD = int(GOAL_PERCENT_DISTANCE_THRESHOLD / 2 * WIDTH)

RED = (0, 0, 255)
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)
BONDS_COLOR = (0, 179, 255)
BLACK = (0, 0, 0)

pipe = pipeline.GripPipeline()
cond = threading.Condition()
notified = [False]


def connection_listener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()


def merger(left_target, right_target):
    area_left = left_target['area']
    area_right = right_target['area']
    total_area = area_left + area_right
    center_left = left_target['center']
    center_right = right_target['center']
    center = midpoint(center_left, center_right)

    difference_area = area_left - area_right
    offset = MID_X - center[0]

    angle = offset / MID_X * HFOV / 2

    data = {
        'left_area': area_left,
        'right_area': area_right,
        'total_area': total_area,
        'difference_area': difference_area,
        'offset': offset,
        'angle': angle,
        'center': center,
        'contour': np.concatenate((left_target['contour'], right_target['contour']))

    }

    return data


def midpoint(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    mx = int((x1 + x2) / 2)
    my = int((y1 + y2) / 2)
    return mx, my


if NETWORK_TABLES_ON:
    NetworkTables.initialize(server='10.58.11.2')
    NetworkTables.addConnectionListener(connection_listener, immediateNotify=True)

    with cond:
        print("Waiting")
        if not notified[0]:
            cond.wait()

    print("Connected!")


while True:
    time_init = time.time()

    ret_val, raw_img = cap.read()
    cropped_img = raw_img[HEIGHT - CROPPED_HEIGHT:HEIGHT, 0:WIDTH]  # cropping image
    # cropped_img = raw_img

    pipe.process(cropped_img)
    img = cv.cvtColor(pipe.cv_erode_output, cv.COLOR_GRAY2BGR)  # converts to color
    contours = pipe.find_contours_output  # contour data

    height = img.shape[0]
    width = img.shape[1]

    targets = []
    goals = []
    for c in contours:  # filters targets and packages them
        rect = cv.minAreaRect(c)  # calculating rotated rectangle
        angle = rect[2]  # angle of rect
        box = cv.boxPoints(rect)  # converts x y w h to 4 pts
        box = np.int0(box)  # converts all to int

        area = cv.contourArea(box)

        if area > CONTOUR_AREA_THRESHOLD:
            m = cv.moments(box)  # calculating center point
            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])
            center = cx, cy

            if DEBUG:
                cv.circle(img, (cx, cy), 1, BLUE, 2)
                cv.drawContours(img, [box], 0, RED, 2)

            if angle > -45:  # calculating side
                target_side = "right"
            else:
                target_side = "left"

            data = {
                'area': area,
                'side': target_side,
                'center': center,
                'contour': box
            }

            targets.append(data)

    x_sorted_targets = sorted(targets, key=lambda k: [k['center']])
    previous_target = None

    for target in x_sorted_targets:  # groups targets into goals
        side = target['side']

        if side == 'left':
            previous_target = target

        elif side == "right":
            if previous_target and previous_target['side'] == 'left':
                merged_target = merger(previous_target, target)
                goals.append(merged_target)
                previous_target = None
            else:
                previous_target = target

    thresholded_goals = []

    for g in goals:  # selects goal to track
        cx = g['center'][0]
        dist = abs(MID_X - cx)

        if dist < GOAL_DISTANCE_THRESHOLD:
            thresholded_goals.append(g)

        if DEBUG or STREAM_VISION:
            cv.circle(img, g['center'], 4, RED)
            rect = cv.minAreaRect(g['contour'])
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(img, [box], 0, RED, 1)

    max_area = 0
    min_dist = WIDTH
    selected_goal = None

    if thresholded_goals:
        for g in thresholded_goals:
            area = g['total_area']

            if area > max_area:

                max_area = area
                selected_goal = g
            # elif  area > max_area - 50:
            #

    else:
        for g in goals:
            cx = g['center'][0]
            dist = abs(MID_X - cx)

            if dist < min_dist:
                min_dist = dist
                selected_goal = g

    if selected_goal:
        if DEBUG or STREAM_VISION:
            cv.circle(img, selected_goal['center'], 4, GREEN)
            rect = cv.minAreaRect(selected_goal['contour'])
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(img, [box], 0, GREEN, 1)
        # print(selected_goal['center'])

    if DEBUG or STREAM_VISION:
        cv.line(img, (MID_X, 0), (MID_X, height), RED)
        cv.line(img, (MID_X - GOAL_DISTANCE_THRESHOLD, 0), (MID_X - GOAL_DISTANCE_THRESHOLD, height), YELLOW)
        cv.line(img, (MID_X + GOAL_DISTANCE_THRESHOLD, 0), (MID_X + GOAL_DISTANCE_THRESHOLD, height), YELLOW)

    if DEBUG:
        cv.imshow("Window", img)
        cv.waitKey(25)

    if NETWORK_TABLES_ON:
        sd = NetworkTables.getTable('SmartDashboard')
        if selected_goal:
            for i in selected_goal:
                sd.putNumber('left_area', selected_goal['left_area'])
                sd.putNumber('right_area', selected_goal['right_area'])
                sd.putNumber('total_area', selected_goal['total_area'])
                sd.putNumber('difference_area', selected_goal['difference_area'])
                sd.putNumber('offset', selected_goal['offset'])
                sd.putNumber('angle', selected_goal['angle'])
                sd.putNumber('center_x', selected_goal['center'][0])
                sd.putNumber('center_y', selected_goal['center'][1])
                sd.putNumber('time_stamp', time.time())

    if STREAM_VISION:
        streamed_img = cv.resize(img, None, fx=0.5, fy=0.5, interpolation=cv.INTER_CUBIC)

        camera.putFrame(streamed_img)

    elapsed = time.time() - time_init  # time one loop takes
    print(selected_goal['angle'])
