
import time
import datetime
import cv2
import numpy as np
import math
import sys


def wait_for_card_swipe():
    #stripe = input('Swipe the card')
    #return stripe[8:17]
    return "200334767"


def logout_pressed():
    return False


# Pressure Sensors
def read_raw_from_pressure_sensor():
    return 0


def weight_is_zero():
    w = read_raw_from_pressure_sensor()
    # return w < 4
    foo = input("y if weight is zero, n if not")
    return foo == "y"


def weight_is_not_zero():
    w = read_raw_from_pressure_sensor()
    # return w >= 4
    return True


# OpenCV
def init_cv():
    tracker = cv2.Tracker_create("BOOSTING")
    video = cv2.VideoCapture("tilted.mp4")

    if not video.isOpened():
        raise RuntimeError("Could not open video")

    # Read first frame.
    ok, frame = video.read()
    if not ok:
        raise RuntimeError('Cannot read video')

    # Define an initial bounding box
    bbox = (50, 250, 1150, 100)

    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)
    if not ok:
        raise RuntimeError('Error initializing tracker')

    return video, tracker


def track_bar(track, frame, switch):
    increment = False
    # Update tracker
    ok, bbox = track.update(frame)
    if ok:

        y = int(bbox[1])
        if y > 550 and switch:
            switch = False
        elif y < 550 and not switch:
            switch = True
            increment = True
    return switch, increment


def find_marker(img):
    # Returns the contour corresponding to maximum area.
    edged = cv2.Canny(img, 35, 125)
    (_, cnts, _) = cv2.findContours(edged, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnts, key=cv2.contourArea)

    return cv2.minAreaRect(c), c


def bar_is_tilted(frame):
    # BGR color bounds for colors on the bar.
    lower1 = [51, 16, 4]
    upper1 = [223, 25, 10]
    # ^ Blue
    lower2 = [0, 0, 153]
    upper2 = [70, 80, 255]
    # ^ Red

    tilted = False
    # ^ Initializations for data being saved

    # process the image
    # create NumPy arrays from the boundaries
    lower1 = np.array(lower1, dtype="uint8")
    upper1 = np.array(upper1, dtype="uint8")

    # find the colors within the specified boundaries and apply
    # the mask
    mask1 = cv2.inRange(frame, lower1, upper1)
    frame1 = cv2.bitwise_and(frame, frame, mask=mask1)

    # create NumPy arrays from the boundaries
    lower2 = np.array(lower2, dtype="uint8")
    upper2 = np.array(upper2, dtype="uint8")

    # find the colors within the specified boundaries and apply
    # the mask
    mask2 = cv2.inRange(frame, lower2, upper2)
    frame2 = cv2.bitwise_and(frame, frame, mask=mask2)

    kernel2 = np.ones((10, 20), np.uint8)

    frame1 = cv2.dilate(frame1, kernel2, iterations=2)
    frame2 = cv2.dilate(frame2, kernel2, iterations=1)

    # Making a box around each color
    marker1, maxcont1 = find_marker(frame1)
    marker2, maxcont2 = find_marker(frame2)

    # draw a bounding box around the image and display it for Blue
    box1 = np.int0(cv2.boxPoints(marker1))
    cv2.drawContours(frame1, [box1], -1, (0, 255, 0), 2)
    # Find center point for Blue
    bluecenter = (((box1[0] + box1[1]) / 2) + ((box1[2] + box1[3]) / 2)) / 2

    # Commands to show the video for Blue
    # cv2.circle(frame1, (bluecenter[0], bluecenter[1]), 7, (255, 255, 255), -1)
    # cv2.putText(frame1, "center", (bluecenter[0] - 20, bluecenter[1] - 20),
    # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # cv2.imshow("Blue", frame1)

    # draw a bounding box around the image and display it for Red

    box2 = np.int0(cv2.boxPoints(marker2))
    cv2.drawContours(frame2, [box2], -1, (0, 255, 0), 2)

    # Computinng the center
    redcenter = (((box2[0] + box2[1]) / 2) + ((box2[2] + box2[3]) / 2)) / 2

    diff = abs(bluecenter[1] - redcenter[1])

    # Finding the angle of difference.
    ang = math.degrees(math.atan(float(redcenter[1] - bluecenter[1]) / (redcenter[0] - bluecenter[0])))

    # This is to remove potential fluctuations in the contour detection
    if diff > 20:
        if diff < 250:
            tilted = True

    return tilted


def perform_set(set_number, weight):
    # init mocap
    video, tracker = init_cv()
    rep = 0
    tilted = False
    flared = False

    bar_track_switch = True

    while weight_is_not_zero():
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break

        # track bar
        bar_track_switch, bar_track_increment = track_bar(tracker, frame, bar_track_switch)
        if bar_track_increment:
            rep += 1

        # check tilt
        # if bar_is_tilted(frame):
        #     tilted = True
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()
    # weight is on sensor, set is complete
    data = {"reps": rep, "weight": weight, "tilted": tilted}
    print(data)

while True:
    weight = 45
    set_number = 1
    perform_set(set_number, weight)

    # Wait until logout pressed or 120s elapsed
    current = datetime.datetime.now()
    while not logout_pressed() or (datetime.datetime.now() - current).total_seconds() < 60:
        if weight_is_zero():
            set_number += 1
            break
    else:
        end_user = True


