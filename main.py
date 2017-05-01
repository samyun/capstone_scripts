import RPi.GPIO as GPIO
import time
from pyrebase import pyrebase
import datetime
import cv2
import numpy as np
import math
import serial
import sys

# Firebase Configuration
FB_CONFIG = {
    "apiKey": "apiKey""AIzaSyA2LCqqgPmU2nc_ozzvIg0s_HJX43Hdtxs",
    "authDomain": "webapp-5d9db.firebaseapp.com",
    "databaseURL": "https://webapp-5d9db.firebaseio.com",
    "storageBucket": "webapp-5d9db.appspot.com",
    "serviceAccount": "/home/pi/Desktop/capstone/webapp-5d9db-firebase-adminsdk-yousj-66a1e4421c.json"
}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

B1 = 18
GPIO.setup(B1, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def wait_for_card_swipe():
    stripe = input('Swipe the card')
    return stripe[8:17]


def logout_pressed():
    return GPIO.input(B1) == GPIO.LOW


# Firebase
def init_firebase():
    firebase = pyrebase.initialize_app(FB_CONFIG)
    # Firebase Database Intialization
    db = firebase.database()
    return db


def get_fb_user_key(fb_db):
    users = fb_db.child("registeredUsers").get()
    for user in users.each():
        buckid = user.val().get("buckid")
        if buckid == user_buckid:
            userkey = user.key()
            return userkey
    raise RuntimeError("Couldn't find user in Firebase DB")


def write_set_to_firebase(fb_db, userkey, rep_data, set_num):
    date_string = "Date: " + (datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    set_string = "set number: {}".format(set_num)
    fb_db.child("registeredUsers").child(userkey).child(date_string).child(set_string).set(rep_data)


# Pressure Sensors
def flush_lines(serial_port):
    serial_port.flushOutput()
    serial_port.flushInput()


def init_sensors(baud=9600):
    # Port may vary, so look for it:
    baseports = ['/dev/ttyUSB', '/dev/ttyACM']
    global serial_port
    serial_port = None
    for baseport in baseports:
        if serial_port : break
        for i in range(0, 8):
            try:
                port = baseport + str(i)
                serial_port = serial.Serial(port, baud, timeout=1)
                print(("Opened", port))
                break
            except :
                serial_port = None
                pass

    if not serial_port:
        raise RuntimeError("Couldn't open a serial port")

    serial_port.write_timeout = 1
    serial_port.timeout = 1

    # wait for initial arduino message
    flush_lines(serial_port)

    # tare sensors
    input("Ready to tare? Enter a key with no load")
    print('Tareing...')
    for x in range(0, 9):
        serial_port.readline().strip()
        time.sleep(.1)
    # send config command
    serial_port.write(bytes("x", 'UTF-8'))
    for x in range(0, 20):
        serial_port.readline().strip()
        time.sleep(.1)
    # wait for messages
    flush_lines(serial_port)
    # send tare command
    serial_port.write(bytes("1", 'UTF-8'))
    for x in range(0, 20):
        serial_port.readline().strip()
        time.sleep(.1)
    # wait for messages
    flush_lines(serial_port)
    # exit config
    serial_port.write(bytes("x", 'UTF-8'))
    for x in range(0, 3):
        serial_port.readline().strip()
        time.sleep(.1)
    # wait for messages
    flush_lines(serial_port)

    print("tared")


def read_raw_from_pressure_sensor():
    # Send characters:
    serial_port.write(bytes("0", 'UTF-8'))

    # check for serial output:
    output = serial_port.readline().strip()

    return int(output.decode("UTF-8")[:-5])


def weight_is_zero():
    w = read_raw_from_pressure_sensor()
    return w < 4


def weight_is_not_zero():
    w = read_raw_from_pressure_sensor()
    return w >= 4


# OpenCV
def init_cv():
    tracker = cv2.Tracker_create("BOOSTING")
    video = cv2.VideoCapture()

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
        if y > 400 and switch:
            switch = False
        elif y < 400 and not switch:
            switch = True
            increment = True
    return switch, increment, bbox


def find_marker(img):
    # Returns the contour corresponding to maximum area.
    edged = cv2.Canny(img, 35, 125)
    (_, cnts, _) = cv2.findContours(edged, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnts, key=cv2.contourArea)

    return cv2.minAreaRect(c), c


def track_tilt(frame):
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
    # Find center point for Blue
    bluecenter = (((box1[0] + box1[1]) / 2) + ((box1[2] + box1[3]) / 2)) / 2

    # draw a bounding box around the image and display it for Red

    box2 = np.int0(cv2.boxPoints(marker2))

    # Computing the center
    redcenter = (((box2[0] + box2[1]) / 2) + ((box2[2] + box2[3]) / 2)) / 2

    diff = abs(bluecenter[1] - redcenter[1])

    # Finding the angle of difference.
    ang = math.degrees(math.atan(float(redcenter[1] - bluecenter[1]) / (redcenter[0] - bluecenter[0])))

    # This is to remove potential fluctuations in the contour detection
    if diff > 20:
        if diff < 250:
            tilted = True
    return tilted, ang, bluecenter, redcenter


def display_frame(frame, barbox, count, bluecenter, redcenter, ang):
    # Place rep stuff
    p1 = (int(barbox[0]), int(barbox[1]))
    p2 = (int(barbox[0] + barbox[2]), int(barbox[1] + barbox[3]))
    cv2.rectangle(frame, p1, p2, (0, 255, 0))

    y = int(barbox[1])
    if y > 400:
        pos = "BELOW"
    else:
        pos = "ABOVE"

    cv2.putText(frame, "Position: {}".format(pos), (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 0), 2)

    cv2.putText(frame, "Count: {}".format(count), (10, 160),
                cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 0), 2)

    # Place tilt stuff
    cv2.circle(frame, (int(bluecenter[0]), int(bluecenter[1])), 7, (255, 255, 255), -1)
    cv2.circle(frame, (int(redcenter[0]), int(redcenter[1])), 7, (255, 255, 255), -1)
    cv2.putText(frame, "Angle: {}".format(ang), (700, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 0), 2)

    cv2.imshow("Image", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        pass


def perform_set(fb_db, set_number, weight):
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
        try:
            bar_track_switch, bar_track_increment, box = track_bar(tracker, frame, bar_track_switch)
            if bar_track_increment:
                rep += 1
        except ValueError:
            continue

        # check tilt
        try:
            frame_tilted, angle, b_center, r_center = track_tilt(frame)
            if frame_tilted:
                tilted = True
        except ValueError:
            continue

        display_frame(frame, box, rep, b_center, r_center, angle)

    video.release()
    cv2.destroyAllWindows()
    # weight is on sensor, set is complete
    data = {"reps": rep, "weight": weight, "tilted": tilted}
    write_set_to_firebase(fb_db, user_key, data, set_number)


# Start program

# initialize firebase
db = init_firebase()
init_sensors()

while True:
    try:
        # Wait for card swipe
        user_buckid = wait_for_card_swipe()

        # get user key
        user_key = get_fb_user_key(db)

        # while weight is zero, wait
        while weight_is_zero():
            time.sleep(0.05)

        # when weight > 0, get weight of lift
        weight = read_raw_from_pressure_sensor()

        # while weight is on sensor, wait
        while weight_is_not_zero():
            time.sleep(0.05)

        # when weight is off sensor, start set
        set_number = 0
        end_user = False

        # loop until end_user is True
        while end_user is False:
            perform_set(db, set_number, weight)

            # Wait until logout pressed or 120s elapsed
            current = datetime.datetime.now()
            while not logout_pressed() or (datetime.datetime.now() - current).total_seconds() < 120:
                if weight_is_zero():
                    set_number += 1
                    break
            else:
                end_user = True
    except RuntimeError as e:
        print(e)
        continue
    except ValueError as e:
        print(e)
        continue


