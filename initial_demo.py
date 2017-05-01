import cv2
import sys
import time

if __name__ == '__main__':

    # Set up tracker.
    # Instead of MIL, you can also use
    # BOOSTING, KCF, TLD, MEDIANFLOW or GOTURN (GOTURN doesn't work)

    tracker = cv2.Tracker_create("BOOSTING")

    # Read video
    video = cv2.VideoCapture("test_video_trimmed.mp4")

    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open video")
        sys.exit()

    # Read first frame.
    ok, frame = video.read()
    if not ok:
        print('Cannot read video file')
        sys.exit()

    # Define an initial bounding box
    bbox = (50, 250, 1150, 100)

    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)

    counter = 0
    switch = True

    cv2.imshow("Tracking", frame)
    time.sleep(10)

    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break

        # Update tracker
        ok, bbox = tracker.update(frame)

        # Draw bounding box
        if ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (0, 0, 255))

            y = int(bbox[1])
            if y > 550 and switch:
                switch = False
            elif y < 550 and not switch:
                switch = True
                counter += 1

        cv2.putText(frame, "Rep Count: {}".format(counter), (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 255), 2)

        # Display result
        cv2.imshow("Tracking", frame)

        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27: break

    time.sleep(1)
