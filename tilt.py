# import the necessary packages
import numpy as np
import cv2
import math

# getting first frame
vidcap = cv2.VideoCapture('tilt.mp4')
success, image = vidcap.read()

cv2.imwrite("frame0.jpg", image)  # save frame as JPEG file
image = cv2.imread('frame0.jpg', 1)


def find_marker(image):
    # Returns the contour correspnding to maximum area.
    edged = cv2.Canny(image, 35, 125)
    (cnts, _) = cv2.findContours(edged, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnts, key=cv2.contourArea)

    return cv2.minAreaRect(c), c


marker, maxcont = find_marker(image)

# BGR color bounds for colors on the bar.
lower1 = [51, 16, 4]
upper1 = [223, 25, 10]
# ^ Blue
lower2 = [0, 0, 153]
upper2 = [70, 80, 255]
# ^ Red

bluecoord = []
redcoord = []
difference = []
angle = []
# ^ Initializations for data being saved

# loop over the images
while True:

    (grabbed, frame) = vidcap.read()

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
    box1 = np.int0(cv2.cv.BoxPoints(marker1))
    cv2.drawContours(frame1, [box1], -1, (0, 255, 0), 2)
    # Find center point for Blue
    bluecenter = (((box1[0] + box1[1]) / 2) + ((box1[2] + box1[3]) / 2)) / 2
    bluecoord.append(bluecenter)

    # Commands to show the video for Blue
    ##	cv2.circle(frame1, (bluecenter[0], bluecenter[1]), 7, (255, 255, 255), -1)
    ##	cv2.putText(frame1, "center", (bluecenter[0] - 20, bluecenter[1] - 20),
    ##	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # cv2.imshow("Blue", frame1)

    # draw a bounding box around the image and display it for Red

    box2 = np.int0(cv2.cv.BoxPoints(marker2))
    cv2.drawContours(frame2, [box2], -1, (0, 255, 0), 2)

    # Computinng the center
    redcenter = (((box2[0] + box2[1]) / 2) + ((box2[2] + box2[3]) / 2)) / 2
    redcoord.append(redcenter)

    diff = abs(bluecenter[1] - redcenter[1])
    difference.append(diff)

    # Finding the angle of difference.
    ang = math.degrees(math.atan(float(redcenter[1] - bluecenter[1]) / (redcenter[0] - bluecenter[0])))
    angle.append(ang)

    # This is to remove potential fluctuations in the contour detection
    if diff > 20:
        if diff < 250:
            print('Bar is tilted by angle {}'.format(ang))


            # Commands to show the video for Red
            ##	cv2.circle(frame2, (redcenter[0], redcenter[1]), 7, (255, 255, 255), -1)
            ##	cv2.putText(frame2, "center", (redcenter[0] - 20, redcenter[1] - 20),
            ##	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            ##	cv2.imshow("Red", frame2)

    # Human readable data for saving all the info.
    np.savetxt('bluecoord.csv', bluecoord)
    np.savetxt('redcoord.csv', redcoord)
    np.savetxt('difference.csv', difference)
    np.savetxt('angle.csv', angle)
    avgangl = mean(angle)
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break



