import cv2
import numpy as np
from pymavlink import mavutil
################################## OPTIONS #########################################

### ACESS METHOD ###
# 0 -> acess a local saved image (needs to inform the image path)
# 1 -> physical local camera acess (remember to set the correct vidinput value below according to your purposes)
# 2 -> remote images acess(web), useful for IP cameras, needs to inform the image url
imageAcessMethod = 1

# 0 - generally first camera in the system. Use 1 with virtual camera from OBS for testing purposes
vidinput = 0
imagePath = r""

### AREA FILTER (IN px) ###
aFilter = 100  # every area smaller is disregarded

### FRAME/IMAGE PASSING ###
# 0 -> manual
# 1 or any other value (passing delay in ms) -> auto
framePassing = 500

### SHOW PARSED VIDEO WITH ALL THE FILTERS###
showparsedvideo = 0


################################## MAVLINK & CONV VARS #########################################

string_connection = '/dev/serial0, 57600'
print(f"Trying to connect to {string_connection}...")
connection = mavutil.mavlink_connection(string_connection)

if connection:
    print("Sucess")
else:
    print("Fail")

# wait for the first hearthbeat
try:
    print("Waiting for heartbeat...")
    connection.wait_heartbeat()
    print("Hearthbeat recivied from (system %u component %u)" %
          (connection.target_system, connection.target_component))
except Exception as e:
    print("Error when waiting for heartbeat: ", e)

### PIXELS TO CM^2 CONVERSION VARS  ###
# base values needs to be discovered by testing, once you have them for your specific camera
# the code can convert any area at any heigth for this camera
baseH = 28  # base distance between the camera and the figure(heigth) in cm
actualH = 0  # distance received by the sensord (heigth) in cm
x = actualH/baseH
baseArea = 50000  # base figure area in pixels
baseAreaCm = 100  # base figure area in cm^2 (real area)

# converts the base area(already know for a specific height)...
# to the estimated size it should be in the actual height (px)


def actualBaseArea():
    actualH = connection.recv_match(type='RANGEFINDER', blocking=True)
    x = (actualH * 100)/baseH
    return baseArea/(x*x)


def nothing(n):
    pass


# pre set canny values (set for local and remote images)
cannyMinVal = 50
cannyMaxVal = 100

# only works for camera acess images
cv2.namedWindow('Canny min and max')
cv2.createTrackbar('Min', 'Canny min and max', cannyMinVal, 500, nothing)
cv2.createTrackbar('Max', 'Canny min and max', cannyMaxVal, 500, nothing)

cam = cv2.VideoCapture(vidinput)

print(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(cam.get(cv2.CAP_PROP_FRAME_WIDTH))

if imageAcessMethod == 0:  # local image
    img = cv2.imread(imagePath)
    grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(grayImg, (5, 5), 0)
    canny = cv2.Canny(blur, cannyMinVal, cannyMaxVal, L2gradient=True)

    contours, hierarchy = cv2.findContours(
        canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
    smallImg = cv2.resize(img, (600, 800))  # only for img showing
    cv2.imshow('Detected contours', smallImg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # FILTERS AND CALCULATES THE AREA FOR EACH CONTOUR
    for i, contour in enumerate(contours):
        if cv2.contourArea(contour) < aFilter:
            continue
        # calcula a area em pixels a partir do contorno (precisao muito alta)
        cv2.drawContours(img, contours, i, (0, 0, 255), 3)
        smallImg = cv2.resize(img, (600, 800))  # only for img showing
        cv2.imshow(f"Contour {i}", smallImg)

        area = cv2.contourArea(contour)  # actual contour area (px)

        areacm = (area*baseAreaCm)/actualBaseArea()  # converts to cm^2
        print(f'Contour {i}: {area}px  |   {areacm}cm^2')

        cv2.waitKey(framePassing)
        print('-'*30)
elif imageAcessMethod == 1:  # camera frames
    while True:
        # returns image from the camera
        sucess, frame = cam.read()
        if not sucess:
            print("Error getting the frame, did you select de correct vidinput?")
            break

        cannyMinVal = cv2.getTrackbarPos('Min', 'Canny min and max')
        cannyMaxVal = cv2.getTrackbarPos('Max', 'Canny min and max')

        # Gaussian Blur - reduce image noise. There are other functions to test for our purposes
        img = cv2.GaussianBlur(frame, (5, 5), 0)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if showparsedvideo == 1:
            # Output of the image to be analized later
            cv2.imshow('CameraView', img)

        # Canny - binarize image contours by their gradient
        raw_contours = cv2.Canny(
            img, cannyMinVal, cannyMaxVal, L2gradient=True)
        # minVal: any contour with gradient less than this value is 0
        # maxVal: any contour with gradient above than this value is 255
        # any value between this two that is connected to a value above max is 255, else is 0
        # L2gradient: when True uses an algoritm more precise
        # needs to be calibrated with different images and gradients, maybe even dinamically by the code itself

        # Find Contours in a binarized image, and returns its points
        contours, hierarchy = cv2.findContours(
            raw_contours, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.RETR_EXTERNAL - only take the external contours,
        # cv2.CHAIN_APPROX_SIMPLE - form the contours only with the essential points
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

        # cv2.imshow('Found contours', frame)

        selContours = []

        for i, contour in enumerate(contours):
            if cv2.contourArea(contour) < aFilter:
                continue
            selContours.append(contour)
            area = cv2.contourArea(contour)  # actual contour area (px)

            areacm = (area*baseAreaCm)/actualBaseArea()  # converts to cm^2
            print(f'Contour {i}: {area}px  |   {areacm}cm^2')
        cv2.drawContours(frame, selContours, -1, (0, 0, 255), 3)
        print('-'*30)
        # press 'esc' to exit, set to 1 for automatic frame passing
        if cv2.waitKey(framePassing) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
else:
    print("[ERROR], invalid imageAcessMethod value")
cv2.destroyAllWindows()
