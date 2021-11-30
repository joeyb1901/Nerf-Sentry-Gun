import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep

# Set up GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Buttons
fireButton = 12
blueButton = 6
redButton = 5

GPIO.setup(fireButton,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(blueButton,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(redButton,GPIO.IN, pull_up_down=GPIO.PUD_UP)

# LEDs
fireLED = 26
blueLED = 20
redLED = 21

GPIO.setup(fireLED,GPIO.OUT)
GPIO.setup(blueLED,GPIO.OUT)
GPIO.setup(redLED,GPIO.OUT)

# Firing motors
revPin = 2
GPIO.setup(revPin, GPIO.OUT)

# Firing servo
servo = 10
GPIO.setup(servo, GPIO.OUT)
pulse = GPIO.PWM(servo, 50)
pulse.start(0)
  
pulse.ChangeDutyCycle(4)
sleep(0.5)
pulse.start(0)


# Initialize outputs as off
GPIO.output(fireLED,False)
GPIO.output(blueLED,False)
GPIO.output(redLED,False)
GPIO.output(revPin,False)

# Initialize values
revMotors = 0

# Set up NEMA 17 (X component)
dirX = 14
pulX = 15
GPIO.setup(dirX, GPIO.OUT)
GPIO.setup(pulX, GPIO.OUT)

# Set up NEMA 23 (Y component)
dirY = 17
pulY = 27
GPIO.setup(dirY, GPIO.OUT)
GPIO.setup(pulY, GPIO.OUT)


pulseY = GPIO.PWM(pulY, 10)
pulseY.start(0)

pulseX = GPIO.PWM(pulX, 10)
pulseX.start(0)

NEMAdc = 30

def empty():
    pass


def getObjects(frame, HSV, frameCenter, resetTargets, ct, minArea = 1000):
    rects = []
    outputImg, mask = maskFrame(frame, HSV)
    cv2.drawMarker(outputImg, frameCenter, (255, 0, 0), thickness=2)  # mark center of frame (blue)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > minArea:  # only draw if area is large enough
            print(str(area))
            peri = cv2.arcLength(cnt, True)  # contour is closed
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)  # approximation of points in shape
            if len(approx) == 8:  # circles are generally 8 points
                x, y, w, h = cv2.boundingRect(approx)  # create temporary values
                (startX, startY, endX, endY) = x, y, x + w, y + h
                box = (startX, startY, endX, endY)
                rects.append(box)
                cv2.rectangle(outputImg, (startX, startY), (endX, endY),
                              (0, 255, 0), 2)

    objects = ct.update(rects, resetTargets)

    return objects, outputImg


def teamSelect(hsv, blue, red, targetColor, nextTarget, shoot, maxTarget):
    resetTargets = False
    if blue:
        hsv = [[80, 105, 0],  # minimum values for H,S,V
               [130, 197, 255]]  # maximum values for H,S,V
        targetColor = 'Blue'
        resetTargets = True
        nextTarget = 0
        maxTarget = 0
        shoot = False
        GPIO.output(blueLED,True)
        GPIO.output(redLED,False)
        print('Blue Selected')

    elif red:
        hsv = [[146, 81, 79],
               [213, 253, 199]]
        targetColor = 'Red'
        resetTargets = True
        nextTarget = 0
        maxTarget = 0
        shoot = False
        GPIO.output(blueLED,False)
        GPIO.output(redLED,True)
        print('Red Selected')

    return hsv, targetColor, resetTargets, nextTarget, shoot, maxTarget


def maskFrame(cap, hsv):
    _, img = cap.read()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    imgBlur = cv2.GaussianBlur(imgHsv, (7, 7), 1)

    lower = np.array([hsv[0][0], hsv[0][1], hsv[0][2]])
    upper = np.array([hsv[1][0], hsv[1][1], hsv[1][2]])

    imgMask = cv2.inRange(imgBlur, lower, upper)
    masked = cv2.bitwise_and(img, img, imgMask)
    result = masked.copy()

    return result, imgMask


def labelFrame(fc, targetColor, shoot):
    if targetColor == 'Blue':
        fontBGR = (255, 0, 0)
    elif targetColor == 'Red':
        fontBGR = (0, 0, 255)
    else:
        fontBGR = (0, 0, 0)
    cv2.putText(fc, 'Target Color: ' + targetColor, (20, 20), cv2.FONT_HERSHEY_COMPLEX,
                0.7, fontBGR, 2)

    if shoot:
        shootText = 'Yes'
        GPIO.output(fireLED,True)
    else:
        shootText = 'No'
        GPIO.output(fireLED,False)
    cv2.putText(fc, 'Ready to shoot: ' + shootText, (20, 40), cv2.FONT_HERSHEY_COMPLEX,
                0.7, (0, 0, 0), 2)


def aimingMechanism(frame, targetCenter, center, Next, setPos, fireCount, shooting, revTime = 75, accuracy = 20):
    if targetCenter[0] > center[0]:  # aim right
        cv2.drawMarker(frame, (center[0] + 20, center[1]), (0, 0, 0), thickness=2)
        setPos[0] = 'right'
    elif targetCenter[0] < center[0]:  # aim left
        cv2.drawMarker(frame, (center[0] - 20, center[1]), (0, 0, 0), thickness=2)
        setPos[0] = 'left'
    if targetCenter[1] > center[1]:  # aim down
        cv2.drawMarker(frame, (center[0], center[1] + 20), (0, 0, 0), thickness=2)
        setPos[1] = 'down'
    elif targetCenter[1] < center[1]:  # aim up
        cv2.drawMarker(frame, (center[0], center[1] - 20), (0, 0, 0), thickness=2)
        setPos[1] = 'up'

    if (center[0] - accuracy) <= targetCenter[0] & targetCenter[0] <= (center[0] + accuracy):
        setPos[0] = 'none'
    if (center[1] - accuracy) <= targetCenter[1] & targetCenter[1] <= (center[1] + accuracy):
        setPos[1] = 'none'
    
    if setPos == ['none', 'none']:
        GPIO.output(revPin,True)
        fireCount += 1
        if fireCount >= revTime:
            print("Firing")
            pulse.ChangeDutyCycle(9)
            sleep(0.5)
            pulse.ChangeDutyCycle(4)
            sleep(0.5)
            pulse.start(0)
            Next += 1
            fireCount = 0
            GPIO.output(revPin,False)
            shoot = False

    return Next, fireCount, shooting


def runSteppers(setPos, shoot):
    if shoot:
        # X movement
        if setPos[0] == 'left':
            pulseX.start(NEMAdc)
            GPIO.output(dirX,False)
            #sleep(0.5)
            #pulseX.start(0)
        elif setPos[0] == 'right':
            pulseX.start(NEMAdc)
            GPIO.output(dirX,True)
            #sleep(0.5)
            #pulseX.start(0)
        elif setPos[0] == 'none':
            pulseX.start(0)
        
        # Y movement
        if setPos[1] == 'down':
            pulseY.start(NEMAdc)
            GPIO.output(dirY,False)
            #sleep(0.5)
            #pulseY.start(0)
        elif setPos[1] == 'up':
            pulseY.start(NEMAdc)
            GPIO.output(dirY,True)
            #sleep(0.5)
            #pulseY.start(0)
        elif setPos[1] == 'none':
            pulseY.start(0)
    else:
        pulseX.start(0)
        pulseY.start(0)
    
    print(setPos)

def sweepState(fc, setPos, count, maxCount=200):
    cv2.putText(fc, str(setPos), (20, 60), cv2.FONT_HERSHEY_COMPLEX,
                0.7, (0, 0, 255), 2)
    count += 1
    if count <= maxCount/2:
        setPos[0] = 'left'
    elif maxCount/2 < count < maxCount:
        setPos[0] = 'right'
    elif count >= maxCount:
        count = 0

    return setPos, count

    
def buttonInterrupt(pressed, button):
    if not GPIO.input(button):
        if not pressed:
            pressed = True
    else:
        pressed = False
        
    return pressed