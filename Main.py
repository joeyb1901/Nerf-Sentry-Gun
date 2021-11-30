import cv2
from classes import CentroidTracker
import modules as mod
import RPi.GPIO as GPIO

ct = CentroidTracker()

# Initialize camera
frame = cv2.VideoCapture(0)
H = 480
W = 640
frame.set(3, W)
frame.set(4, H)
frameCenter = (int(W / 2), int(H / 2)+60)

# Set initial values
HSV = [[0, 0, 0],
       [0, 0, 0]]
shootTargets = False
nextTarget = 0
targetColor = 'None'
resetTargets = False
setPos = ['empty', 'empty']
maxTarget = 0
fireCount = 0
sweepState = False
count = 0
shooting = False

# Set up GPIO

fireButton = 12
blueButton = 6
redButton = 5

pressedShoot = False
pressedBlue = False
pressedRed = False

while True:

    objects, outputImg = mod.getObjects(frame, HSV, frameCenter, resetTargets, ct)

    if len(objects.items()) == 0 and shootTargets:
        sweepState = True
    else:
        sweepState = False

    if not shootTargets:
        setPos = ['none', 'none']

    if sweepState:
        setPos, count = mod.sweepState(outputImg, setPos, count)
    else:
        for (objectID, centroid) in objects.items():
            text = "ID {}".format(objectID)  # label each target
            cv2.putText(outputImg, text, (centroid[0] - 10, centroid[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(outputImg, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)
            if objectID > maxTarget:
                maxTarget = objectID
            if objectID is nextTarget:
                if shootTargets:
                    nextTarget, fireCount, shooting = mod.aimingMechanism(outputImg, centroid, frameCenter,
                                                                            nextTarget, setPos, fireCount, shooting)
            elif nextTarget > maxTarget:
                setPos, count = mod.sweepState(outputImg, setPos, count)
            elif nextTarget not in objects:  # if next target is an existing target ID
                nextTarget += 1

    print("Next target: " + str(nextTarget))
    mod.runSteppers(setPos, shootTargets)

    mod.labelFrame(outputImg, targetColor, shootTargets)  # label frame with target color and shooting status
    
    cv2.imshow("Frame", outputImg)  # output processed image
    
    # Detect button presses
    pressedShoot = mod.buttonInterrupt(pressedShoot, fireButton)
    pressedBlue = mod.buttonInterrupt(pressedBlue, blueButton)
    pressedRed = mod.buttonInterrupt(pressedRed, redButton)
    
    if pressedShoot and targetColor != 'None':
        shootTargets = True
        resetTargets = False
        nextTarget = 0
        maxTarget = 0
    
    if pressedBlue or pressedRed:
        HSV, targetColor, resetTargets, nextTarget, shootTargets, maxTarget = mod.teamSelect(HSV, pressedBlue, pressedRed,
                                                                                             targetColor, nextTarget,
                                                                                             shootTargets, maxTarget)
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

