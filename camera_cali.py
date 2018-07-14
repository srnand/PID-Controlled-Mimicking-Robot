import numpy as np
import cv2
import math
from time import sleep
import matplotlib.pyplot as plt
import serial
from struct import *

ser = serial.Serial('/dev/cu.usbmodem1411',9600)
ser.close()

cap = cv2.VideoCapture(0)
calibPoints = [(), (), (), ()]
M = np.zeros((3, 3))
projReady = False
startMimic = False
angleA = 0
angleB = 0

angleB_array=[]
error_array=[]

Kc = 0.5
Ki = 0.1
Kd = 0.3

global reset, lastError
reset = 0.0
lastError = 0.0

def PID(desired_angle, current_angle):
    global reset, lastError
    error = desired_angle - current_angle
    reset = reset + Ki * error
    out = Kc*error + reset + Kd*(error - lastError)
    lastError = error
    return out


def PI(desired_angle, current_angle):
    global reset
    error = desired_angle - current_angle
    reset = reset + Ki * error
    return Kc*error + reset


def P(desired_angle, current_angle):
    error = desired_angle - current_angle
    return Kc*error


def pointDist(a, b):
    dist = np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    return dist


def getImagePoint(r, a):
    p = [r * math.cos(math.radians(a)), radius * math.sin(math.radians(a))]
    return p


lower_red = np.array([0, 70, 50])
upper_red = np.array([10, 255, 255])
lower_red1 = np.array([170, 70, 50])
upper_red1 = np.array([180, 255, 255])
lower_green = np.array([45, 70, 50])
upper_green = np.array([75, 255, 255])

while True:
    ret, frame = cap.read()

    # Our operations on the frame come here
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask2 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask3 = cv2.inRange(hsv, lower_green, upper_green)
    mask = mask1 | mask2 | mask3
    output = cv2.bitwise_and(frame, frame, mask=mask)
    maskR = mask1 | mask2
    maskG = mask3
    outputR = cv2.bitwise_and(frame, frame, mask=maskR)
    outputG = cv2.bitwise_and(frame, frame, mask=maskG)
    # Display the resulting frame
    # print(output.shape)
    # cv2.cvtColor(output, )
    # out = np.array(output, np.uint8)
    pointsR = cv2.findNonZero(outputR[:, :, 2])
    pointsG = cv2.findNonZero(outputG[:, :, 2])
    if pointsR is not None and pointsG is not None:
        pointsR = pointsR.squeeze()
        pointsG = pointsG.squeeze()
        # print(pointsR.shape, pointsG.shape)
        pointR = np.array(np.mean(pointsR, axis=0), dtype=np.int)
        pointG = np.array(np.mean(pointsG, axis=0), dtype=np.int)
        # print("red = ",pointR," green = ", pointG)
        # print(pointR.shape, pointG.shape)
        if pointR.shape == (2,) and pointG.shape == (2,):
            output[pointR[1] - 10:pointR[1] + 10, pointR[0] - 10:pointR[0] + 10] = (255, 255, 255)
            output[pointG[1] - 10:pointG[1] + 10, pointG[0] - 10:pointG[0] + 10] = (255, 255, 255)
    else:
        pointR = None
        pointG = None
    cv2.imshow('frame', output)
    inp = cv2.waitKey(1)
    if inp & 0xFF == ord('z'):
        break
    elif inp & 0xFF == ord('q'):
        if pointR is not None and pointG is not None:
            print("point 1 : red = ", pointR, " green = ", pointG)
            calibPoints[0] = pointG
            calibPoints[1] = pointR
        else:
            print("red or green or both not detected")
    elif inp & 0xFF == ord('w'):
        if pointR is not None and pointG is not None:
            print("point 2 : red = ", pointR, " green = ", pointG)
            calibPoints[2] = pointR
        else:
            print("red or green or both not detected")
    elif inp & 0xFF == ord('e'):
        if pointR is not None and pointG is not None:
            print("point 3 : red = ", pointR, " green = ", pointG)
            calibPoints[3] = pointR
        else:
            print("red or green or both not detected")
    elif inp & 0xFF == ord('p'):
        print("calibration points = ", calibPoints)
        # temp = [pointDist(calibPoints[0], calibPoints[1]),
        #                   pointDist(calibPoints[0], calibPoints[2]),
        #                   pointDist(calibPoints[0], calibPoints[3])]
        # print("temp = ", temp)
        radius = np.mean([pointDist(calibPoints[0], calibPoints[1]),
                          pointDist(calibPoints[0], calibPoints[2]),
                          pointDist(calibPoints[0], calibPoints[3])])
        print("radius = ", radius)
        src = np.array(calibPoints, np.float32)
        dst = np.array([[0, 0], getImagePoint(radius, 60), getImagePoint(radius, 90), getImagePoint(radius, 120)],
                       np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        print("perspective transform matrix = ", M)
        projReady = True
    elif inp & 0xFF == ord('m'):
        if not projReady:
            print("not ready")
        else:
            print("ready")
            startMimic = True
    if startMimic and projReady:
        ser.open()
        # desired_angle = 90
        # current_angle = 0
        # error = desired_angle - current_angle
        # CO = P(desired_angle,current_angle)
        # CO = PI(desired_angle,current_angle)
        # CO = PID(desired_angle,current_angle)
        # angle = input("Angle: ")
        vel = 0
        if pointR is not None and pointG is not None:
            # print(pointR, pointG)
            pointsG = np.matmul(M,pointG)
            pointsR = np.matmul(M,pointR)
                
            angleA = int(np.arctan2(pointG[1] - pointR[1], pointR[0] - pointG[0]) * 180.0 / np.pi)
            print(angleA, angleB)
            angleB_array.append(angleB)
            vel = P(angleA, angleB)
            error_array.append(vel)
            # vel = PI(angleA, angleB)
            # vel = PID(angleA, angleB)
            print(vel)
            if vel > 10:
                vel = 10
            elif vel < -10:
                vel = -10
        ser.write(pack("B", int(vel+100)))
        angleB = int(ser.readline().decode())
        # print(angleA, angleB)
        sleep(.1)
        ser.close()

# When everything done, release the capture
x_axis=[i for i in range(len(angleB_array))]
plt.plot(x_axis,angleB_array)
plt.show()
plt.plot(x_axis,error_array)
plt.show()
cap.release()
cv2.destroyAllWindows()
