import numpy as np
import cv2
from picamera import PiCamera
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

def capture():
    #Init camera
    #camera = PiCamera()
    #camera.resolution = (3280,2464)
    #camera.capture('Image.png')

    #Open image in greyscale
    I = cv2.imread('lt11.jpg',0)

    #Resize image
    I = cv2.resize(I, (0,0), fx=0.4, fy=0.4)

    return I

def detectedges(img):
    #Canny edge detection
    threshold = 700
    BW2 = cv2.Canny(img,threshold,threshold*0.4)

    minLineLength = 300
    maxLineGap = 0

    #Hough Transform
    lines = cv2.HoughLinesP(BW2,1,np.pi/180,0,100,320,130)


    #Return to BGR so I can draw green lines on the image
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    #Draw lines on picture
    for i in range(len(lines)):
        for x1,y1,x2,y2 in lines[i]:
            cv2.line(img,(x1,y1),(x2,y2),(0,255,0),4)
        cv2.imwrite('houghlines.jpg',img)

    #HoughLinesP saves a 3D array. convert to 2D array
    lines = lines.squeeze()
    
    return lines

def getdistance():
    #GPIO Pins on the rpi
    TRIG = 18 
    ECHO = 24

    #Set which pins get echo and trig
    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    
    GPIO.output(TRIG, False)

    #Wait for the sensor to settle
    time.sleep(2)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    #Get the time for the calculation
    while GPIO.input(ECHO)==0:
        pulse_start = time.time()

    while GPIO.input(ECHO)==1:
        pulse_end = time.time()

    #Calculate the distance
    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration * 17150

    distance = round(distance, 2)

    print "Distance:",distance,"cm"
    
    return 120

def mapping(lines, distance):
    #Calculate GSD
    Sw = 3.68
    Fr = 3.04
    #Height of wall
    wall_h = 30
    H = distance-wall_h
    Iw = 3280
    Ih = 2464
    GSD = (Sw*H*100)/(Fr*Iw)

    #Apply to lines
    for i in range(len(lines)):
        lines[i] = GSD*lines[i]
        
    #Return scaled lines
    return lines
    

def main():
    
    #Capture and mapping
    img = capture()
    lines = detectedges(img)
    distance = getdistance()
    reallines = mapping(lines,distance)

    #Positioning
    x = 0
    y = 0
    theta = 90

    for i in range(len(reallines)):
        print("{U: %s, %s, %s, (%s,%s),(%s,%s)}" % (x, y, theta, reallines[i][0], reallines[i][1]
                                                    ,reallines[i][2],reallines[i][3]))
        
    #Cleanup
    GPIO.cleanup()

main()
