import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO
import time
import threading

##===============INIT==============

GPIO.setmode(GPIO.BOARD)
IN3 = 36
IN4 = 38
ENB = 40
IN1 = 35
IN2 = 37
ENA = 31
ServoPin = 12
GPIO.setwarnings(False)
def motor_init():
    global pwm_ENA
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    global pwm_ENB
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
def servo_init():
    global pwm_servo
    GPIO.setup(ServoPin, GPIO.OUT,initial=GPIO.LOW)
    pwm_servo = GPIO.PWM(ServoPin, 100)
    pwm_servo.start(0)
    time.sleep(0.1)
##===============INIT==============
    
##===============CTRL==============
def toward(L,R):      
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)   
    pwm_ENA.ChangeDutyCycle(L)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)   
    pwm_ENB.ChangeDutyCycle(R)             

def back(L,R):
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN1, GPIO.LOW)   
    pwm_ENA.ChangeDutyCycle(L)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)   
    pwm_ENB.ChangeDutyCycle(R)
   
def ctrturn(angle):      
    pwm_servo.ChangeDutyCycle(5 + 20 * (93+angle)/270)
    time.sleep(0.25)
    pwm_servo.ChangeDutyCycle(0)
    time.sleep(0.02)
    
def stop():
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN1, GPIO.LOW)   
    pwm_ENA.ChangeDutyCycle(0)
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)   
    pwm_ENB.ChangeDutyCycle(0)

def ctrl(Ag,speedL,speedR):
    if (int(Ag) == -1):
        stop()
        ctrturn(0)
    elif (int(Ag) in range(1,179)):
        turn(int(-(int(Ag)-90)/1.6))
        toward(speedL ,speedR + getc((int(Ag)-90)) * 8)        
    else:
        turn(int((int(Ag)-270)/1.6))
        back(speedL ,speedR - getc((int(Ag)-270)) * 8)
        
        
##===============CTRL==============

##===============AUTO==============

global e
e = [0,0,0] 
global angle
global Se
global Ion
Ion = 0
angle = 0
Se = 0
lower_black = np.array([0, 0, 0]) 
upper_black = np.array([180, 255, 78]) 

import random
global auto
auto = 0    

########################line def
def getc(num): 
    if num != 0:
        return int(num/abs(num))
    else:
        return 0
      
def up(speed,Dspeed):     
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)   
    pwm_ENA.ChangeDutyCycle(speed+Dspeed/2)  # left
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)   
    pwm_ENB.ChangeDutyCycle(speed-Dspeed/2)  # right       
    
def turn(angle):     
    pwm_servo.ChangeDutyCycle(5 + 20 * (93+angle)/270)
    #time.sleep(0.01)
    #print(95+angle)
     
def stopmotor():
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN1, GPIO.LOW)   
    pwm_ENA.ChangeDutyCycle(0)   
    GPIO.output(IN4, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)   
    pwm_ENB.ChangeDutyCycle(0)
    pwm_servo.ChangeDutyCycle(5 + 20 * 93/270)
    time.sleep(0.2)
    pwm_servo.ChangeDutyCycle(0)
    

def endif():
    global frame1
    frameend = frame1[180:220,80:240]
    hsvend = cv.cvtColor(frameend, cv.COLOR_BGR2HSV)
    maskend = cv.inRange(hsvend, lower_black, upper_black)
    image,contoursend,hierarchyend = cv.findContours(maskend, 3, 1)
    if len(contoursend) > 0:
        cntend = contoursend[0]   
        areaend = cv.contourArea(contoursend[0])+cv.arcLength(cntend,True)
        Mend = cv.moments(cntend)
        if Mend["m00"] != 0:
            cXend=int(Mend["m10"]/Mend["m00"])  
            angle = getc(cXend - 100) * 50#====not end
            turn(angle)
            speed = civ - ckv * abs(angle)
            Dspeed = getc(angle) * 8   
            up(speed,Dspeed)
        else:
            stopmotor()
    else:
        stopmotor()


def tracking():
    if(auto is 1):
        global angle
        global Se
        ret, frame2 = cap.read()
        frame1 = cv.resize(frame2, (320,240), interpolation=cv.INTER_AREA) 
        frame = frame1[90:220,40:280]#85..145
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
        mask = cv.inRange(hsv, lower_black, upper_black)
        cv.line(mask,(0,0),(240,0),[0,0,0],1) 
        cv.line(mask,(0,130),(240,130),[0,0,0],1) 
        image,contours,hierarchy = cv.findContours(mask, 3, 1) 
        if len(contours) > 0:
            cnt = contours[0]  
            M = cv.moments(cnt)
            if M["m00"] != 0:
                cX=int(M["m10"]/M["m00"])   
                area = cv.contourArea(contours[0])+cv.arcLength(cnt,True)     
                if (area > 100 and area < 9000):
                    e[1] = e[0]
                    e[0] = cX - 100                                                        
                    if (abs(e[0]) >  10):
                        Up = cp * e[0] 
                        ''' remove ui
                        if (abs(e[0]) > 35): 
                            Ion = 0
                        else:
                            Ion = 1'''
                            
                        if (abs(angle) < 45):
                            Se += e[0] 
                        Ui = ci * Se * Ion / 2 
                        #Ui = 0 
                        Ud = cd * (e[0]-e[1])
                        angle =  Up + Ui + Ud
                 
                        if (abs(angle) > 65):   
                            angle = getc(angle) * 65
                        if (angle < 0):
                            angle  = angle + 10
                    else:
                        angle = 0
                    turn(angle)
                
                    speed = civ - ckv * abs(angle)
                    Dspeed = getc(angle) * cdv
                    up(speed,Dspeed)
                else:
                    stopmotor()
            #endif()
            else:
                stopmotor()
            #endif()  
        else:
            stopmotor()
            #endif()
    else:
        stopmotor()
    #print(angle,speed,Dspeed)
        #cv.imshow('mask', mask)   
        #k = cv.waitKey(1) & 0xFF   
    if(auto is 1):
        threading.Timer(0.0001,tracking).start()     
    else:
        stopmotor()
        
def control():
    #stopmotor()
    global auto
    auto = 0
    cap.release()
    #cv.destroyAllWindows()
    #stop()
    
def track_init(P,I,D,IV,KV,DV):
    global auto
    auto = 1
    global cap
    cap = cv.VideoCapture(0)
    global cp
    global ci
    global cd
    global civ
    global ckv
    global cdv
    global e
    e = [0,0,0] 
    global angle
    global Se
    global Ion
    Ion = 0
    angle = 0
    Se = 0
    cp = float(P)
    ci = float(I)
    cd = float(D)
    civ = int(IV)
    ckv = float(KV)
    cdv = int(DV)
    timer = threading.Timer(0.02,tracking)
    timer.start()
    
    
##===============AUTO==============
cap = cv.VideoCapture(0)
motor_init()
time.sleep(0.5)
servo_init()
cap.release()
print("motoring")