from socket import *
from motor import *
import cv2 as cv
import numpy as np

import time
import threading
import sys

global speedL
global speedR
speedL = 60
speedR = 60
global auto

def getip():
    try:
        s = socket(AF_INET,SOCK_DGRAM)
        s.connect(('8.8.8.8',80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip

HOST = getip()
print (HOST)
PORT = 8000
BUFSIZE = 1024
ADDR = (HOST,PORT)

tcpSerSock = socket(AF_INET, SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(5)
while True:
    #print ('Waiting for connection...') 
    (tcpCliSock,addr) = tcpSerSock.accept()
    #print (addr)
    try:
        while True:
            data = ''
            data = tcpCliSock.recv(BUFSIZE)
            if not data:
                break
            print(data)
            if data == b'CTRL':               
                stopmotor()
                control()
                auto = 0
            elif data[:2] == b'Ag':
                strsplit = str(data).split(":")
                Ag = strsplit[1]
                ctrl(Ag,speedL,speedR)
            elif data[:4] == b'AUTO':
                strsplit = str(data).split(":")
                P = strsplit[1]
                I = strsplit[2]
                D = strsplit[3]
                IV = strsplit[4]
                KV = strsplit[5]
                DV = strsplit[6]
                auto = 1
                track_init(P,I,D,IV,KV,DV)
            elif data[:2] == b'RV':
                speedR = int(data[-2:])
            elif data[:2] == b'LV':
                speedL = int(data[-2:])
            elif data == b'LEFT':
                ctrturn(-35)
            elif data == b'RIGHT':
                ctrturn(45)
            elif data == b'RESET':
                ctrturn(0)
            elif data == b'STOP':         
                stop()
            elif data == b'UP':
                toward(speedL,speedR)
            elif data == b'DOWN':
                back(speedL,speedR)  
    except KeyboardInterrupt:
        tcpSerSock.close();
        pwm_servo.stop()
        pwm_ENA.stop()
        pwm_ENB.stop()
        GPIO.cleanup()
        break

