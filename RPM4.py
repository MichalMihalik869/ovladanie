import RPi.GPIO as GPIO
import time
import os
import math
import Adafruit_MCP4725

rc=23 #prve pismeno Lavy alebo pravy motor druhy je farba vodica
rw=24
rh=25
lc=8 #1 dopredu
lw=7 #2 dopredu
lh=1 #3 dopredu
iR = 0
iL = 0
pastL = 0
pastR = 0
revL = 17
revR = 16
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(revR, GPIO.OUT)
GPIO.setup(revL, GPIO.OUT)
GPIO.setup(lh,GPIO.IN)
GPIO.setup(lw,GPIO.IN)
GPIO.setup(lc,GPIO.IN)
GPIO.setup(rh,GPIO.IN)
GPIO.setup(rw,GPIO.IN)
GPIO.setup(rc,GPIO.IN)
os.system("rm fileR.txt")



global start_timeL, start_timeR, elapseL, elapseR, dirR, dirL, rpmR, rpmL
rpmL = 0
rpmR = 0
dirR = 1
dirL = 1
elapseL = 0
elapseR = 0
start_timeL = time.time()
start_timeR = time.time()

def zapis(speedR):
    #fileL = open('fileL.txt','w') 
    fileR = open('fileR.txt','a')   
    #fileL.write(str(speedL))
    fileR.write(str(speedR))
    fileR.write(str("\n"))
    #fileL.close()
    fileR.close()
    #print("zapis rychlosti")

def countR(channel): 
    global dirR, rh, rw
    if GPIO.input(16):#pomiesane kanaly asi tak je inac ako pri L
        dirR = -1
    else:
        dirR = 1
    global elapseR, start_timeR
    elapseR = (time.time() - start_timeR)
    start_timeR = time.time()
    #print("R prerusenie")

def countL(channel):
    global dirL, lw
    if GPIO.input(17):
        dirL = 1
    else:
        dirL = -1
    global elapseL,start_timeL    
    elapseL = (time.time() - start_timeL)
    start_timeL = time.time()
    #print("L prerusenie")

def calculateR():
    global dirR, elapseR, start_timeR, rpmR,iR,pastR
    if elapseR < (time.time() - start_timeR):
        elapseR = time.time() - start_timeR
    if elapseR != 0:
        rpmR = (1/elapseR)/16 * dirR
        circ_m = (2*3.1415)*0.1
        dist_km = circ_m/1000
        m_per_sec = (circ_m / elapseR)/16 * dirR
        km_per_hour = m_per_sec * 3.6
    if -0.01 < km_per_hour < 0.01:
        km_per_hour = 0
    #if km_per_hour > 0:
       # print("vecsie ako nula")
    if iR == 0:
        pastR = km_per_hour
        iR = 1
    if 0 < pastR and pastR > 2:
        km_per_hour = abs(km_per_hour)
        pastR = km_per_hour
    if pastR < 0 and pastR < -2 and km_per_hour > 0:
        km_per_hour = km_per_hour * (-1)
        pastR = km_per_hour
    if -0.01 < km_per_hour < 0.01:
        km_per_hour = 0
    pastR = km_per_hour
    return km_per_hour



def calculateL(r_cm):
    global dirL, elapseL, start_timeL, rpmL, pastL, iL
    if elapseL < (time.time() - start_timeL):
        elapseL = time.time() - start_timeL
    if elapseL != 0:
        rpmL = (1/elapseL*60 )/16 * dirL
        circ_cm = (2*3.1415)*r_cm
        dist_km = circ_cm/100000
        km_per_sec = (dist_km / elapseL)/16 * dirL
        km_per_hour = km_per_sec * 3600
    if -0.01 < km_per_hour < 0.01:
        km_per_hour = 0
    if iL == 0:
        pastL = km_per_hour
        iL = 1
    if 0 < pastL and pastL > 2:
        km_per_hour = abs(km_per_hour)
        pastL = km_per_hour
    if pastL < 0 and pastL < -2 and km_per_hour > 0:
        km_per_hour = km_per_hour * (-1)
        pastL = km_per_hour
    if -0.01 < km_per_hour < 0.01:
        km_per_hour = 0
    pastL = km_per_hour
    return km_per_hour

GPIO.add_event_detect(rh, GPIO.RISING, callback = countR, bouncetime = 2)
GPIO.add_event_detect(lw, GPIO.RISING, callback = countL, bouncetime = 2)

i=1
while i==1:
    time.sleep(0.5)
    a = calculateL(10) 
    b = calculateR()
    #print("lave- ",a)

    print("prave",b,"  RPM-",rpmR)
    print("lave ",a)
    if b < 0 :
        print("zaporne")
    if a < 0 :
        print("zaporne")
    #b = str(b)+str(";")
    #zapis(b)
    #


