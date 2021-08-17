import RPi.GPIO as GPIO
import time
import os
import math
import sys
from simple_pid import PID
from SimpleServer import SimpleServer
import Adafruit_MCP4725

rc=23 #prve pismeno Lavy alebo pravy motor druhy je farba vodica
rw=24
rh=25
lc=8 #1 dopredu
lw=7 #2 dopredu
lh=1 #3 dopredu
pwmR = 12
pwmL = 13
revL = 17
breakL = 27
speedL = 22
revR = 16
breakR = 20
speedR = 21
pwmR = 12
pwmL = 13
hodnota = 0
LED = 6
iL = 0
iR = 0
pastL = 0
pastR = 0
dacR = Adafruit_MCP4725.MCP4725(address=0x60)
dacL = Adafruit_MCP4725.MCP4725(address=0x61)

global vL_ref, vR_ref
uL = 0
uR = 0
vL_ref = 0.0 # km.h-1
vR_ref = 0.0 # km.h-1
vL = 0
vR = 0

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
GPIO.setup(lh,GPIO.IN)
GPIO.setup(lw,GPIO.IN)
GPIO.setup(lc,GPIO.IN)
GPIO.setup(rh,GPIO.IN)
GPIO.setup(rw,GPIO.IN)
GPIO.setup(rc,GPIO.IN)
GPIO.setup(pwmR, GPIO.OUT) # PWM R
GPIO.setup(pwmL, GPIO.OUT) # PWM L
GPIO.setup(revL, GPIO.OUT) #revers L
GPIO.setup(breakL, GPIO.OUT) #Break L
GPIO.setup(speedL, GPIO.OUT) #Speed L
GPIO.setup(revR, GPIO.OUT) #Revers R
GPIO.setup(breakR, GPIO.OUT) #Break R
GPIO.setup(speedR, GPIO.OUT) #Speed R


global start_timeL, start_timeR, elapseL, elapseR, dirR, dirL, rpmR, rpmL
rpmL = 0
rpmR = 0
dirR = 1
dirL = 1
elapseL = 0
elapseR = 0
start_timeL = time.time()
start_timeR = time.time()

def handleRequest(req):
    global vL_ref, vR_ref
    if req != "":
        sp = req.split(" ")
        if len(sp) == 2:
            vL_ref = float(sp[0])
            vR_ref = float(sp[1])      
    return "OK"
          
server = SimpleServer('localhost', 2000, handleRequest)

def countR(channel): 
    global dirR, rw, rc
    if GPIO.input(revR):#je naopak otoceny prto iny kanal bolo rh
        dirR = -1
    else:
        dirR = 1
    global elapseR, start_timeR
    elapseR = (time.time() - start_timeR)
    start_timeR = time.time()
    #print("R prerusenie")

def countL(channel):
    global dirL, lh, lw
    if GPIO.input(17): #revL
        dirL = 1
    else:
        dirL = -1
    global elapseL,start_timeL    
    elapseL = (time.time() - start_timeL)
    start_timeL = time.time()
    

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
    if iR == 0:
        pastR = km_per_hour
        iR = 1
    if 0 < pastR and pastR > 2:
        km_per_hour = abs(km_per_hour)
        pastR = km_per_hour
    if pastR < 0 and pastR < -2 and km_per_hour > 0:
        km_per_hour = km_per_hour * (-1)
        pastR = km_per_hour
    if -0.1 < km_per_hour < 0.1:
        km_per_hour = 0
    pastR = km_per_hour
    return km_per_hour



def calculateL():
    global dirL, elapseL, start_timeL, rpmL,pastL,iL
    r_cm = 10
    if elapseL < (time.time() - start_timeL):
        elapseL = time.time() - start_timeL
    if elapseL != 0:
        rpmL = (1/elapseL)/16 * dirL
        circ_m = (2*3.1415)*0.1
        dist_km = circ_m/1000
        m_per_sec = (circ_m / elapseL)/16 * dirL
        km_per_hour = m_per_sec * 3.6
    if -0.1 < km_per_hour < 0.1:
        km_per_hour = 0

    # if iL == 0: #zaciatocna podmienka pri nastaveni
    #     pastL = km_per_hour
    #     iL = 1
    # if 0 < pastL and pastL > 2:
    #     km_per_hour = abs(km_per_hour)
    #     pastL = km_per_hour
    # if pastL < 0 and pastL < -2 and km_per_hour > 0:
    #     km_per_hour = km_per_hour * (-1)
    #     pastL = km_per_hour

    # pastL = km_per_hour
    return km_per_hour

GPIO.add_event_detect(rh, GPIO.RISING, callback = countR, bouncetime = 10)
GPIO.add_event_detect(lw, GPIO.RISING, callback = countL, bouncetime = 10)

P = 0.11 #0.11 iba pri regulacii soft
I = 0.13
D = 0
pidL = PID(P, I, D)
pidR = PID(P, I, D)
pidL.output_limits = (-1, 1)
pidR.output_limits = (-1, 1)
pidL.sample_time = 0.2
pidR.sample_time = 0.2
pidL.setpoint = 0
pidL.setpoint = 0
casovac = time.time()
casovac2 = time.time()
PWM_L = 0
PWM_R = 0
vR_ref = 0
vL_ref = 0
rev_speed = 0.3
try:
    while True:
        if (time.time() - casovac2 > 0.1):
            casovac2 = time.time()
            #pidL.setpoint = vL_ref
            #pidR.setpoint = vR_ref
            vL = calculateL()
            vR = calculateR()
#vL_ref - pozadovana rychlost; vL- aktualna rychlost; uL-regulacna odchylka
            uL = pidL(vL)
            uR = pidR(vR)
            if vL_ref == 100000:
                print("koniec")
                exit()
            if vL_ref == 0 and vR_ref == 0:
                # pozadovane zastavenie
                PWM_L = 0
                PWM_R = 0
                dacR.set_voltage(int(PWM_R))
                dacL.set_voltage(int(PWM_L))

            else:   
                if vL_ref > 0 :  # reverzy
                    GPIO.output(revL, GPIO.HIGH) 
                else:
                    GPIO.output(revL, GPIO.LOW)
                if vR_ref > 0 :
                    GPIO.output(revR, GPIO.LOW) 
                else:
                    GPIO.output(revR, GPIO.HIGH)

                PWM_L = 250+abs(vL_ref) * 1500 #1000 - 6.3km/h
                PWM_R = 250+abs(vR_ref) * 1500
   
                dacR.set_voltage(int(PWM_R))
                dacL.set_voltage(int(PWM_L))
                        
            if (time.time() - casovac)> 0.5: # 
                
                print("nastavenaL-", "{:.2f}".format(pidL.setpoint),"  aktualna:  ", "{:.2f}".format(vL),"  PWM:  ", "{:.2f}".format(PWM_L),"    ", "{:.2f}".format(uL))
                print("nastavenaR-", "{:.2f}".format(pidR.setpoint),"  aktualna:  ","{:.2f}".format(vR),"  PWM:  ","{:.2f}".format(PWM_R),"    ","{:.2f}".format(uR))
                casovac = time.time()
     

        time.sleep(0.01)#CPU ide na 100%
except KeyboardInterrupt: 
    print("Skratka ctrl+c")
    dacR.set_voltage(0)
    dacL.set_voltage(0)

    server.disconnect()
    exit()
finally:
    print("final")
    dacR.set_voltage(0)
    dacL.set_voltage(0)
    server.disconnect()
    exit()
    
