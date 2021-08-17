#192.168.1.101
#pi
#michal869
#158.193.224.101

import RPi.GPIO as GPIO
from pyPS4Controller.controller import Controller
import time
import os
from SimpleServer import SimpleClient

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
global x,y,pomocna
x = 0
pomocna = 0
y = 0
pwmR = 12
pwmL = 13
rychlostL = 0
rychlostR = 0
 #hodnoty na riadenie pomocou packy R3 rozsah <-32767, 32767>


GPIO.setmode(GPIO.BCM)
GPIO.setup(LED, GPIO.OUT)
client = SimpleClient('localhost', 2000)

################################################################################
def ovladanieR():
    global x,y,rychlostL,rychlostR  
    #GPIO.output(revL, GPIO.HIGH)
   # GPIO.output(revR, GPIO.LOW)
    vektor= ((y*y)+(x*x)) **(1/2)
    #rychlostL = (vektor*((x / 65535) + 0.5)/32767) * 5 #maximalna rychlost bude 5 km/h
    #rychlostR = (vektor*(0.5 - (x / 65535))/32767) * 5
    #rychlostL = (y*((x / 65535) + 0.5)/32767) * 5 #maximalna rychlost bude 5 km/h
    #rychlostR = (y*(0.5 - (x / 65535))/32767) * 5
    rychlostL = ( x + y )/65535 
    rychlostR = ( -y + x )/65535 

    if -1000 < x < 1000 and -1000 < y < 1000:
        rychlostL = 0
        rychlostR = 0
        print ("pasmo ticha")
    rychlostL = "%.3f" % rychlostL
    rychlostR = "%.3f" % rychlostR
    print("L:",rychlostL)
    print("R:",rychlostR)
    zapis(rychlostL,rychlostR)

###################################################################    
def zapis(speedL, speedR):
    client.write(str(speedL) + " " + str(speedR))

def brzdi():
    print("funkcia brzdi")
    zapis(0,0)
    time.sleep(0.05)
def brzdiR():
   # print("funkcia brzdi")
    zapis(0,0)
      
def nebrzdi():
    #print("\nnebrzdi")
    time.sleep(0.05)
def nebrzdiR():
    print("\nnebrzdi")
    time.sleep(0.001)
    
def disconnect():  #akcie vykonané po odpojení joysticku
    brzdi()
    print("joystick je odpojeny")
    zapis(100000, 100000)    
    GPIO.output(LED, GPIO.LOW) 
    exit()
    #GPIO.cleanup()
    # any code you want to run during loss of connection with the controller or keyboard interrupt
def connect(): 
    GPIO.output(LED, GPIO.HIGH) 
    zapis(0, 0)

####################################################################################################
class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_circle_press(self): #trojuholnik dopredu
       print("idem do predu")
       nebrzdi()
       zapis(0.5,0.5)
       
      
    def on_circle_release(self):
       print("stojim")
       zapis(0,0)
       brzdi()
       
            
    def on_x_press(self):    #gulicka do prava
       print("doprava")  
      # GPIO.output(revL, GPIO.HIGH) # kvoli tomu že je naopak koleso
      # GPIO.output(revR, GPIO.HIGH)  # otacka na mieste  
       #chle.ChangeDutyCycle(65)
       #chri.ChangeDutyCycle(60) 
       zapis(0.6,-0.3) 
       nebrzdi()   
       
    def on_x_release(self):
       brzdi() 
       print("stojim")
       zapis(0,0)
      # GPIO.output(revR, GPIO.LOW)  #zrušenie reverszu
       
    def on_triangle_press(self): #stvorec dolava
       print("dolava")
       zapis(-0.3,0.6)       
       #GPIO.output(revL, GPIO.LOW)
       #GPIO.output(revR, GPIO.LOW)
       nebrzdi()
       #chle.ChangeDutyCycle(60)
       #chri.ChangeDutyCycle(65)
       
    def on_triangle_release(self):
       print("stojim")
       brzdi()
       zapis(0,0)
       #GPIO.output(revL, GPIO.LOW)
       
    def on_square_press(self):   #X dozadu
        print("idem do dozadu")
        zapis(-0.5,-0.5) 
        #GPIO.output(revL, GPIO.LOW)
        #GPIO.output(revR, GPIO.HIGH)
        nebrzdi()
       
    def on_square_release(self):
        print("stojim")
        brzdi()
    
    def on_L1_press(self):
        global pomocna  
        pomocna = 1
    def on_L1_release(self):
        global pomocna
        pomocna = 0
        brzdi()
           
    def on_R1_press(self):                   #nitro
        print("L1 je vypnute")   
          
    def on_R1_release(self):
        print("L1 je pustene")
        
        
    def on_R3_up(self, value):
        global x, pomocna
        if pomocna == 1:      
            nebrzdiR() 
            #GPIO.output(revL, GPIO.HIGH)
            #GPIO.output(revR, GPIO.LOW)       
            x = int(value) * (-1)       #kvoli tomuze je to zle napamovane v kniznici
            ovladanieR()
        else:
            brzdiR()
         
            
    def on_R3_left(self, value):
        if pomocna == 1: 
            nebrzdiR()        
            global y 
            y = int(value)
            ovladanieR()
        else:
            brzdiR()
        
            
    def on_R3_right(self, value):
        if pomocna == 1:
            nebrzdiR()        
            global y
            y = int(value)
            ovladanieR()
        else:
            brzdiR()
        
    def on_R3_down(self, value):
        global x, pomocna
        if pomocna == 1:      
            nebrzdiR() 
            #GPIO.output(revL, GPIO.HIGH)
            #GPIO.output(revR, GPIO.LOW)       
            x = int(value) * (-1)       #kvoli tomuze je to zle napamovane v kniznici
            ovladanieR()
        else:
            brzdiR()
              
    def on_R3_x_at_rest(self):
        pass
    def on_R3_y_at_rest(self):   
        brzdiR()    
    def on_L3_press(self):           #odpojenie bluethoth a vypnutie programu
        os.system("sudo service bluetooth restart")
########################################################################################
zapis(0,0)

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)

# you can start listening before controller is paired, as long as you pair it within the timeout $
controller.listen(on_connect=connect, on_disconnect=disconnect)#funkcie connect  disconet sa vykonuja pri pripojeni a odpojeni
