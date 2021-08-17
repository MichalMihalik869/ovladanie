from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
kit.servo[15].angle = 0
kit.servo[15].angle = 90

import Adafruit_MCP4725
dacR = Adafruit_MCP4725.MCP4725(address=0x60)
dacR.set_voltage(800)
dacR.set_voltage(0)


