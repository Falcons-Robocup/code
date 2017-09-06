# Author: Dirk-Jan Vethaak
# Date: 2015-11-19
import serial
import time
import numpy as np
      


print " \n\n ------CAL COMPASS-------\n\n if port can not be found , \n then press compass reset button..\n .. loading wait 2 sec...\n"

time.sleep(1)
s= serial.Serial(port='/dev/ttyUSB0', baudrate =9600, stopbits=serial.STOPBITS_TWO)
s.close()	    
	    
s.open()
	# 0x31,0x45,0x5A


time.sleep(1)
s.write(chr(0x31))
time.sleep(1)

s.write(chr(0x45))
time.sleep(1)

s.write(chr(0x5A))
time.sleep(1)

# first cal

print ("Cal first point 0 degree (NORD)  CLOCKWISE -> EAST")
raw_input("Press Enter to calibrate...\n\n")

s.write(chr(0x5A))
# rotate 90 degrree  



print ("Cal 2nd point 90 degree (ROTATE 90 degree")
raw_input("Press Enter to calibrate...\n\n")
s.write(chr(0x5A))

#rotate again 90 degree (180


print ("Cal 3th point 180 deree (ROTATE 90 degree again ")
raw_input("Press Enter to calibrate...\n\n")
s.write(chr(0x5A))

print ("Cal 4th point 270 deree (ROTATE 90 degree again ")
raw_input("Press Enter to calibrate...\n\n")
s.write(chr(0x5A))

print ("Cal Done...\n")


		
s.close()
	    










