#Pycom LED control code 
#developed with code provided by Pycom 
#Created: 02/17/17
# -Joseph Fuentes
# -jafuentes3594@yahoo.com

#Importing Pycom library for board function
import pycom

#Turn off default pycom Heart beat LED
pycom.heartbeat(False)

# Change the LED to a color  in form 0xcolor code
# Reference chart: http://www.rapidtables.com/web/color/RGB_Color.htm
pycom.rgbled(0x7f0000) 
