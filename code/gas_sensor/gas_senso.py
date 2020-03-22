# load the required libraries
import smbus
import sys
import math
import time
import datetime
from subprocess import call

# enum{CO, NO2, NH3, C3H8, C4H10, CH4, H2, C2H5OH}

print(sys.version)	    # FOR DEBUG     purposes prints the kernel, gcc version, can be commented

# INITIALIZE THE STREAM OF DATA -> ADD CONDITION
while True:
    value = eval(raw_input("Insert the bus line 1 or 0\n")) # specify the bus to read data from (as RPi defines them)
    if  value == 1 or value == 0:
        line = value
        break    
print("Bus selected is: " + str(value))

call('i2cdetect -y -r ' + str(line), shell = True)

bus = smbus.SMBus(line) 				# 0 = /dev/i2c-0 | 1 = /dev/i2c-1

# DEVICE ADDRESSES
# MiCS-6814 I2C address
DEVICE1 = 0x04

# I2C REGISTER ADDRESSES
# CALIBRATION
ADDR_USER_ADC_NH3 = 0x08   			# register address to get the calibrated default value for NH3 channel (with no calibration, it should be the factory default value)
ADDR_USER_ADC_CO = 0x0A    			# register address to get the calibrated default value of CO channel
ADDR_USER_ADC_NO2 = 0x0C   			# register address to get the calibrated default value of CO channel
# REGISTER
CH_VALUE_NH3 = 0x01        			# register address to get the current value for NH3 channel
CH_VALUE_CO = 0x02         			# register address to get the current value for CO channel
CH_VALUE_NO2 = 0x03    			    # register address to get the current value for NO2 channel

Power_On =  ['l', [0x11,0x01]] 	# to power on the heating resistance
Power_Off = ['l', [0x11,0x00]] 	# to power off the heating resistance

# VALUES FOR THE DATA ACQUISITION
CO_ppm = 0.0               			# ppm value for CO
NO2_ppm = 0.0
NH3_ppm = 0.0              			# ppm value for NH3
C3H8_ppm = 0.0
C4H10_ppm = 0.0
CH4_ppm = 0.0
H2_ppm = 0.0
C2H5OH_ppm = 0.0
TEMP = 0.0
CHANNEL_ADC_TEMP = 0
CHANNEL_ADC_CO = 1

print "Connected to Device \n\t"
# print bus.read_word_data(termometer, DEVICE_1)

raw_input("Insert a key to start:")

# FUNCTION DEFINITIONS

def calibrated_value(gas):
    return {
        'NH3': ADDR_USER_ADC_NH3,
        'CO': ADDR_USER_ADC_CO,
        'NO2': ADDR_USER_ADC_NO2,
        'CH4' : ADDR_USER_ADC_CO,
    }[gas]

def current_value(gas):
	return{
	'NH3': CH_VALUE_NH3,
        'CO': CH_VALUE_CO,
        'NO2': CH_VALUE_NO2,
        'CH4': CH_VALUE_CO,
	}[gas]

def fraction(cal, curr):
   	value = curr/cal * (1023.0 - cal)/(1023.0 - curr)
        return value
   

def write_to_file(data):   
        # check if it makes sense!
        string = ' '.join(str(data_i)  for data_i in data)
        print string
	with open('test.txt', "a") as myfile:
            myfile.write(string)

def write_time(time):
	with open('test.txt', "a") as myfile:
            myfile.write(time)
            print(time)

def calculate_ppm(gas): # get ppm value of specified gas
        # print calibrated_value(gas)
        # raw_input("again? \n") # to debug each gas value please uncomment this and the previous line
        cal = bus.read_word_data(DEVICE1, calibrated_value(gas))
	curr = bus.read_word_data(DEVICE1, current_value(gas))
	ratio = fraction(cal, curr)
        
	if gas == 'NH3':
            c_NH3 = pow(ratio, -1.67) / 1.47
            c_C3H8 = pow(ratio, -2.518) * 570.164
            c_C4H10 = pow(ratio, -2.138) * 398.107
            values = [' NH3: ', c_NH3, 'C3H8: ', c_C3H8, 'C4H10: ', c_C4H10]
            write_to_file(values)
        elif gas == 'CH4':
            c_CH4 = pow(ratio, -4.363) * 630.957
            c_H2 = pow(ratio, -1.8) * 0.73
            c_C2H5OH = pow(ratio, -1.552) * 1.622
	    c_CO = pow(ratio, -1.179) * 4.385
	    values = [' CH4: ', c_CH4, 'H2: ', c_H2, 'C2H5OH: ', c_C2H5OH, 'CO: ', c_CO]
            write_to_file(values)
        elif gas == 'NO2':
            c_NO2 = pow(ratio, 1.007)/6.855
            values = [' NO2: ', c_NO2]
            write_to_file(values)


# MAIN PROGRAM

# INITIALIZATION

print (":"*15 + " DATA ACQUISITION  " + ":"*15)

from datetime import date
write_time("Data acquisition date: " + str(date.today()))

while True:
    bus.write_byte_data(0x04, 0x0A, 1)      # led on
    # bus.write_byte_data(0x04, 0x11, 1)    # heating the resistance
    # bus.write_byte_data(0x04, 0x00, 1)
    current_time = datetime.datetime.now()
    formatted_time = current_time.strftime("%H:%M:%S")
    write_time("\n" + formatted_time + ": ")
    # read_values = bus.read_word_data(DEVICE1, ADDR_USER_ADC_HN3)
    calculate_ppm('NH3') # with the same reading also other mixtures are calculated
    calculate_ppm('CH4') # with the same reading also other mixtures are calculated
    calculate_ppm('NO2') # with the same reading also other mixtures are calculated
    time.sleep(0.5)
    bus.write_byte_data(0x04, 0x0A, 0) # led off
    time.sleep(0.5)

