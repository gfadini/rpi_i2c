#!/usr/bin/env python
# This library allows to enstablish simple i2c communications with RPi
# we use the command to detect the presence of the device on the RPi, finding the table of addressed with i2cdetect
import smbus
import sys
import time
import os
import datetime
import math

# DEBUG SYS VERSION
print(sys.version)

# DEVICE ADDRESSES
# TMP006B
DEVICE1 = 0x41
# BMA222
DEVICE2 = 0x18

class BMA222():
    bus = smbus.SMBus(1)
    def __init__(self):
        self.address = 0x18
        self.ACC_X_NEW = 0x02 # bit 0 is new data
        self.ACC_X = 0x03
        self.ACC_Y_NEW = 0x04 # bit 0 is new data
        self.ACC_Y = 0x05
        self.ACC_Z_NEW = 0x06 # bit 0 is new data
        self.ACC_Z = 0x07
        self.BANDWIDTH = 0x10
        self.RANGE = 0x0F
        self.SELF_TEST = 0x32
        self.INTERRUPT = 0x09

        # self.bus.write_byte_data(address, 0x07, 0x00) # Setup the Mode
        # self.bus.write_byte_data(address, 0x06, 0x10) # Calibrate

    def getValueX(self):
        try:
            return self.bus.read_byte_data(self.address, self.ACC_X)
        except ValueError as err:
            print(err.args)

    def getValueY(self):
        try:
            return self.bus.read_byte_data(self.address, self.ACC_Y)
        except ValueError as err:
            print(err.args)

    def getValueZ(self):
        try:
            return self.bus.read_byte_data(self.address, self.ACC_Z)
        except ValueError as err:
            print(err.args)

class TMP006():
    bus = smbus.SMBus(1)
    def __init__(self):
        self.address = 0x41
        self.TEMPORARY = 0x08 # bit 0 is new data
        self.ID = 0xff
        self.CONFIG = 0x02
        self.MANUFACTURER = 0xFE
        self.TEMPERATURE = 0x01
        self.VOBJECT = 0x00

        # self.bus.write_byte_data(address, 0x07, 0x00) # Setup the Mode
        # self.bus.write_byte_data(address, 0x06, 0x10) # Calibrate
    def getTemp(self):
        try:
            return self.bus.read_word_data(self.address, self.TEMPERATURE)
        except ValueError as err:
            print(err.args)
        
# HASHES FREQUENCY -> BITVALUE
BANDWIDTH_VALUES = {
					'8.81' : '1000',
					'16.63' : '1001',
					'32.25' : '1010',
					'63.5' : '1011',
					'126' : '1100',
					'251' : '1101',
					'501' : '1110',
                                        '1001' : '1111',
					}
def add_inverse_dictionary(dictionary):
    inverse = {v : k for k, v in dictionary.iteritems()}
    dictionary.update(inverse)

add_inverse_dictionary(BANDWIDTH_VALUES)

# HASHES RANGES PLUS MINUS G -> BITVALUE
RANGE_VALUES = {
				'2g' : '0011',
				'4g' : '0101',
				'8g' : '1000',
				'16g': '1100',
				}
add_inverse_dictionary(RANGE_VALUES)

G = 9.81

CONVERSION_VALUES = {
                        '2g' : 15.6e-3 * G,
                        '4g' : 31.3e-3 * G,
                        '8g' : 62.5e-3 * G,
                        '16g' : 125e-3 * G,
                    }    

print "Connected to Device \n\t"
# print bus.read_word_data(termometer, DEVICE_1)
# also bus.read_i2c_block_data(device, register)

raw_input("Insert key to start data stream: \n\t")

file = open("text", "w+")

# print CONVERSION_VALUES[range]

def twos_complement(input_value, num_bits):
	'''Calculates a two's complement integer from the given input value's bits'''
	mask = 2**(num_bits - 1)
	return -(input_value & mask) + (input_value & ~mask)

# READ FROM THE ACCELEROMETER
accelerometer = BMA222()
thermometer = TMP006()
while True:
    
    acc_X = accelerometer.getValueX()
    acc_Y = accelerometer.getValueY()
    acc_Z = accelerometer.getValueZ()

    acc_X = twos_complement(acc_X, 8)*CONVERSION_VALUES['2g']
    acc_Y = twos_complement(acc_Y, 8)*CONVERSION_VALUES['2g']
    acc_Z = twos_complement(acc_Z, 8)*CONVERSION_VALUES['2g']
    wrd = "ACC_X: " + str(acc_X) + "\nACC_Y: " + str(acc_Y)  + "\nACC_Z: " + str(acc_Z)  
    print wrd
    # print twos_complement(thermometer.getTemp()>>2,16)/32.0
    # print '{0:b}'.format(thermometer.getTemp())
    # w = (thermometer.getTemp() & 255)
    # k = (thermometer.getTemp() & 65280)
    # n = w + k + 2**8
    # print n/32.0
    print 'TEMP: ' + str(thermometer.getTemp()>>10)

    t = str(time.time())
    file.write(t + " : " + wrd + "\n")   #" : %f\r\n" % wrd)
    time.sleep(1)
    os.system('clear')

