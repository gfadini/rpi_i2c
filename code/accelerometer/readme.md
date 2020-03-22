# Instructions

This code shows how to use the Python classes to easily implement minimal libraries that can be used to access I2C sensors data, write and read.
The code is intended for use on a properly configured rpi, as explained it the other documentations files.

In particular the implementation uses **BMA222** sensor as accelerometer and **TMP006B** for temperature readings.

Moroever the code also uses python dictionaries in order to simplify the process of using the addresses and for the conversion of the binary files also a *two complement* function is included.
