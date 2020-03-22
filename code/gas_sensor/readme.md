# Instructions

The code is intended to get some readings from a gas sensor, save the data locally and show them on an open terminal (either on system or by remote SSH). 
The use of the library is intended on a properly configured rpi, as explained in the documentation file.

It's intended for use on an RPi with the **SMBus** library and the **i2c-tools**.

The conversions of the data from binary to real values are done accordingly to the specifications provided in the original C++ library (that's still attached in the folder).

For string formatting the time library from Python is used
