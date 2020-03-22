# Instructions

On the master and slave drones the communication by the I2C has to be set up and activated according to the instructions provided.
Once done that, physically connect the master and the slave as shown in the documentation, then:

- on the slave compile the **slave_rpi.cpp** using the arguments that are stated in the **slave_compilation** file

- run the so compiled executable on the slave rpi to initialize the comunication

- on the master rpi execute the **master_i2c.py** on python with the interactive mode **python -i master_i2c.py** then, on it run call the function to send data on the slave

- on the ssh terminal connected on the slave, once data is received, it will be printed out on the screen
 
