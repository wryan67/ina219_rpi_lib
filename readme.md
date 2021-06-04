# INA219 Raspberry Pi Library
This library reads the INA219 chip and returns the values caputred by the chip.
 The example app will read an INA219 chip and print the voltage, power, and current values.

Copyright (c) 2021 Wade Ryan


## Requirements
On your Raspberry Pi, please use the raspi-config program to enable the I2C interface.
Then use the gpio command to make sure your i2c devices can be found.  The default address 
for an ADS1115 chip is 0x48.  

    $ sudo raspi-config
    $ gpio i2cd

## Download
Use git to download the software from github.com:

    $ cd ~/projects { or wherever you keep downloads }
    $ git clone https://github.com/wryan67/ina219_rpi_lib.git

## Install
To compile this library, navigate into the src folder and use the make utility to compile 
and install the library.

    $ cd [project folder]
    $ cd src
    $ make && sudo make install


## Compiling
Complie your applications using these command line arguments: -lwiringPi 


## Example
To run the example program, nagaviate into the example folder and use make to compile the program.  The timestamp 

    $ cd ../example
    $ make 
    $ ./knobtest

