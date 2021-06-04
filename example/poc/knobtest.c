
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <inttypes.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <byteswap.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>


#define BUFSIZE 256				// Typical charbuffer

#define INA219_ConfigurationRegister 0x00
#define INA219_ShuntVoltageRegister  0x01
#define INA219_BusVoltageRegister    0x02
#define INA219_PowerRegister         0x03
#define INA219_CurrentRegister       0x04
#define INA219_CalibrationRegister   0x05


struct ina219Config {
                                //   bit
    int reset;                  //    15  1=set all default values
    int na;                     //    14  na
    int busVoltage;             //    13  bus voltage (FSR): 0=16v; 1=32v (default); 
    int pga;                    // 11-12  PGA gain & range
                                //        Value  Gain   Range
                                //        ------ ---- ----------
                                //        0 (00)  /1  +/-  40 mv
                                //        1 (01)  /2  +/-  80 mv
                                //        2 (10)  /4  +/- 160 mv
                                //        3 (11)  /8  +/- 320 mv (default)

    // BADC - Bus Voltage ADC Resolution/Averaging
    int badcMode;               //    10  0=single shot (default); 1=average
    int badc;                   //   7-9  When badcMode=0, then precision
                                //        When badcMode=1, then samples  
                                //    
                                //        * same table as sadc
 
    // SADC - Shunt Voltage ADC Resolution/Averaging
    int sadcMode;               //     6  0=single shot (default); 1=average
    int sadc;                   //   3-5  When sadcMode=0, then precision
                                //        When sadcMode=1, then samples  
                                //
                                //        Mode  Value    Bits Samples Conversion Time
                                //        ----  -------- ---- ------- ---------------
                                //        0     0 (000)  9    1        84 μs
                                //        0     1 (001)  10   1       148 μs
                                //        0     2 (010)  11   1       276 μs
                                //        0     3 (011)  12   1       532 μs (default)
                                //        
                                //        1     0 (000)  12   1       532 μs
                                //        1     1 (001)  12   2       1.06 ms
                                //        1     2 (010)  12   4       2.13 ms
                                //        1     3 (011)  12   8       4.26 ms
                                //        1     4 (100)  12   16      8.51 ms
                                //        1     5 (101)  12   32      17.02 ms
                                //        1     6 (110)  12   64      34.05 ms
                                //        1     7 (111)  12   128     86.10 ms
 

    int mode;                   //   0-2  Operating Mode
                                //        
                                //        Value    Description
                                //        -------- -----------------
                                //        0 (000)  Power-down
                                //        1 (001)  Shunt voltage, triggered
                                //        2 (010)  Bus voltage, triggered
                                //        3 (011)  Shunt and bus, triggered
                                //        4 (100)  ADC off (disabled)
                                //        5 (101)  Shunt voltage, continuous
                                //        6 (110)  Bus voltage, continuous
                                //        7 (111)  Shunt and bus, continuous (default)
     
};

unsigned long long currentTimeMillis() {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);

    return (unsigned long long)(currentTime.tv_sec) * 1000 +
        (unsigned long long)(currentTime.tv_usec) / 1000;
}

uint16_t configHi(struct ina219Config config) {
    uint16_t high=0;

    high |= (0x01 & config.reset)       << 7;  // 1 bit   
    high |= (0x01 & config.na)          << 6;  // 1 bit   
    high |= (0x01 & config.busVoltage)  << 5;  // 1 bits
    high |= (0x03 & config.pga)         << 3;  // 2 bits
    high |= (0x01 & config.badcMode)    << 2;  // 1 bit
    high |= (0x06 & config.badc)        >> 1;  // 2 most significant bits

    return high;
}
uint16_t configLo(struct ina219Config config) {
    uint16_t low=0;

    low |= (0x01 & config.badc)         << 7;  // least significant bit
    low |= (0x01 & config.sadcMode)     << 6;  // 1 bits
    low |= (0x07 & config.sadc)         << 3;  // 3 bits
    low |= (0x07 & config.mode)         << 0;  // 3 bits  

    return low;
}
uint16_t config2int(struct ina219Config config) {
    uint16_t high=0;
    uint16_t low=0;

    high |= (0x01 & config.reset)       << 7;  // 1 bit   
    high |= (0x01 & config.na)          << 6;  // 1 bit   
    high |= (0x01 & config.busVoltage)  << 5;  // 1 bits
    high |= (0x03 & config.pga)         << 3;  // 2 bits
    high |= (0x01 & config.badcMode)    << 2;  // 1 bit
    high |= (0x06 & config.badc)        >> 1;  // 2 most significant bits

    low |= (0x01 & config.badc)         << 7;  // least significant bit
    low |= (0x01 & config.sadcMode)     << 6;  // 1 bits
    low |= (0x07 & config.sadc)         << 3;  // 3 bits
    low |= (0x07 & config.mode)         << 0;  // 3 bits  

    return (high << 8)|low;
}

int main(int argc, char **argv) {

  	int ina219_handle;

	int ina219_address = 0x40;		// 0x40 is the default address on i2c bus for ina219

		// open i2c device
	if ((ina219_handle = wiringPiI2CSetup(ina219_address)) < 0) {
		printf("Error: Couldn't open device 0x%02x: %s\n", ina219_address, strerror(errno));
		exit(EXIT_FAILURE);
	}


 	printf("connected to 0x%02x via wiringPi\n", ina219_address);


    struct ina219Config config;


    config.reset       = 0;  // 0=no action; 1=reset
    config.busVoltage  = 0;  // 0=16v; 1=32v
    config.pga         = 3;  // 3=+/ 320mv

    config.badcMode    = 0;  // 0=single shot
    config.badc        = 3;  // 12 bit precision
    config.sadcMode    = 0;  // 0=single shot
    config.sadc        = 3;  // 12 bit precision
    config.mode        = 3;  // shunt & bus; continuous


	if (wiringPiI2CWriteReg16(ina219_handle, INA219_ConfigurationRegister, 0xff)!=0) { // reset
		perror("Write to config register");
		exit(EXIT_FAILURE);
	}
        delay(10);
	if (wiringPiI2CWriteReg16(ina219_handle, INA219_ConfigurationRegister, __bswap_16(config2int(config)))!=0) {
		perror("Write to config register");
		exit(EXIT_FAILURE);
	}

    	int16_t val1, val2, val3, val4;


	printf("%-13s    shunt    bus  power  current\n", "timestamp");
	while (1) {
		if (wiringPiI2CWriteReg16(ina219_handle, INA219_ConfigurationRegister, __bswap_16(config2int(config)))!=0) {
			perror("Write to config register");
			exit(EXIT_FAILURE);
		}
		delay(1);
		val1 = __bswap_16(wiringPiI2CReadReg16(ina219_handle, INA219_ShuntVoltageRegister));
		val2 = __bswap_16(wiringPiI2CReadReg16(ina219_handle, INA219_BusVoltageRegister));
		val3 = __bswap_16(wiringPiI2CReadReg16(ina219_handle, INA219_PowerRegister));
		val4 = __bswap_16(wiringPiI2CReadReg16(ina219_handle, INA219_CurrentRegister));

		printf("%lld   0x%04x 0x%04x 0x%04x   0x%04x",currentTimeMillis(), val1, val2, val3, val4);
		printf("\r"); fflush(stdout);
                delay(100);
	}
}
