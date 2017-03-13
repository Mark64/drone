// It became obvious that i2c is integral to more than just the sensor code, so it should be a separate file
//   with its own header
//
//   by Mark Hill

#include<iostream>
#include<stdint.h>

using namespace std;


// theres really no use in having to specify the bus for each operation since in most cases i2c_1 will be used throughout, but this adds the ability to use mutiple busses with this code
bool i2cSetBus(int8_t bus);

// read from register at i2c address
// similar to 'i2cget -y bus address register'
int16_t i2cRead(int16_t address, int16_t register);

// obvious a write is needed
// similar to 'i2cset -y bus address register value;
void i2cWrite(int16_t address. int16_t register, int16_t value);


