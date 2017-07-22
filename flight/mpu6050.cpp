#include "mpu6050.h"
#include "i2cctl.h"
#include <iostream>
#include <cmath>


void gyroTest() {
	//begin by setting full-scale range to 250dps through FS_SEL set at 0
	
	
	uint8_t dataGConfig[2] = { GYRO_CONFIG, 0x00 };
	int gconfigSuc = i2c_write(mpuAddress, dataGConfig, 1);
	if (gconfigSuc == 0) {
		std::cout << "Gyro successfully set to 250 dps\n";
	} else {
		std::cout << "Failure to set Gyro to 250 dps\n";
	}

	//Now we must gather Factory Trim value of the self test response, FT[Xg],FT[Yg],FT[Zg]
	uint8_t gTest[3];
	if (i2c_read(mpuAddress, SELF_TEST_X, gTest, 3) == 0 ) {
		std::cout << "Factory Trim values (x,y,z) successfully read\n" ;
		std::cout << extracted(gTest[0],0,5) << "     " << extracted(gTest[1],0,5) << "     " << extracted(gTest[2],0,5) << "\n";
	 
	} else {
		std::cout << "Factory Trim values failed to read\n";
	}
	

	double ft[3] = { 25*131*pow(1.046, extracted(gTest[0], 0, 5)-1 ) , 
			-25*131*pow(1.046, extracted(gTest[1], 0, 5)-1 ) ,
			 25*131*pow(1.046, extracted(gTest[2], 0, 5)-1 ) };
	std::cout << "Factory Trim x " << ft[0] << " |  Factory Trim y " << ft[1] << " |  Factory Trim z " << ft[2] << "\n\n";	
	
}

unsigned short extracted(unsigned short value, int begin, int end) {
	unsigned short mask = (1 << (end - begin ) ) -1 ;
	return (value >> begin) & mask;
}

