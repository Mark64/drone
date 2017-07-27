// device manager header
// sets the framework for adding a device to the runtime
// by Mark Hill
#ifndef __device_manager_h
#define __device_manager_h

#include<stdint.h>
#include<pthread.h>

#define NAMELEN 100

enum dr_dev_type {
	DR_ACCEL = 0b1,
	DR_GYRO = 0b10,
	DR_MAG = 0b100,
	DR_TEMP = 0b1000,
	DR_PRESS = 0b10000,
	DR_VOLT = 0b100000,
	DR_DIS = 0b1000000,
	DR_POS = 0b10000000
};

enum dr_bus_type {
	DR_BUS_I2C,
	DR_BUS_SPI,
	DR_BUS_USB,
	DR_BUS_UART,
};

/**
 * only use negative flags when device is EXTREMELY bad
 * flags are not required
 */
enum dr_dev_flags {
	/* positives */
	DR_dev_FAST = 0b1,
	DR_dev_ACCURATE = 0b10,
	/* negatives */
	DR_dev_SLOW = -0b1,
	DR_dev_INACCURATE = -0b10,
};

struct dr_dev;
struct dr_dev_mngr;


/**
 * this struct defines all device operations and parameters
 * use the register_device and unregister_device functions to insert the device into the runtime
 *
 * **required before registration**
 * @dev_init		called when the device is registered to setup initial configuration
 * 						returns 0 on success
 * @dev_close		called when the device is unregistered or set to inactive due to ping failure
 * 						if possible device should be powered off
 * @ping			called to check if the device is accessibile
 * 						returns 0 on success
 * @type			the type of device this is
 * 						can be bitwise ORed if IC contains multiple devices
 * @flags			bitwise ORed flags
 * 						only use negatives for a very bad device that could slow or inhibit accurate calculations
 * @name			a friendly string for referring to the device
 * 						not necessarily unique
 * 						must use name_dr_dev()
 * @hw_name			a friendly string used to identify the module to be loaded for the device
 * @bus_type		the type of bus this device is using
 * @bus_num			which bus the device is attached to
 * @address			the i2c address, spi chipselect, etc.
 * 						usually the max i2c, spi, or usb speed the device supports
 * 
 * **managed by device_manager, values not used in register_device**
 * @active			a flag managed by device_manager to determine device data
 * 					0 = off
 * 					1 = active
 * 					-1 = ping failed, disconnected
 *
 * **extended docs**
 * @dev_init, @dev_close, @ping
 * 		@dev		a struct referring to the device from which to read
 * 						@type will contain the desired device readings
 * 							read a value for each number in count going in the order
 * 							of the enum dr_dev_type values
 * 						if @type = 0b111 and count is 4, read accel x, then y, then z,
 * 							and then x again and place them in that order into @data
 *
 */
struct dr_dev {
	// functions
	int (*dev_init)(struct dr_dev *dev);
	void (*dev_close)(struct dr_dev *dev);
	int (*ping)(struct dr_dev *dev);

	enum dr_dev_type type;
	enum dr_dev_flags flags;
	uint8_t *name[NAMELEN];
	uint8_t *hw_name[NAMELEN]
	
	enum dr_bus_type bus_type;
	unsigned int bus_num;
	uint16_t address;
	int active;

	pthread_mutext_t lock;
};

/**
 * used to name a dr_dev struct
 * @name			a null terminated string containing the desired name
 *
 * @return			0 on success and -1 on failure
 */
int name_dr_dev(const char *name);

/**
 * registers *dev with the device manager
 * @dev				the device to be registered
 *
 * @return			0 if successful
 */
int register_device(struct dr_dev *dev);

/**
 * unregisters *dev with the device manager and sends dev_close()
 * @dev				the devices to be unregistered
 */
void unregister_device(struct dr_dev *dev);

#endif
