// device manager header
// sets the framework for adding a sensor or device to the runtime
// by Mark Hill

#include<stdint.h>

#define NAMELEN 200

enum dr_dev_type {
	DR_ACCEL = 0b00000001,
	DR_GYRO = 0b00000010,
	DR_MAG = 0b00000100,
	DR_BARO = 0b00001000,
	DR_CAM = 0b00010000,
	DR_BATT = 0b00100000,
};
enum dr_bus_type {
	DR_BUS_I2C,
	DR_BUS_SPI,
	DR_BUS_USB,
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
 * @read			called to read from the device, returns the number of bytes read
 * @write			called to write to the device, returns the number of bytes written
 * @ping			called to check if the device is accessibile
 * 						returns 0 on success
 * @type			the type of device this is
 * @name			a friendly string for referring to the device
 * 						not necessarily unique
 * 						must use name_dr_dev()
 * @bus_type		the type of bus this device is using
 * @bus_num			which bus the device is attached to
 * @address			the i2c address, spi chipselect, etc.
 * @max_read_rate	the maximum rate at which data can be read without rereading data
 * @max_write_rate	the maximum rate data can be written to the device
 * 						usually the max i2c, spi, or usb speed the device supports
 * 
 * **managed by device_manager, values not used in register_device**
 * @active			a flag managed by device_manager to determine device data
 * 					0 = off
 * 					1 = active
 * 					-1 = ping failed, disconnected
 */
struct dr_dev {
	// functions
	int (*dev_init)(struct dr_dev *dev);
	void (*dev_close)(struct dr_dev *dev);
	int (*read)(struct dr_dev *dev, void *data, int count);
	int (*write)(struct dr_dev *dev, const void *data, int count);
	int (*ping)(struct dr_dev *dev);
	int (*configure)(struct dr_dev *dev, const void *data, int count);

	enum dr_dev_type type;
	char name[NAMELEN];
	
	enum dr_bus_type bus_type;
	unsigned int bus_num;
	uint16_t address;
	int active;
	unsigned int max_read_rate;
	unsigned int max_write_rate;
};

/**
 * used to name a dr_dev struct
 * returns 0 on success and -1 on failure
 */
int name_dr_dev(const char *name);


/**
 * registers *dev with the device manager
 * returns 0 if successful
 */
int register_device(struct dr_dev *dev);

/**
 * unregisters *dev with the device manager
 */
void unregister_device(struct dr_dev *dev);

/**
 * reads from all devices of <type> and places the output in data
 * returns the number of bytes read
 */
int read_from_devices(enum dr_dev_type type, void *data, int count);

/**
 * if dev is null, writes <data> to all devices that match <type>
 * else writes data to *dev
 * returns the number of bytes written
 */
int write_to_device(enum dr_dev_type type, struct dr_dev *dev, const void *data, int count);




