// sensor manager header
// sets the framework for adding a sensor to the runtime
// by Mark Hill
#ifndef __sensor_manager
#define __sensor_manager

#include<stdint.h>

#define NAMELEN 200

enum dr_sens_type {
	DR_ACCELX = 0b1,
	DR_ACCELY = 0b10,
	DR_ACCELZ = 0b100,
	DR_GYROX = 0b1000,
	DR_GYROY = 0b10000,
	DR_GYROZ = 0b100000,
	DR_MAGX = 0b1000000,
	DR_MAGY = 0b10000000,
	DR_MAGZ = 0b100000000,
	DR_TEMP = 0b1000000000,
	DR_PRES = 0b10000000000,
	DR_VOLT = 0b100000000000,
	DR_DIS = 0b1000000000000,
	DR_POSX = 0b10000000000000,
	DR_POSY = 0b100000000000000,
	DR_POSZ = 0b1000000000000000
};

enum dr_bus_type {
	DR_BUS_I2C,
	DR_BUS_SPI,
	DR_BUS_USB,
	DR_BUS_UART,
};

/**
 * only use negative flags when sensor is EXTREMELY bad
 * flags are not required
 */
enum dr_sens_flags {
	/* positives */
	DR_sens_FAST = 0b1,
	DR_sens_ACCURATE = 0b10,
	/* negatives */
	DR_sens_SLOW = -0b1,
	DR_sens_INACCURATE = -0b10,
};

struct dr_sens;
struct dr_sens_mngr;


/**
 * this struct defines all sensor operations and parameters
 * use the register_sensor and unregister_sensor functions to insert the sensor into the runtime
 *
 * **required before registration**
 * @sens_init		called when the sensor is registered to setup initial configuration
 * 						returns 0 on success
 * @sens_close		called when the sensor is unregistered or set to inactive due to ping failure
 * @read			called to read from the sensor, returns the number of values
 * @write			called to write to the sensor, returns the number of values written
 * @ping			called to check if the sensor is accessibile
 * 						returns 0 on success
 * @type			the type of sensor this is
 * 						can be bitwise ORed if IC contains multiple sensors
 * @flags			bitwise ORed flags
 * 						only use negatives for a very bad sensor that could slow or inhibit accurate calculations
 * @name			a friendly string for referring to the sensor
 * 						not necessarily unique
 * 						must use name_dr_sens()
 * @bus_type		the type of bus this sensor is using
 * @bus_num			which bus the sensor is attached to
 * @address			the i2c address, spi chipselect, etc.
 * @max_read_rate	the maximum rate at which data can be read without rereading data
 * @max_write_rate	the maximum rate data can be written to the sensor
 * 						usually the max i2c, spi, or usb speed the sensor supports
 * 
 * **managed by sensor_manager, values not used in register_sensor**
 * @active			a flag managed by sensor_manager to determine sensor data
 * 					0 = off
 * 					1 = active
 * 					-1 = ping failed, disconnected
 * @hw_id			a number identifying the sensor
 * 						created by hashing the name and verifying uniqueness
 * 						in the event of collision, increments and tries again
 *
 * **optional**
 * @configure		sensor specific configuration setup
 *
 * **extended docs**
 * @sens_init, @sens_close, @read, @write, @ping, @configure
 * 		@sens		a struct referring to the sensor from which to read
 * 						@type will contain the desired sensor readings
 * 							read a value for each number in count going in the order
 * 							of the enum dr_sens_type values
 * 						if @type = 0b111 and count is 4, read accel x, then y, then z,
 * 							and then x again and place them in that order into @data
 *
 */
struct dr_sens {
	// functions
	int (*sens_init)(struct dr_sens *sens);
	void (*sens_close)(struct dr_sens *sens);
	int (*read)(struct dr_sens *sens, double *data, int count);
	int (*write)(struct dr_sens *sens, const double *data, int count);
	int (*ping)(struct dr_sens *sens);
	int (*configure)(struct dr_sens *sens, const uint8_t *data, int count);

	enum dr_sens_type type;
	enum dr_sens_flags flags;
	uint8_t *name[NAMELEN];
	int hw_id;
	
	enum dr_bus_type bus_type;
	unsigned int bus_num;
	uint16_t address;
	int active;
	unsigned int max_read_rate;
	unsigned int max_write_rate;
};

/**
 * used to name a dr_sens struct
 * @name			a null terminated string containing the desired name
 *
 * @return			0 on success and -1 on failure
 */
int name_dr_sens(const char *name);

/**
 * registers *sens with the sensor manager
 * @sens				the sensor to be registered
 *
 * @return			0 if successful
 */
int register_sensor(struct dr_sens *sens);

/**
 * unregisters *sens with the sensor manager
 * @sens				the sensors to be unregistered
 */
void unregister_sensor(struct dr_sens *sens);

/**
 * reads from all sensors of <type> and places the output in data
 * @type			the type of sensor from which to read data
 * @data			an array to hold the data
 * @count			number of bytes to read
 *
 * @return			the number of bytes read
 */
int read_from_sensors(enum dr_sens_type type, uint8_t *data, int count);

/**
 * if sens is null, writes <data> to all sensors that match <type>
 * else writes data to *sens
 * @type			the type of sensor to which to write data
 * @sens				an optional pointer to a sensor to allow specific data writes
 * @data			the data to be written
 * @count			number of bytes to write from data to the sensor(s)
 *
 * @return the number of bytes written
 */
int write_to_sensor(enum dr_sens_type type, struct dr_sens *sens, const uint8_t *data, int count);



#endif
