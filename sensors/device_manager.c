// implementation of device manager
// Mark Hill

#include<unistd.h>
#include<stdlib.h>
#include<stdio.h>
#include<stdint.h>
#include<string.h>

#include<device_manager.h>
#include<dynamic_set.h>
#include<string_additions.h>


/* success if non-null string was copied */
int8_t name_dr_dev(struct dr_dev *dev, const char *name) {
	return (strlcpy(dev->name, name, NAME_LEN - 1)) ? 0 : 1;
}

int8_t register_device(struct dr_dev *dev) {
	
}

