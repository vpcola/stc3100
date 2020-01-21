#ifndef _STC3100_H_
#define _STC3100_H_

#ifdef __cpluplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "driver/i2c.h"

esp_err_t stc3100_init(i2c_port_t i2cnum);
esp_err_t stc3100_read_device_id(i2c_port_t i2cnum, uint8_t * devid, size_t len); 
esp_err_t stc3100_get_battery_voltage(i2c_port_t i2cnum, float * battvoltage);
esp_err_t stc3100_get_battery_current(i2c_port_t i2cnum, float * battcurrent);
esp_err_t stc3100_get_battery_rem_charge(i2c_port_t i2cnum, float * battremchrg);
esp_err_t stc3100_get_battery_temperature(i2c_port_t i2cnum, float * batttemp);

#ifdef __cpluplus
}
#endif

#endif
