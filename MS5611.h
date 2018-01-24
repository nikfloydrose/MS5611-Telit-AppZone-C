/*
 * MS5611-01BA03 sensor driver for Telit AppZone C.
 * 
 * Copyright (C) 2018  Nicola Simoni <nikfloydrose@hotmail.it>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
 
#ifndef __MS5611_H
#define __MS5611_H

#include "m2m_type.h"

#define CSB_LOW /* CSB wired to GND. Undef if CSB is wired to Vcc */

/* oversampling ratios */
typedef enum {
  OSR_4096,
  OSR_2048,
  OSR_1024,
  OSR_512,
  OSR_256
} MS5611_OSR;

/* MS5611 result definition */
typedef enum {
  MS5611_RESULT_SUCCESS = 0,
  MS5611_I2C_ERROR,
  MS5611_BAD_OSR,
  MS5611_BAD_CRC
} MS6511_RESULT;

/* MS5611 data */
typedef struct {
  FLOAT32 T;
  FLOAT32 P;
} MS5611_DATA;

/* Description: initialize the MS5611 sensor
 * Parameters:
 *  i2cSda: I2C SDA pin
 *  i2cScl: I2C SCL pin
 * 	osr: oversampling ratio
 * Return value: refer to MS6511_RESULT enum */
MS6511_RESULT MS5611Init(UINT8 i2cSda, UINT8 i2cScl, MS5611_OSR osr);

/* Description: get pressure and temperature data from the MS5611 sensor
 * Parameter: pointer to data
 * Return value: refer to MS6511_RESULT enum */
MS6511_RESULT MS5611GetData(MS5611_DATA *sensorData);

/* Description: change the oversampling ratio (after sensor initialization)
 * Parameter: oversampling ratio
 * Return value: refer to MS6511_RESULT enum */
MS6511_RESULT MS5611SetOSR(MS5611_OSR osr);

/* Description: send reset signal to the MS5611 sensor
 * Parameter: none
 * Return value: refer to MS6511_RESULT enum */
MS6511_RESULT MS5611Reset(void);

#endif /* __MS5611_H */
