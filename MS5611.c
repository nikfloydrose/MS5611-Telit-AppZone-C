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
 
#include "MS5611.h"

#include "m2m_i2c_api.h"
#include "math.h"

typedef unsigned char UINT8;

/* I2C address */
#ifdef CSB_LOW
#define I2C_ADDR 0xEE
#else
#define I2C_ADDR 0xEF
#endif

/* PROM parameters */
#define PROM_ADDR 0xA0
#define NUM_PROM_REGS 8

/* ADC commands */
#define CMD_RESET    0x1E
#define CMD_ADC_D1   0x40
#define CMD_ADC_D2   0x50
#define CMD_OSR_256  0x00
#define CMD_OSR_512  0x02
#define CMD_OSR_1024 0x04
#define CMD_OSR_2048 0x06
#define CMD_OSR_4096 0x08

#define DELAY_RESET 3 /* ms */
#define DELAY_WAIT 1000 /* ms */
#define DELAY_OSR_4096 9040 /* us */
#define DELAY_OSR_2048 4540 /* us */
#define DELAY_OSR_1024 2280 /* us */
#define DELAY_OSR_512  1170 /* us */
#define DELAY_OSR_256   600 /* us */
#define ROUND_US_TO_MS(delay) (delay/1000 + 1)

#define LOW_TEMPERATURE 2000 /* 20 °C */
#define VERY_LOW_TEMPERATURE -1500 /* -15 °C */

static UINT8 CMD_OSR;
static UINT16 DELAY_OSR;
static UINT16 C[NUM_PROM_REGS];  /* calibration coefficients */
static UINT32 D1; /* raw pressure */
static UINT32 D2; /* raw temperature */
static FLOAT32 dT;
static FLOAT32 OFF, OFF2;
static FLOAT32 SENS, SENS2;
static FLOAT32 TEMP, T2;
static FLOAT32 P;


static MS6511_RESULT MS5611GetADC(UINT8 commandADC, UINT32 *conversionData);
static MS6511_RESULT MS5611ReadPROM(UINT16 *coeff);
static UINT8 crc4(UINT16 n_prom[]);


MS6511_RESULT MS5611Init(UINT8 i2cSda, UINT8 i2cScl, MS5611_OSR osr)
{
  UINT8 crc;
  MS6511_RESULT res;
  M2M_T_HW_I2C_RESULT ret;

  ret = m2m_hw_i2c_conf(i2cSda, i2cScl);
  if(ret != M2M_HW_I2C_RESULT_SUCCESS) return MS5611_I2C_ERROR;

  res = MS5611SetOSR(osr);
  if(res != MS5611_RESULT_SUCCESS) return MS5611_BAD_OSR;

  res = MS5611Reset();
  if(res != MS5611_RESULT_SUCCESS) return MS5611_I2C_ERROR;

  res = MS5611ReadPROM(C);
  if(res != MS5611_RESULT_SUCCESS) return MS5611_I2C_ERROR;

  crc = crc4(C);
  if (crc != (C[NUM_PROM_REGS - 1] & 0x0F)) return MS5611_BAD_CRC;

  return MS5611_RESULT_SUCCESS;
}


MS6511_RESULT MS5611GetData(MS5611_DATA *sensorData)
{
  MS6511_RESULT res;

  /* get temperature */
  res = MS5611GetADC((CMD_ADC_D2 | CMD_OSR), &D2);
  if(res != MS5611_RESULT_SUCCESS) return MS5611_I2C_ERROR;
  dT = D2 - C[5] * pow(2, 8);
  TEMP = 2000 + dT * C[6] / pow(2, 23);
  
  /* second order temperature compensation */
  if(TEMP < LOW_TEMPERATURE)
  {
    T2 = pow(dT, 2) / pow(2, 31);
    OFF2 = 5 * pow(TEMP - 2000, 2) / 2;
    SENS2 = 5 * pow(TEMP - 2000, 2) / 4;

    if(TEMP < VERY_LOW_TEMPERATURE)
    {
      OFF2 += 7 * pow(TEMP + 1500, 2);
      SENS2 += 11 * pow(TEMP + 1500, 2) / 2;
    }
  }
  else
  {
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;
  }
  TEMP -= T2;

  /* get pressure */
  res = MS5611GetADC((CMD_ADC_D1 | CMD_OSR), &D1);
  if(res != MS5611_RESULT_SUCCESS) return MS5611_I2C_ERROR;
  OFF = C[2] * pow(2, 16) + C[4] * dT / pow(2, 7) - OFF2;
  SENS = C[1] * pow(2, 15) + C[3] * dT / pow(2, 8) - SENS2;
  P = (D1 * SENS / pow(2, 21) - OFF) / pow(2, 15);

  sensorData->T = TEMP / 100;
  sensorData->P = P / 100;
  
  return res;
}


MS6511_RESULT MS5611SetOSR(MS5611_OSR osr)
{
  switch(osr)
  {
    case OSR_4096:
      CMD_OSR = CMD_OSR_4096;
      DELAY_OSR = DELAY_OSR_4096;
      break;

    case OSR_2048:
      CMD_OSR = CMD_OSR_2048;
      DELAY_OSR = DELAY_OSR_2048;
      break;

    case OSR_1024:
      CMD_OSR = CMD_OSR_1024;
      DELAY_OSR = DELAY_OSR_1024;
      break;

    case OSR_512:
      CMD_OSR = CMD_OSR_512;
      DELAY_OSR = DELAY_OSR_512;
      break;

    case OSR_256:
      CMD_OSR = CMD_OSR_256;
      DELAY_OSR = DELAY_OSR_256;
      break;

    default:
      return MS5611_BAD_OSR;
  }

  return MS5611_RESULT_SUCCESS;
}


MS6511_RESULT MS5611Reset(void)
{
  UINT8 cmd = CMD_RESET;
  UINT8 trials = 0;
  M2M_T_HW_I2C_RESULT ret = MS5611_I2C_ERROR;

  while((ret != M2M_HW_I2C_RESULT_SUCCESS) && (trials++ < 10))
  {
    ret = m2m_hw_i2c_cmb_format(I2C_ADDR, &cmd, 1, 0);
  }
  m2m_os_sleep_ms(DELAY_RESET);

  return ret;
}


static MS6511_RESULT MS5611GetADC(UINT8 commandADC, UINT32 *conversionData)
{
  UINT8 data[] = {0, 0, 0};
  UINT8 command = commandADC;
  M2M_T_HW_I2C_RESULT ret;

  ret = m2m_hw_i2c_cmb_format(I2C_ADDR, &command, 1, 0);
  if(ret != M2M_HW_I2C_RESULT_SUCCESS) return MS5611_I2C_ERROR;

  m2m_os_sleep_ms(ROUND_US_TO_MS(DELAY_OSR));

  ret = m2m_hw_i2c_cmb_format(I2C_ADDR, data, 1, 3);
  if(ret != M2M_HW_I2C_RESULT_SUCCESS) return MS5611_I2C_ERROR;

  *conversionData = ((UINT32)data[0] << 16 | (UINT16)data[1] << 8 | data[2]);

  return MS5611_RESULT_SUCCESS;
}


static MS6511_RESULT MS5611ReadPROM(UINT16 *coeff)
{
  UINT8 reg;
  UINT8 data[] = {0, 0};
  M2M_T_HW_I2C_RESULT ret;
  MS6511_RESULT res;

  for (reg = 0; reg < NUM_PROM_REGS; reg++)
  {
    data[0] = (reg * 2 + PROM_ADDR);
    ret = m2m_hw_i2c_cmb_format(I2C_ADDR, data, 1, 2);
    if(res != M2M_HW_I2C_RESULT_SUCCESS) return MS5611_I2C_ERROR;
    *coeff++ = ((UINT16)data[0] << 8 | data[1]);
  }

  return MS5611_RESULT_SUCCESS;
}


/* function taken from AN520_004 applicno1629 ECN1531 */
static UINT8 crc4(UINT16 n_prom[])
{
  INT16 cnt;
  UINT16 n_rem;
  UINT16 crc_read;
  UINT16 n_bit;
  
  n_rem = 0x00;
  crc_read = n_prom[7];
  n_prom[7] = (0xFF00 & (n_prom[7]));
  
  for (cnt = 0; cnt < 16; cnt++)
  {
    if (cnt % 2 == 1) n_rem ^= (UINT8)((n_prom[cnt >> 1]) & 0x00FF);
    else n_rem ^= (UINT8)(n_prom[cnt >> 1] >> 8);
    for (n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & (0x8000))
        n_rem = (n_rem << 1) ^ 0x3000;
      else
        n_rem = (n_rem << 1);
    }
  }
  
  n_rem =  (0x000F & (n_rem >> 12));
  n_prom[7] = crc_read;
  
  return (n_rem ^ 0x00);
}
