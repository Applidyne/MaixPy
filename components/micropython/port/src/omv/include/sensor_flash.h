/**
 * \file    sensor_flash.h
 *
 * \brief   Device driver for sensor IR flash control
 *
 * \author  Marco Hess <marcoh@applidyne.com.au>
 *
 * \date    28/10/2021
 */

/* -------------------------------------------------------------------------- */

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */

#include "sipeed_i2c.h"

/* -------------------------------------------------------------------------- */

typedef struct
{
    i2c_device_number_t   i2c;                  /* I2C device number */
    uint32_t              i2c_freq;             /* I2C device frequency */
    uint8_t               sclk;                 /* I2C clock pin */
    uint8_t               sda;                  /* I2C data pin */
    uint8_t               gpio_torch;           /* GPIO to active TORCH mode */
    uint8_t               gpio_enable;          /* GPIO to activate ENABLE line */
    uint8_t               gpio_ambient_power;   /* GPIO to power ambient sensor */
}  sensor_flash_config_t;

typedef struct
{
    sensor_flash_config_t config;               /* Config paramters. */
    bool                  enable;               /* Enable strobe from sensor */
    bool                  torch;                /* Torch mode ON/OFF */
    uint16_t              current;              /* LED current setting */
} sensor_flash_t;

/* -------------------------------------------------------------------------- */

extern sensor_flash_t sensor_flash;

/* -------------------------------------------------------------------------- */

void sensor_flash_load_config( sensor_flash_config_t * sensor_flash_cfg );

/* -------------------------------------------------------------------------- */

int sensor_flash_reset( void );

/* -------------------------------------------------------------------------- */

int sensor_flash_enable( int enable );

/* -------------------------------------------------------------------------- */

int sensor_flash_torch( int enable );

/* -------------------------------------------------------------------------- */

int sensor_flash_set_current( int current );

/* -------------------------------------------------------------------------- */

int sensor_flash_get_current( void );

/* -------------------------------------------------------------------------- */

int sensor_flash_get_ambient( void );

/* -------------------------------------------------------------------------- */

