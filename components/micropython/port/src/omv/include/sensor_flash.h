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

typedef struct
{
    uint8_t sensor_flash_i2c_num;
    uint8_t sensor_flash_i2c_clk;
    uint8_t sensor_flash_i2c_sda;
    uint8_t sensor_flash_gpio_torch;
    uint8_t sensor_flash_gpio_enable;
}  sensor_flash_config_t;

typedef struct
{
    sensor_flash_config_t   config;
    uint8_t                 i2c_num;
    bool                    enable;
    uint16_t                current;
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

int sensor_flash_set_current( int current );

/* -------------------------------------------------------------------------- */

int sensor_flash_get_current( void );

/* -------------------------------------------------------------------------- */

int sensor_flash_get_ambient( void );

/* -------------------------------------------------------------------------- */

