/**
 * \file    sensor_flash.c
 *
 * \brief   Device driver for sensor IR flash control
 *
 * \author  Marco Hess <marcoh@applidyne.com.au>
 *
 * \date    28/10/2021
 */

/* -------------------------------------------------------------------------- */

#include <stdlib.h>
#include <string.h>

/* -------------------------------------------------------------------------- */

/* There are likely too many includes here */
#include "mp.h"
#include "cambus.h"
#include "sensor_flash.h"
#include "omv_boardconfig.h"
#include "mphalport.h"
#include "plic.h"
#include "fpioa.h"
#include "syslog.h"
#include "mphalport.h"
#include "Maix_config.h"
#include "gpiohs.h"
#include "adp1650.h"

/* -------------------------------------------------------------------------- */

static const sensor_flash_config_t sensor_flash_config_defaults =
{
    .sensor_flash_i2c_num     = -1,
    .sensor_flash_i2c_clk     = 31,
    .sensor_flash_i2c_sda     = 30,
    .sensor_flash_gpio_torch  = 10,
    .sensor_flash_gpio_enable = 11,
};

sensor_flash_t sensor_flash = {0};

/* -------------------------------------------------------------------------- */

#define SENSOR_FLASH_CHECK_CONFIG(GOAL, val)                                                                         \
    {                                                                                                         \
        const char key[] = #GOAL;                                                                             \
        mp_map_elem_t *elem = mp_map_lookup(&self->map, mp_obj_new_str(key, sizeof(key) - 1), MP_MAP_LOOKUP); \
        if (elem != NULL)                                                                                     \
        {                                                                                                     \
            *(val) = mp_obj_get_int(elem->value);                                                             \
        }                                                                                                     \
    }

/* -------------------------------------------------------------------------- */

void sensor_flash_load_config( sensor_flash_config_t * sensor_flash_cfg )
{
    const char cfg[] = "sensor_flash";

    mp_obj_t tmp = maix_config_get_value( mp_obj_new_str( cfg, sizeof(cfg) - 1), mp_const_none );

    if (tmp != mp_const_none && mp_obj_is_type(tmp, &mp_type_dict))
    {
        mp_obj_dict_t *self = MP_OBJ_TO_PTR(tmp);

        SENSOR_FLASH_CHECK_CONFIG( sensor_flash_i2c_num,     &sensor_flash_cfg->sensor_flash_i2c_num );
        SENSOR_FLASH_CHECK_CONFIG( sensor_flash_i2c_clk,     &sensor_flash_cfg->sensor_flash_i2c_clk );
        SENSOR_FLASH_CHECK_CONFIG( sensor_flash_i2c_sda,     &sensor_flash_cfg->sensor_flash_i2c_sda );
        SENSOR_FLASH_CHECK_CONFIG( sensor_flash_gpio_torch,  &sensor_flash_cfg->sensor_flash_gpio_torch );
        SENSOR_FLASH_CHECK_CONFIG( sensor_flash_gpio_enable, &sensor_flash_cfg->sensor_flash_gpio_enable );   }
}

/* -------------------------------------------------------------------------- */

int sensor_flash_reset( void )
{
    mp_printf(&mp_plat_print, "[sensor_flash]: reset\n");

    /* Load config */
    sensor_flash_config_t config = sensor_flash_config_defaults;
    sensor_flash_load_config( &config );

    /* Enable IO pins */

    /* Enable I2C */

    /* Check chip is there */

    /* Default disable */

    /* Set default current */
    sensor_flash.current = 10;
    return 0;
}

/* -------------------------------------------------------------------------- */

int sensor_flash_enable( int enable )
{
    mp_printf( &mp_plat_print,
               "[sensor_flash]: enable %s\n",
               enable ? "ON" : "OFF");
    if( enable )
    {
        /* Toggle IO */
    }
    else
    {
        /* Toggle IO */
    }

    return 0;
}

/* -------------------------------------------------------------------------- */

int sensor_flash_set_current( int current )
{
    /* Write current to chip */
    mp_printf( &mp_plat_print,
               "[sensor_flash]: set current %d\n", current );
    sensor_flash.current = current;
    return 0;
}

/* -------------------------------------------------------------------------- */

int sensor_flash_get_current( void )
{
    /* Read set current from chip */
    mp_printf( &mp_plat_print,
               "[sensor_flash]: get current %d\n", sensor_flash.current );
    return sensor_flash.current;
}

/* -------------------------------------------------------------------------- */

int sensor_flash_get_ambient( void )
{
    /* Read ambient ADC from chip */
    int ambient = sensor_flash.current / 2; /* TEST */

    mp_printf( &mp_plat_print,
               "[sensor_flash]: get ambient %d\n", ambient );

    return ambient;
}

/* -------------------------------------------------------------------------- */

