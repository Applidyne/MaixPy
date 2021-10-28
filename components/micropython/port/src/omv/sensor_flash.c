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

#include "sensor_flash.h"

/* There are likely too many includes here */
#include "mp.h"
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
    .i2c                 = I2C_DEVICE_0,
    .i2c_freq            = 125000UL,
    .sclk                = 31,
    .sda                 = 30,
    .gpio_torch          = 10,
    .gpio_enable         = 11,
    .gpio_ambient_power  = 12,
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

        SENSOR_FLASH_CHECK_CONFIG( i2c,         &sensor_flash_cfg->i2c );
        SENSOR_FLASH_CHECK_CONFIG( i2c_freq,    &sensor_flash_cfg->i2c_freq );
        SENSOR_FLASH_CHECK_CONFIG( sclk,        &sensor_flash_cfg->sclk );
        SENSOR_FLASH_CHECK_CONFIG( sda,         &sensor_flash_cfg->sda );
        SENSOR_FLASH_CHECK_CONFIG( gpio_torch,  &sensor_flash_cfg->gpio_torch );
        SENSOR_FLASH_CHECK_CONFIG( gpio_enable, &sensor_flash_cfg->gpio_enable );   }
}

/* -------------------------------------------------------------------------- */

int sensor_flash_reset( void )
{
    mp_printf(&mp_plat_print, "[sensor_flash]: reset\n");

    /* Load config */
    sensor_flash.config = sensor_flash_config_defaults;
    sensor_flash_load_config( &sensor_flash.config );

    if( ( sensor_flash.config.i2c > 2 )
        ||
        ( sensor_flash.config.i2c < 0 ) )
    {
        return -1;
    }

    /* Init I2C pins */
    fpioa_set_function( sensor_flash.config.sclk,
                        FUNC_I2C0_SCLK + ( sensor_flash.config.i2c * 2 ) );
    fpioa_set_function( sensor_flash.config.sda,
                        FUNC_I2C0_SDA  + ( sensor_flash.config.i2c * 2 ) );

    fpioa_set_function( sensor_flash.config.gpio_torch,
                        FUNC_GPIOHS0 + sensor_flash.config.gpio_torch);
    gpiohs_set_drive_mode( sensor_flash.config.gpio_torch,
                           GPIO_DM_OUTPUT );
    gpiohs_set_pin( sensor_flash.config.gpio_torch,
                    GPIO_PV_LOW );

    /* Init ENABLE pin */
    fpioa_set_function( sensor_flash.config.gpio_enable,
                        FUNC_GPIOHS0 + sensor_flash.config.gpio_enable);
    gpiohs_set_drive_mode( sensor_flash.config.gpio_enable,
                           GPIO_DM_OUTPUT );
    gpiohs_set_pin( sensor_flash.config.gpio_enable,
                    GPIO_PV_LOW );

    /* Init AMBIENT sensor power pin */
    fpioa_set_function( sensor_flash.config.gpio_enable,
                        FUNC_GPIOHS0 + sensor_flash.config.gpio_enable);
    gpiohs_set_drive_mode( sensor_flash.config.gpio_enable,
                           GPIO_DM_OUTPUT );
    gpiohs_set_pin( sensor_flash.config.gpio_enable,
                    GPIO_PV_HIGH ); /* Default powered ON */

    mp_printf( &mp_plat_print,
               "[sensor_flash]: sensor_flash config i2c %d freq %lu sclk %d sda %d torch %d enable %d ambient %d\n",
               sensor_flash.config.i2c,
               sensor_flash.config.i2c_freq,
               sensor_flash.config.sclk,
               sensor_flash.config.sda,
               sensor_flash.config.gpio_torch,
               sensor_flash.config.gpio_enable,
               sensor_flash.config.gpio_ambient_power );

    /* Init I2C device */
    maix_i2c_init( sensor_flash.config.i2c,
                   7, /* Address width */
                   sensor_flash.config.i2c_freq );

    /* Check chip is there */

    /* Default disable */

    /* Set defaults */
    sensor_flash.current = 10;
    sensor_flash_torch( false );
    sensor_flash_enable( false );
    return 0;
}

/* -------------------------------------------------------------------------- */

int sensor_flash_enable( int enable )
{
    sensor_flash.enable = (enable > 0);

    mp_printf( &mp_plat_print,
               "[sensor_flash]: enable %s\n",
               sensor_flash.enable ? "ON" : "OFF");

    if( sensor_flash.enable )
    {
        gpiohs_set_pin( sensor_flash.config.gpio_enable, GPIO_PV_HIGH );
    }
    else
    {
        gpiohs_set_pin( sensor_flash.config.gpio_enable, GPIO_PV_LOW );
    }

    return 0;
}

/* -------------------------------------------------------------------------- */

int sensor_flash_torch( int enable )
{
    sensor_flash.torch = (enable > 0);

    mp_printf( &mp_plat_print,
               "[sensor_flash]: torch %s\n",
               sensor_flash.torch ? "ON" : "OFF");

    if( sensor_flash.torch )
    {
        gpiohs_set_pin( sensor_flash.config.gpio_enable, GPIO_PV_HIGH );
    }
    else
    {
        gpiohs_set_pin( sensor_flash.config.gpio_enable, GPIO_PV_LOW );
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

