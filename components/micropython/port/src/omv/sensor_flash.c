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
    .sclk                = 30,
    .sda                 = 31,
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

int sensor_flash_init( void )
{
    mp_printf(&mp_plat_print, "[sensor_flash]: init\n");

    /* Load config */
    sensor_flash.config = sensor_flash_config_defaults;
    sensor_flash_load_config( &sensor_flash.config );

    if( ( sensor_flash.config.i2c > 2 )
        ||
        ( sensor_flash.config.i2c < 0 ) )
    {
        return -1;
    }

    /* Init ENABLE pin */
    fpioa_set_function( sensor_flash.config.gpio_enable,
                        FUNC_GPIOHS0 + sensor_flash.config.gpio_enable);
    gpiohs_set_drive_mode( sensor_flash.config.gpio_enable,
                           GPIO_DM_OUTPUT );
    /* ENABLE low to RESET */
    gpiohs_set_pin( sensor_flash.config.gpio_enable, GPIO_PV_LOW );

    /* Init I2C pins */
    fpioa_set_function( sensor_flash.config.sclk,
                        FUNC_I2C0_SCLK + ( sensor_flash.config.i2c * 2 ) );
    fpioa_set_function( sensor_flash.config.sda,
                        FUNC_I2C0_SDA  + ( sensor_flash.config.i2c * 2 ) );

    /* Init I2C device */
    maix_i2c_init( sensor_flash.config.i2c,
                   7, /* Address width */
                   sensor_flash.config.i2c_freq );

    /* ENABLE high to allow I2C access */
    gpiohs_set_pin( sensor_flash.config.gpio_enable, GPIO_PV_HIGH );

    /* Init TORCH enable pin */
    fpioa_set_function( sensor_flash.config.gpio_torch,
                        FUNC_GPIOHS0 + sensor_flash.config.gpio_torch);
    gpiohs_set_drive_mode( sensor_flash.config.gpio_torch,
                           GPIO_DM_OUTPUT );

    /* Init AMBIENT sensor power pin */
    fpioa_set_function( sensor_flash.config.gpio_enable,
                        FUNC_GPIOHS0 + sensor_flash.config.gpio_enable);
    gpiohs_set_drive_mode( sensor_flash.config.gpio_enable,
                           GPIO_DM_OUTPUT );

    mp_printf( &mp_plat_print,
               "[sensor_flash]: sensor_flash config i2c %d freq %lu sclk %d sda %d torch %d enable %d ambient %d\n",
               sensor_flash.config.i2c,
               sensor_flash.config.i2c_freq,
               sensor_flash.config.sclk,
               sensor_flash.config.sda,
               sensor_flash.config.gpio_torch,
               sensor_flash.config.gpio_enable,
               sensor_flash.config.gpio_ambient_power );

    /* Check chip is there */
    if( !adp1650_detect( sensor_flash.config.i2c ) )
    {
        mp_printf( &mp_plat_print,
                   "[sensor_flash]: adp1650 not detected!\n" );
        return -1;
    }

    /* ENABLE low to RESET */
    gpiohs_set_pin( sensor_flash.config.gpio_enable, GPIO_PV_LOW );
    return 0;
}

/* -------------------------------------------------------------------------- */

/**  Set EN low to bring the quiescent current (IQ) to <1 μA.
 *   Registers are set to their defaults when EN is brought from low to high.
 */

int sensor_flash_enable( int enable )
{
    sensor_flash.enable = (enable > 0);

    mp_printf( &mp_plat_print,
               "[sensor_flash]: enable %s\n",
               sensor_flash.enable ? "ON" : "OFF");

    if( sensor_flash.enable )
    {
        /* Enable chip. */
        gpiohs_set_pin( sensor_flash.config.gpio_enable, GPIO_PV_HIGH );

        /* terrible implementation of a delay. */
        uint32_t ms = 1;
        while (ms && ms--)
        {
            for (uint32_t i = 0; i < 25000; i++)
                __asm__ __volatile__("nop");
        }

        /* Check chip is there */
        if( !adp1650_detect( sensor_flash.config.i2c ) )
        {
            mp_printf( &mp_plat_print,
                    "[sensor_flash]: adp1650 not detected!\n" );
            return -1;
        }

        /* Default TORCH off */
        sensor_flash_torch( false );

        /* Configure defaults */
        adp1650_init( sensor_flash.config.i2c );

        sensor_flash.current_mA = 300;
        adp1650_set_current( sensor_flash.config.i2c,
                            sensor_flash.current_mA );
    }
    else
    {
        /* Disable chip. */
        gpiohs_set_pin( sensor_flash.config.gpio_enable, GPIO_PV_LOW );
    }

    return 0;
}

/* -------------------------------------------------------------------------- */

/**  Sets the output enabled register bit
 */

int sensor_flash_output( int enable )
{
    bool output_type = (enable > 0);
    mp_printf( &mp_plat_print,
               "[sensor_flash]: output %s\n",
               output_type ? "ON" : "OFF" );

    adp1650_set_output( sensor_flash.config.i2c,
                        output_type );
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
        gpiohs_set_pin( sensor_flash.config.gpio_torch, GPIO_PV_HIGH );
    }
    else
    {
        gpiohs_set_pin( sensor_flash.config.gpio_torch, GPIO_PV_LOW );
    }

    return 0;
}

/* -------------------------------------------------------------------------- */

int sensor_flash_set_current( int current_mA )
{
    /* Write current to chip */
    mp_printf( &mp_plat_print,
               "[sensor_flash]: set current %d\n", current_mA );
    sensor_flash.current_mA = current_mA;
    return adp1650_set_current( sensor_flash.config.i2c,
                                sensor_flash.current_mA );
}

/* -------------------------------------------------------------------------- */

int sensor_flash_get_current( void )
{
    /* Read set current from chip */
    mp_printf( &mp_plat_print,
               "[sensor_flash]: get current %d\n", sensor_flash.current_mA );
    return sensor_flash.current_mA;
}

/* -------------------------------------------------------------------------- */

/* Read ambient sensor ADC via chip ADC external input */

int sensor_flash_get_ambient( void )
{
    /* TODO: Check if it OK to power ON/OFF the sensor around this call */
    int ambient = adp1650_get_adc( sensor_flash.config.i2c,
                                   ADC_CHANNEL_EXT_VOLTAGE );

    mp_printf( &mp_plat_print,
               "[sensor_flash]: get ambient %d\n", ambient );

    return ambient;
}

/* -------------------------------------------------------------------------- */

/** Read chip die temperature */

int sensor_flash_get_temperature( void )
{
    /* TODO: Check if it OK to power ON/OFF the sensor around this call */

    int adc = adp1650_get_adc( sensor_flash.config.i2c,
                               ADC_CHANNEL_TEMPERATURE );

    /* We get a 0..15 reading that is mapped 25 to 150 degrees with
     * 0 being 150 degrees and 15 being 25 degrees.
     */
    int temperature = ( ( 16 - adc ) * ( ( 150 - 25 ) / 15 ) ) + 25;

    mp_printf( &mp_plat_print,
               "[sensor_flash]: get temperature %d\n", temperature );

    return temperature;
}

/* -------------------------------------------------------------------------- */

/** Read chip fault status register. See ADP1650 reg 0x05 for details */

int sensor_flash_get_fault( void )
{
    int status = adp1650_get_fault_status( sensor_flash.config.i2c );

    mp_printf( &mp_plat_print,
               "[sensor_flash]: fault status 0x%x\n", status );

    return status;
}

/* -------------------------------------------------------------------------- */

int sensor_flash_set_mode( int mode )
{
    int status = adp1650_set_mode( sensor_flash.config.i2c, mode );

    mp_printf( &mp_plat_print,
               "[sensor_flash]: mode set 0x%x ret 0x%x\n", mode, status );

    return status;
}

/* -------------------------------------------------------------------------- */