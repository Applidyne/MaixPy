/**
 * \file    mt9v022.c
 *
 * \brief   Device driver for MT9V022 image sensor, adopted from
 *          mt9d111 device driver by Gabriel Mariano Marcelino
 *          <gabriel.mm8@gmail.com>
 *
 *          MT9V022-Driver is free software: you can redistribute it and/or
 *          modify it under the terms of the GNU Lesser General Public
 *          License as published by the Free Software Foundation, either
 *          version 3 of the License, or (at your option) any later version.
 *
 *          MT9V022-Driver is distributed in the hope that it will be useful,
 *          but WITHOUT ANY WARRANTY; without even the implied warranty of
 *          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *          GNU Lesser General Public License for more details.
 *
 *          You should have received a copy of the GNU Lesser General Public
 *          License along with MT9V022-Driver.
 *          If not, see <http://www.gnu.org/licenses/>.
 *
 * \author  Marco Hess <marcoh@applidyne.com.au>
 *
 * \date    23/09/2021
 */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <stdio.h>
#include "mt9v022.h"
#include "dvp.h"
#include "plic.h"
#include "sleep.h"
#include "sensor.h"
#include "mphalport.h"
#include "cambus.h"
#include "printf.h"
#include "omv_boardconfig.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

// i2c_device_number_t components/micropython/port/src/omv/cambus.c
static i2c_device_number_t mt9v022_i2c_device        = -2;
static uint8_t             mt9v022_i2c_slave_address = 0;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/**
 * \brief Struct to store an register address and its value.
 */
typedef struct
{
    uint8_t     reg;        /**< register address                  */
    uint16_t    mask;       /**< if set, use in read-modify-write. */
    uint16_t    value;      /**< register value to set.            */
    uint16_t    wait_ms;    /**< time in ms to wait after write.   */
} Register_t;

/**
 * \brief Default registers values (Values after reset).
 */
static const Register_t mt9v022_reg_defaults[] =
{
    { .reg     = MT9V022_RESET,
      .mask    = 0,
      .value   = MT9V022_RESET_DIGITAL | MT9V022_RESET_AGC,
      .wait_ms = 2 },
    { .reg     = MT9V022_CHIP_CONTROL,
      .mask    = 0,
      .value   = MT9V022_CHIP_CONTROL_MASTER_MODE
                 | MT9V022_CHIP_CONTROL_DOUT_ENABLE
                 | MT9V022_CHIP_CONTROL_SEQUENTIAL,
      .wait_ms = 0 },
    { .reg     = MT9V022_MONITOR_MODE,
      .mask    = 0,
      .value   = MT9V022_MONITOR_MODE_DISABLE,
      .wait_ms = 0 },
    { .reg     = MT9V022_MONITOR_MODE_CAPTURE_CONTROL,
      .mask    = 0,
      .value   = MT9V022_MONITOR_MODE_CAPTURE_CONTROL_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_COLUMN_START,
      .mask    = 0,
      .value   = MT9V022_COLUMN_START_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_ROW_START,
      .mask    = 0,
      .value   = MT9V022_ROW_START_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_WINDOW_HEIGHT,
      .mask    = 0,
      .value   = MT9V022_WINDOW_HEIGHT_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_WINDOW_WIDTH,
      .mask    = 0,
      .value   = MT9V022_WINDOW_WIDTH_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_HORIZONTAL_BLANKING,
      .mask    = 0,
      .value   = MT9V022_HORIZONTAL_BLANKING_BIN_1_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_VERTICAL_BLANKING,
      .mask    = 0,
      .value   = MT9V022_VERTICAL_BLANKING_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_AEC_AGC_ENABLE,
      .mask    = 0,
      .value   = MT9V022_AEC_ENABLE | MT9V022_AGC_ENABLE,
      .wait_ms = 0 },
    { .reg     = MT9V022_ADC_MODE_CONTROL,
      .mask    = 0,
      .value   = MT9V022_ADC_MODE_CONTROL_LOW_LIGHT,
      .wait_ms = 0 },
    { .reg     = MT9V022_ANALOG_GAIN,
      .mask    = 0,
      .value   = MT9V022_ANALOG_GAIN_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_TOTAL_SHUTTER_WIDTH,
      .mask    = 0,
      .value   = MT9V022_TOTAL_SHUTTER_WIDTH_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_BLACK_LEVEL_CALIB_CTRL,
      .mask    = 0,
      .value   = 1,
      .wait_ms = 0 },
    { .reg     = MT9V022_TEST_PATTERN,
      .mask    = 0,
      .value   = MT9V022_TEST_PATTERN_NONE,
      .wait_ms = 0 },
    { .reg     = MT9V022_AEGC_DESIRED_BIN,
      .mask    = 0,
      .value   = MT9V022_AEGC_DESIRED_BIN_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_AEC_LPF,
      .mask    = 0,
      .value   = MT9V022_AEC_LPF_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_AGC_LPF,
      .mask    = 0,
      .value   = MT9V022_AGC_LPF_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_AEC_UPDATE_FREQUENCY,
      .mask    = 0,
      .value   = MT9V022_AEC_UPDATE_FREQUENCY_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_AGC_UPDATE_FREQUENCY,
      .mask    = 0,
      .value   = MT9V022_AGC_UPDATE_FREQUENCY_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_AEC_MAX_SHUTTER_WIDTH,
      .mask    = 0,
      .value   = MT9V022_AEC_MAX_SHUTTER_WIDTH_DEF,
      .wait_ms = 0 },

    /* Table End Marker */
    { 0, 0, 0, 0 }
};

/**
 * \brief Registers settings for a QQVGA mode.
 */
static const Register_t mt9v022_reg_framesize_QQVGA[] =
{
    { .reg     = MT9V022_COLUMN_START,
      .mask    = 0,
      .value   = 56 / 4,
      .wait_ms = 0 },
    { .reg     = MT9V022_ROW_START,
      .mask    = 0,
      .value   = MT9V022_ROW_START_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_WINDOW_HEIGHT,
      .mask    = 0,
      .value   = MT9V022_WINDOW_HEIGHT_DEF / 4,
      .wait_ms = 0 },
    { .reg     = MT9V022_WINDOW_WIDTH,
      .mask    = 0,
      .value   = 640 / 4,
      .wait_ms = 0 },
    { .reg     = MT9V022_HORIZONTAL_BLANKING,
      .mask    = 0,
      .value   = MT9V022_HORIZONTAL_BLANKING_BIN_4_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_VERTICAL_BLANKING,
      .mask    = 0,
      .value   = MT9V022_VERTICAL_BLANKING_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_READ_MODE,
      .mask    = MT9V022_READ_MODE_ROW_BIN_MASK | MT9V022_READ_MODE_COLUMN_BIN_MASK,
      .value   = (2 << MT9V022_READ_MODE_ROW_BIN_SHIFT) | (2 << MT9V022_READ_MODE_COLUMN_BIN_SHIFT),
      .wait_ms = 0 },

    /* Table End Marker */
    { 0, 0, 0, 0 }
};

/**
 * \brief Registers settings for a QVGA mode.
 */
static const Register_t mt9v022_reg_framesize_QVGA[] =
{
    { .reg     = MT9V022_COLUMN_START,
      .mask    = 0,
      .value   = 56 / 2,
      .wait_ms = 0 },
    { .reg     = MT9V022_ROW_START,
      .mask    = 0,
      .value   = MT9V022_ROW_START_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_WINDOW_HEIGHT,
      .mask    = 0,
      .value   = MT9V022_WINDOW_HEIGHT_DEF / 2,
      .wait_ms = 0 },
    { .reg     = MT9V022_WINDOW_WIDTH,
      .mask    = 0,
      .value   = 640 / 2,
      .wait_ms = 0 },
    { .reg     = MT9V022_HORIZONTAL_BLANKING,
      .mask    = 0,
      .value   = MT9V022_HORIZONTAL_BLANKING_BIN_2_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_VERTICAL_BLANKING,
      .mask    = 0,
      .value   = MT9V022_VERTICAL_BLANKING_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_READ_MODE,
      .mask    = MT9V022_READ_MODE_ROW_BIN_MASK | MT9V022_READ_MODE_COLUMN_BIN_MASK,
      .value   = (1 << MT9V022_READ_MODE_ROW_BIN_SHIFT) | (1 << MT9V022_READ_MODE_COLUMN_BIN_SHIFT),
      .wait_ms = 0 },

    /* Table End Marker */
    { 0, 0, 0, 0 }
};

/**
 * \brief Registers settings for a VGA mode.
 */
static const Register_t mt9v022_reg_framesize_VGA[] =
{
    { .reg     =  MT9V022_COLUMN_START,
      .mask    = 0,
      .value   = 56,
      .wait_ms = 0 },
    { .reg     = MT9V022_ROW_START,
      .mask    = 0,
      .value   = MT9V022_ROW_START_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_WINDOW_HEIGHT,
      .mask    = 0,
      .value   = MT9V022_WINDOW_HEIGHT_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_WINDOW_WIDTH,
      .mask    = 0,
      .value   = 640,
      .wait_ms = 0 },
    { .reg     = MT9V022_HORIZONTAL_BLANKING,
      .mask    = 0,
      .value   = MT9V022_HORIZONTAL_BLANKING_BIN_1_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_VERTICAL_BLANKING,
      .mask    = 0,
      .value   = MT9V022_VERTICAL_BLANKING_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_READ_MODE,
      .mask    = MT9V022_READ_MODE_ROW_BIN_MASK | MT9V022_READ_MODE_COLUMN_BIN_MASK,
      .value   = (0 << MT9V022_READ_MODE_ROW_BIN_SHIFT) | (0 << MT9V022_READ_MODE_COLUMN_BIN_SHIFT),
      .wait_ms = 0 },

    /* Table End Marker */
    { 0, 0, 0, 0 }
};

/**
 * \brief Registers settings for a WVGA mode.
 */
static const Register_t mt9v022_reg_framesize_WVGA[] =
{
    { .reg     = MT9V022_COLUMN_START,
      .mask    = 0,
      .value   = MT9V022_COLUMN_START_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_ROW_START,
      .mask    = 0,
      .value   = MT9V022_ROW_START_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_WINDOW_HEIGHT,
      .mask    = 0,
      .value   = MT9V022_WINDOW_HEIGHT_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_WINDOW_WIDTH,
      .mask    = 0,
      .value   = MT9V022_WINDOW_WIDTH_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_HORIZONTAL_BLANKING,
      .mask    = 0,
      .value   = MT9V022_HORIZONTAL_BLANKING_BIN_1_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_VERTICAL_BLANKING,
      .mask    = 0,
      .value   = MT9V022_VERTICAL_BLANKING_DEF,
      .wait_ms = 0 },
    { .reg     = MT9V022_READ_MODE,
      .mask    = MT9V022_READ_MODE_ROW_BIN_MASK | MT9V022_READ_MODE_COLUMN_BIN_MASK,
      .value   = (0 << MT9V022_READ_MODE_ROW_BIN_SHIFT) | (0 << MT9V022_READ_MODE_COLUMN_BIN_SHIFT),
      .wait_ms = 0 },

    /* Table End Marker */
    { 0, 0, 0, 0 }
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_write( const int8_t reg, const uint16_t val )
{
    uint8_t tmp[3] = { 0 };
    tmp[0] = (uint8_t)(reg & 0xff);
    tmp[1] = (uint8_t)((val >> 8) & 0xff);
    tmp[2] = (uint8_t)(val & 0xff);
    int ret = maix_i2c_send_data( mt9v022_i2c_device,
                                  mt9v022_i2c_slave_address, tmp, 3, 20);
    return ret;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static uint16_t
mt9v022_read( const uint8_t reg )
{
    uint8_t buf[2] = { 0 };
    buf[0] = reg;
    maix_i2c_send_data( mt9v022_i2c_device,
                        mt9v022_i2c_slave_address, buf, 1, 20);
    int ret = maix_i2c_recv_data( mt9v022_i2c_device,
                                  mt9v022_i2c_slave_address, NULL, 0, buf, 2, 30);
    if (ret == 0)
    {
        return (buf[0] << 8) + buf[1];
    }
    return -1;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_write_registers( const Register_t * registers )
{
    uint16_t i = 0;
    while( registers[i].reg != 0 )
    {
        uint16_t value = registers[i].value;

        if( registers[i].mask )
        {
            /* If there is a mask specified, do a read-modify-write */
            value = mt9v022_read( registers[i].reg );

            /* Blank out the mask area and OR the new value into it */
            value = ( value & ~registers[i].mask )
                    | ( registers[i].value & registers[i].mask );
        }

        mt9v022_write( registers[i].reg, value );

        if( registers[i].wait_ms )
        {
            msleep( registers[i].wait_ms );
        }
        i++;
    }
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_reset( sensor_t * sensor )
{
    /* Reinit register configuration */
    mt9v022_write_registers( mt9v022_reg_defaults );
    mt9v022_write_registers( mt9v022_reg_framesize_WVGA );
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_sleep( sensor_t * sensor, int enable )
{
    printk("%s sensor %p\r\n", __func__, sensor);

    if(enable)
    {
        mt9v022_write( MT9V022_MONITOR_MODE,
                       MT9V022_MONITOR_MODE_ENABLE );
        mt9v022_write( MT9V022_MONITOR_MODE_CAPTURE_CONTROL,
                       MT9V022_MONITOR_MODE_CAPTURE_CONTROL_DEF );
    }
    else
    {
        mt9v022_write( MT9V022_MONITOR_MODE, MT9V022_MONITOR_MODE_DISABLE );
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_read_reg( sensor_t * sensor, uint8_t reg_addr)
{
    printk("%s sensor %p\r\n", __func__, sensor);
    return mt9v022_read( reg_addr );
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_write_reg( sensor_t * sensor, uint8_t reg_addr, uint16_t reg_data)
{
    printk("%s sensor %p\r\n", __func__, sensor);
    mt9v022_write( reg_addr, reg_data );
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_pixformat( sensor_t * sensor, pixformat_t pixformat )
{
    printk("%s sensor %p\r\n", __func__, sensor);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_framesize( sensor_t * sensor, framesize_t framesize )
{
    printk("%s sensor %p framesize %d\r\n", __func__, sensor, framesize);

    uint16_t width  = resolution[framesize][0];
    uint16_t height = resolution[framesize][1];

    /* Only some framesizes supported */
    switch( framesize )
    {
        case FRAMESIZE_QQVGA:
            mt9v022_write_registers( mt9v022_reg_framesize_QQVGA );
            break;

        case FRAMESIZE_QVGA:
            mt9v022_write_registers( mt9v022_reg_framesize_QVGA );
            break;

        case FRAMESIZE_VGA:
            mt9v022_write_registers( mt9v022_reg_framesize_VGA );
            break;

        case FRAMESIZE_WVGA:
            mt9v022_write_registers( mt9v022_reg_framesize_WVGA );
            break;

        default:
            return -1;  /* Return ERROR */
    }

    dvp_set_image_size( width, height );
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_framerate( sensor_t * sensor, framerate_t framerate )
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_contrast( sensor_t * sensor, int level )
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_brightness( sensor_t * sensor, int level )
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_saturation( sensor_t * sensor, int level )
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_gainceiling( sensor_t * sensor, gainceiling_t gainceiling )
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_quality( sensor_t * sensor, int qs)
{
    /* Relates to JPEG quality. Not supported on this sensor */
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_colorbar( sensor_t * sensor, int enable )
{
    uint16_t value = mt9v022_read( MT9V022_TEST_PATTERN );

    value = ( value & ~MT9V022_TEST_PATTERN_GRAY_VERTICAL );
    if( enable )
    {
        value |= MT9V022_TEST_PATTERN_GRAY_VERTICAL;
    }

    mt9v022_write( MT9V022_TEST_PATTERN, value );

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_auto_gain( sensor_t * sensor,
                       int        enable,
                       float      gain_db,
                       float      gain_db_ceiling)
{
    if( enable )
    {
        uint16_t value = mt9v022_read( MT9V022_AEC_AGC_ENABLE );
        value |= MT9V022_AGC_ENABLE;
        mt9v022_write( MT9V022_AEC_AGC_ENABLE, value );
    }
    else
    {
            /* The user wants to set gain manually, hope, she
               * knows, what she's doing... Switch AGC off.
               */
        uint16_t value = mt9v022_read( MT9V022_AEC_AGC_ENABLE );
        value &= ~MT9V022_AGC_ENABLE;
        mt9v022_write( MT9V022_AEC_AGC_ENABLE, value );

        /* TODO: Set gain DB & gain DB ceiling */
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_get_gain_db( sensor_t * sensor, float *gain_db )
{
    uint16_t value = mt9v022_read( MT9V022_TEST_PATTERN );

    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_auto_exposure( sensor_t * sensor, int enable, int exposure_us )
{
    if( enable )
    {
        uint16_t value = mt9v022_read( MT9V022_AEC_AGC_ENABLE );
        value |= MT9V022_AEC_ENABLE;
        mt9v022_write( MT9V022_AEC_AGC_ENABLE, value );
    }
    else
    {
        /* The user wants to set shutter width manually, hope,
               * she knows, what she's doing... Switch AEC off.
               */
        uint16_t value = mt9v022_read( MT9V022_AEC_AGC_ENABLE );
        value &= ~MT9V022_AEC_ENABLE;
        mt9v022_write( MT9V022_AEC_AGC_ENABLE, value );

        uint32_t shutter = mt9v022_read( MT9V022_TOTAL_SHUTTER_WIDTH );
        mt9v022_write( MT9V022_TOTAL_SHUTTER_WIDTH, exposure_us );
        printk("Shutter width from %lu to %lu\r\n", shutter, exposure_us );

        /* TODO: Check shutter width units and range and how it matches exposure_us */
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_get_exposure_us( sensor_t * sensor, int *exposure_us)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_auto_whitebal( sensor_t * sensor,
                           int        enable,
                           float      r_gain_db,
                           float      g_gain_db,
                           float      b_gain_db)
{
    /* Not used on monochrome sensor */
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_get_rgb_gain_db( sensor_t * sensor,
                         float    * r_gain_db,
                         float    * g_gain_db,
                         float    * b_gain_db)
{
    /* Not used on monochrome sensor */
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_hmirror( sensor_t * sensor, int enable)
{
    uint16_t value = mt9v022_read( MT9V022_READ_MODE );

    value = ( value & ~MT9V022_READ_MODE_COLUMN_FLIP );
    if( enable )
    {
        value |= MT9V022_READ_MODE_COLUMN_FLIP;
    }

    mt9v022_write( MT9V022_READ_MODE, value );

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_vflip( sensor_t * sensor, int enable )
{
    uint16_t value = mt9v022_read( MT9V022_READ_MODE );

    value = ( value & ~MT9V022_READ_MODE_ROW_FLIP );
    if( enable )
    {
        value |= MT9V022_READ_MODE_ROW_FLIP;
    }

    mt9v022_write( MT9V022_READ_MODE, value );

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_special_effect( sensor_t * sensor, sde_t sde )
{
    /* SDE_NORMAL, SDE_NEGATIVE */
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v022_set_lens_correction( sensor_t * sensor, int enable, int radi, int coef )
{
    /* Not used */
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mt9v022_set_windowing( framesize_t framesize, int x, int y, int w, int h )
{
    /* Set a subframe within the current framesize e.g. 224 x 224 */
    /* TODO */
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
      return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Return TRUE when an MT9V022 sensor is detected on the indicated bus */

static const uint8_t mt9v022_i2c_slave_addresses[] =
{
    MT9V022_CONFIG_I2C_ADDRESS_0,
    MT9V022_CONFIG_I2C_ADDRESS_1,
    MT9V022_CONFIG_I2C_ADDRESS_2,
    MT9V022_CONFIG_I2C_ADDRESS_3,
};

bool
mt9v022_detect( sensor_t * sensor )
{
    /* Scan possible bus addresses */
    for( uint8_t i = 0; i < sizeof( mt9v022_i2c_slave_addresses ); i++ )
    {
        /* Try to get MT9V022_CHIP_VERSION register from this address */
        uint16_t chip_id = 0;
        uint8_t  slave_address = mt9v022_i2c_slave_addresses[i];
        if( cambus_readw( slave_address,
                          MT9V022_CHIP_VERSION,
                          &chip_id ) )
        {
            if( ( chip_id == MT9V022_CHIP_ID_REV_1 )
                ||
                ( chip_id == MT9V022_CHIP_ID_REV_3 )
                ||
                ( chip_id == MT9V034_CHIP_ID ) )
            {
                mt9v022_i2c_slave_address = slave_address;
                sensor->slv_addr          = slave_address;
                sensor->chip_id           = chip_id;
                return true;
            }
        }
    }

    return false;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int
mt9v022_init( sensor_t * sensor )
{
    /* Initialize sensor structure. */
    sensor->gs_bpp              = 1;    /* We only read 8 bits per pixel */
    sensor->reset               = mt9v022_reset;
    sensor->sleep               = mt9v022_sleep;
    sensor->read_reg            = mt9v022_read_reg;
    sensor->write_reg           = mt9v022_write_reg;
    sensor->set_pixformat       = mt9v022_set_pixformat;
    sensor->set_framesize       = mt9v022_set_framesize;
    sensor->set_framerate       = mt9v022_set_framerate;
    sensor->set_contrast        = mt9v022_set_contrast;
    sensor->set_brightness      = mt9v022_set_brightness;
    sensor->set_saturation      = mt9v022_set_saturation;
    sensor->set_gainceiling     = mt9v022_set_gainceiling;
    sensor->set_quality         = mt9v022_set_quality;
    sensor->set_colorbar        = mt9v022_set_colorbar;
    sensor->set_auto_gain       = mt9v022_set_auto_gain;
    sensor->get_gain_db         = mt9v022_get_gain_db;
    sensor->set_auto_exposure   = mt9v022_set_auto_exposure;
    sensor->get_exposure_us     = mt9v022_get_exposure_us;
    sensor->set_auto_whitebal   = mt9v022_set_auto_whitebal;
    sensor->get_rgb_gain_db     = mt9v022_get_rgb_gain_db;
    sensor->set_hmirror         = mt9v022_set_hmirror;
    sensor->set_vflip           = mt9v022_set_vflip;
    sensor->set_special_effect  = mt9v022_set_special_effect;
    sensor->set_lens_correction = mt9v022_set_lens_correction;
    sensor->set_windowing       = mt9v022_set_windowing;
    /* sensor->snapshot and sensor->flush are set by default */

    dvp_set_xclk_rate(MT9V022_SYSCLK_FREQ_DEF); /* 26.6MHz Max */

    /* Set sensor flags */
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_VSYNC, 1);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_HSYNC, 1);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_PIXCK, 1);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_FSYNC, 1);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_JPEGE, 0);

    return 0;
}