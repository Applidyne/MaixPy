/**
 * \file    mt9v111.c
 *
 * \brief   Device driver for MT9V022 image sensor, adopted from
 *          mt9d111 device driver by Gabriel Mariano Marcelino
 *          <gabriel.mm8@gmail.com>
 *
 *          MT9V111-Driver is free software: you can redistribute it and/or
 *          modify it under the terms of the GNU Lesser General Public
 *          License as published by the Free Software Foundation, either
 *          version 3 of the License, or (at your option) any later version.
 *
 *          MT9V111-Driver is distributed in the hope that it will be useful,
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

#include <stdio.h>
#include "mt9v111.h"
#include "dvp.h"
#include "plic.h"
#include "sleep.h"
#include "sensor.h"
#include "mphalport.h"
#include "cambus.h"
#include "printf.h"
#include "omv_boardconfig.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static i2c_device_number_t mt9v111_i2c_device = -2; // i2c_device_number_t components/micropython/port/src/omv/cambus.c

static uint8_t mt9v111_i2c_slave_address = 0;
static uint8_t mt9v111_addr_space        = 0;

/**
 * \brief Struct to store an register address and its value.
 */
typedef struct
{
    uint8_t     space;      /**< register space                    */
    uint8_t     reg;        /**< register address                  */
    uint16_t    mask;       /**< if set, use in read-modify-write. */
    uint16_t    value;      /**< register value to set.            */
    uint16_t    wait_ms;    /**< time in ms to wait after write.   */
} Register_t;

/**
 * \brief Default registers values (Values after reset).
 * Derived from
 * https://elixir.bootlin.com/linux/v5.4.77/source/drivers/media/i2c/mt9v111.c
 */

static const Register_t mt9v111_reg_defaults[] =
{
    /* Software reset core and IFP blocks. */
    { .space   = MT9V111_ADDR_SPACE_CORE,
      .reg     = MT9V111_CORE_R0D_CORE_RESET,
      .mask    = MT9V111_CORE_R0D_CORE_RESET_MASK,
      .value   = 1,
      .wait_ms = 1 },
    { .space   = MT9V111_ADDR_SPACE_CORE,
      .reg     = MT9V111_CORE_R0D_CORE_RESET,
      .mask    = MT9V111_CORE_R0D_CORE_RESET_MASK,
      .value   = 0,
      .wait_ms = 1 },
    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     = MT9V111_IFP_R07_IFP_RESET,
      .mask    = MT9V111_IFP_R07_IFP_RESET_MASK,
      .value   = 1,
      .wait_ms = 1 },
    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     = MT9V111_IFP_R07_IFP_RESET,
      .mask    = MT9V111_IFP_R07_IFP_RESET_MASK,
      .value   = 0,
      .wait_ms = 1 },

    /* Configure internal clock sample rate. */

    /* TODO */
    { .space   = MT9V111_ADDR_SPACE_CORE,
      .reg     = MT9V111_CORE_R07_OUT_CTRL,
      .mask    = MT9V111_CORE_R07_OUT_CTRL_SAMPLE,
      .value   = 1,
      .wait_ms = 0 },
    { .space   = MT9V111_ADDR_SPACE_CORE,
      .reg     = MT9V111_CORE_R07_OUT_CTRL,
      .mask    = MT9V111_CORE_R07_OUT_CTRL_SAMPLE,
      .value   = 0,
      .wait_ms = 0 },

    /*
	 * Configure output image format components ordering.
	 *
	 * TODO: IFP block can also output several RGB permutations, we only
	 *	     support YUYV permutations at the moment.
	 */
    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     = MT9V111_IFP_R3A_OUTFMT_CTRL2,
      .mask    = MT9V111_IFP_R3A_OUTFMT_CTRL2_SWAP_MASK,
      .value   = MT9V111_IFP_R3A_OUTFMT_CTRL2_SWAP_YC,
      .wait_ms = 0 },

    /*
	 * Do not change default sensor's core configuration:
	 * output the whole 640x480 pixel array, skip 18 columns and 6 rows.
	 *
	 * Instead, control the output image size through IFP block.
	 *
	 * TODO: No zoom&pan support. Currently we control the output image
	 *	 size only through decimation, with no zoom support.
	 */
    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     = MT9V111_IFP_RA5_HPAN,
      .mask    = 0,
      .value   = MT9V111_IFP_DECIMATION_FREEZE,
      .wait_ms = 0 },
    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     = MT9V111_IFP_RA8_VPAN,
      .mask    = 0,
      .value   = MT9V111_IFP_DECIMATION_FREEZE,
      .wait_ms = 0 },
    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     = MT9V111_IFP_RA6_HZOOM,
      .mask    = 0,
      .value   = MT9V111_IFP_DECIMATION_FREEZE | MT9V111_PIXEL_ARRAY_WIDTH,
      .wait_ms = 0 },
    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     =  MT9V111_IFP_RA9_VZOOM,
      .mask    = 0,
      .value   = MT9V111_IFP_DECIMATION_FREEZE | MT9V111_PIXEL_ARRAY_HEIGHT,
      .wait_ms = 0 },
    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     = MT9V111_IFP_RA7_HOUT,
      .mask    = 0,
      .value   = MT9V111_IFP_DECIMATION_FREEZE | 640, // Default VGA
      .wait_ms = 0 },
    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     = MT9V111_IFP_RAA_VOUT,
      .mask    = 0,
      .value   = MT9V111_IFP_DECIMATION_FREEZE | 480, // Default VGA
      .wait_ms = 0 },

    /* Apply controls to set auto exp, auto awb and timings */

    /* TODO */

    /*
	 * Set pixel integration time to the whole frame time.
	 * This value controls the the shutter delay when running with AE
	 * disabled. If longer than frame time, it affects the output
	 * frame rate.
	 */
    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     = MT9V111_CORE_R09_PIXEL_INT,
      .mask    = 0,
      .value   = MT9V111_PIXEL_ARRAY_HEIGHT,
      .wait_ms = 0 },

    /* Table End Marker */
    { 0, 0, 0, 0, 0 }
};

static const Register_t mt9v111_reg_framesize_QQVGA[] =
{
    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     = MT9V111_ADDR_SPACE,
      .mask    = 0,
      .value   = MT9V111_ADDR_SPACE_IFP,
      .wait_ms = 0 },

    /* Write IFP registers */

    { .space   = MT9V111_ADDR_SPACE_IFP,
      .reg     = MT9V111_ADDR_SPACE,
      .mask    = 0,
      .value   = MT9V111_ADDR_SPACE_CORE,
      .wait_ms = 0 },

    /* Write Core registers */

    /* Table End Marker */
    { 0, 0, 0, 0, 0 }
};

static const Register_t mt9v111_reg_framesize_QVGA[] =
{
    { .reg     = MT9V111_ADDR_SPACE,
      .mask    = 0,
      .value   = MT9V111_ADDR_SPACE_IFP,
      .wait_ms = 0 },

    /* Write IFP registers */

    { .reg     = MT9V111_ADDR_SPACE,
      .mask    = 0,
      .value   = MT9V111_ADDR_SPACE_CORE,
      .wait_ms = 0 },

    /* Write Core registers */

    /* Table End Marker */
    { 0, 0, 0, 0, 0 }
};

static const Register_t mt9v111_reg_framesize_VGA[] =
{
    { .reg     = MT9V111_ADDR_SPACE,
      .mask    = 0,
      .value   = MT9V111_ADDR_SPACE_IFP,
      .wait_ms = 0 },

    /* Write IFP registers */

    { .reg     = MT9V111_ADDR_SPACE,
      .mask    = 0,
      .value   = MT9V111_ADDR_SPACE_CORE,
      .wait_ms = 0 },

    /* Write Core registers */

    /* Table End Marker */
    { 0, 0, 0, 0, 0 }
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
__mt9v111_write( const uint8_t reg, const uint16_t val )
{
    uint8_t tmp[3] = { 0 };
    tmp[0] = (uint8_t)(reg & 0xff);
    tmp[1] = (uint8_t)((val >> 8) & 0xff);
    tmp[2] = (uint8_t)(val & 0xff);
    int ret = maix_i2c_send_data( mt9v111_i2c_device,
                                  mt9v111_i2c_slave_address, tmp, 3, 20 );
    return ret;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static uint16_t
__mt9v111_read( const uint8_t reg )
{
    uint8_t buf[2] = { 0 };
    buf[0] = reg;
    maix_i2c_send_data( mt9v111_i2c_device,
                        mt9v111_i2c_slave_address, buf, 1, 20);
    int ret = maix_i2c_recv_data( mt9v111_i2c_device,
                                  mt9v111_i2c_slave_address, NULL, 0, buf, 2, 30 );
    if (ret == 0)
    {
        return (buf[0] << 8) + buf[1];
    }
    return -1;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
__mt9v111_addr_space_select( const uint8_t space )
{
    if( mt9v111_addr_space == space )
    {
        return 0;
    }

    __mt9v111_write( MT9V111_ADDR_SPACE, space );
    mt9v111_addr_space = space;
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_write( const uint8_t space, const uint8_t reg, const uint16_t val )
{
    __mt9v111_addr_space_select( space );

    return __mt9v111_write( reg, val );
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static uint16_t
mt9v111_read( const uint8_t space, const uint8_t reg )
{
    __mt9v111_addr_space_select( space );

    return __mt9v111_read( reg );
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_write_registers( const Register_t * registers )
{
    uint16_t i = 0;
    while( registers[i].reg != 0 )
    {
        uint16_t value = registers[i].value;

        if( registers[i].mask )
        {
            /* If there is a mask specified, do a read-modify-write */
            value = mt9v111_read( registers[i].space,registers[i].reg );

            /* Blank out the mask area and OR the new value into it */
            value = ( value & ~registers[i].mask )
                    | ( registers[i].value & registers[i].mask );
        }

        mt9v111_write( registers[i].space,
                       registers[i].reg,
                       registers[i].value );

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
mt9v111_reset( sensor_t * sensor)
{
    mt9v111_write_registers( mt9v111_reg_defaults );
    mt9v111_write_registers( mt9v111_reg_framesize_VGA );
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_read_reg( sensor_t * sensor, uint8_t reg_addr )
{
    /* User be aware of selecting the correct user space */
    return __mt9v111_read( reg_addr );
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_write_reg( sensor_t * sensor, uint8_t reg_addr, uint16_t reg_data )
{
    /* User be aware of selecting the correct user space */
    __mt9v111_write( reg_addr, reg_data );

    /* Update our cached space variable, in case users switches through here */
    if( reg_addr == MT9V111_ADDR_SPACE )
    {
        mt9v111_addr_space = reg_data;
    }
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_pixformat( sensor_t * sensor, pixformat_t pixformat)
{
    printk("%s sensor %p\r\n", __func__, sensor);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_framesize( sensor_t * sensor, framesize_t framesize)
{
    printk("%s sensor %p framesize %d\r\n", __func__, sensor, framesize);
    uint16_t width = resolution[framesize][0];
    uint16_t height = resolution[framesize][1];

    switch( framesize )
    {
        case FRAMESIZE_QQVGA:
            mt9v111_write_registers( mt9v111_reg_framesize_QQVGA );
            break;

        case FRAMESIZE_QVGA:
            mt9v111_write_registers( mt9v111_reg_framesize_QVGA );
            break;

        case FRAMESIZE_VGA:
            mt9v111_write_registers( mt9v111_reg_framesize_VGA );
            break;

        default:
            return -1;  /* Return ERROR */
    }

    /* delay n ms */
    dvp_set_image_size(width, height);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_framerate( sensor_t * sensor, framerate_t framerate)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_contrast( sensor_t * sensor, int level)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_brightness( sensor_t * sensor, int level)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_saturation( sensor_t * sensor, int level)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_gainceiling( sensor_t * sensor, gainceiling_t gainceiling)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_quality( sensor_t * sensor, int qs)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_colorbar( sensor_t * sensor, int enable)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_auto_gain( sensor_t * sensor, int enable, float gain_db, float gain_db_ceiling)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_get_gain_db( sensor_t * sensor, float *gain_db)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_auto_exposure( sensor_t * sensor, int enable, int exposure_us)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_get_exposure_us( sensor_t * sensor, int *exposure_us)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_auto_whitebal( sensor_t * sensor, int enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_get_rgb_gain_db( sensor_t * sensor, float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_hmirror( sensor_t * sensor, int enable)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static int
mt9v111_set_vflip( sensor_t * sensor, int enable)
{
    printk("%s %s %d\r\n", __func__, __FILE__, __LINE__);
    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Return TRUE when an MT9V111 sensor is detected */

static const uint8_t mt9v111_i2c_slave_addresses[] =
{
    MT9V111_CONFIG_I2C_ADDRESS_0,
    MT9V111_CONFIG_I2C_ADDRESS_1,
};

bool
mt9v111_detect( sensor_t * sensor )
{
    /* Scan possible bus addresses */
    for( uint8_t i = 0; i < sizeof( mt9v111_i2c_slave_addresses ); i++ )
    {
        /* Try to get MT9V111_CORE_CHIP_VERSION register from this address */
        uint16_t chip_id = 0;
        uint8_t  slave_address = mt9v111_i2c_slave_addresses[i];
        if( cambus_readw( slave_address,
                          MT9V111_CORE_CHIP_VERSION,
                          &chip_id ) )
        {
            if( chip_id == MT9V111_CHIP_ID )
            {
                mt9v111_i2c_slave_address = slave_address;
                sensor->slv_addr          = slave_address;
                sensor->chip_id           = chip_id;
                return true;
            }
        }
    }

    return false;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int mt9v111_init( sensor_t * sensor)
{
    //Initialize sensor structure.
    sensor->gs_bpp              = 1;
    sensor->reset               = mt9v111_reset;
    sensor->read_reg            = mt9v111_read_reg;
    sensor->write_reg           = mt9v111_write_reg;
    sensor->set_pixformat       = mt9v111_set_pixformat;
    sensor->set_framesize       = mt9v111_set_framesize;
    sensor->set_framerate       = mt9v111_set_framerate;
    sensor->set_contrast        = mt9v111_set_contrast;
    sensor->set_brightness      = mt9v111_set_brightness;
    sensor->set_saturation      = mt9v111_set_saturation;
    sensor->set_gainceiling     = mt9v111_set_gainceiling;
    sensor->set_quality         = mt9v111_set_quality;
    sensor->set_colorbar        = mt9v111_set_colorbar;
    sensor->set_auto_gain       = mt9v111_set_auto_gain;
    sensor->get_gain_db         = mt9v111_get_gain_db;
    sensor->set_auto_exposure   = mt9v111_set_auto_exposure;
    sensor->get_exposure_us     = mt9v111_get_exposure_us;
    sensor->set_auto_whitebal   = mt9v111_set_auto_whitebal;
    sensor->get_rgb_gain_db     = mt9v111_get_rgb_gain_db;
    sensor->set_hmirror         = mt9v111_set_hmirror;
    sensor->set_vflip           = mt9v111_set_vflip;

    dvp_set_xclk_rate( MT9V111_SYSCLK_FREQ_DEF ); /* 24-27MHz */

    // Set sensor flags
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_VSYNC, 0);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_HSYNC, 0);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_PIXCK, 1);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_FSYNC, 0);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_JPEGE, 1);

    return 0;
}

/* ~~~~~ End ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
