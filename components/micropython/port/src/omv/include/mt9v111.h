/**
 * \file    mt9v111.h
 *
 * \brief   Device driver for MT9V111 image sensor, adopted from
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

#ifndef __MT9V111_H__
#define __MT9V111_H__

#include <stdint.h>
#include <stdbool.h>

#include "sensor.h"
#include "sipeed_i2c.h"

/* Default I2C address (controlled by SADDR line) */
#define MT9V111_CONFIG_I2C_ADDRESS_0                            0x90    /* 0 */
#define MT9V111_CONFIG_I2C_ADDRESS_1                            0xB8    /* 1 */

/* The first four rows are black rows. The active area spans 753x481 pixels. */
#define MT9V111_PIXEL_ARRAY_WIDTH			                    640
#define MT9V111_PIXEL_ARRAY_HEIGHT			                    480

/* Default pixel clock rate */
#define MT9V111_SYSCLK_FREQ_DEF				                    26600000

/* Register addresses & default values */
#define MT9V111_ADDR_SPACE				                0x01
#define         MT9V111_ADDR_SPACE_IFP					        0x01    /* Default */
#define         MT9V111_ADDR_SPACE_CORE				            0x04

#define MT9V111_IFP_R06_OPMODE_CTRL			            0x06
#define		    MT9V111_IFP_R06_OPMODE_CTRL_AWB_EN	            BIT(1)
#define		    MT9V111_IFP_R06_OPMODE_CTRL_AE_EN	            BIT(14)

#define MT9V111_IFP_R07_IFP_RESET			            0x07
#define		    MT9V111_IFP_R07_IFP_RESET_MASK		            BIT(0)

#define MT9V111_IFP_R08_OUTFMT_CTRL			            0x08
#define		    MT9V111_IFP_R08_OUTFMT_CTRL_FLICKER	            BIT(11)
#define		    MT9V111_IFP_R08_OUTFMT_CTRL_PCLK	            BIT(5)

#define MT9V111_IFP_R3A_OUTFMT_CTRL2			        0x3a
#define		    MT9V111_IFP_R3A_OUTFMT_CTRL2_SWAP_CBCR	        BIT(0)
#define		    MT9V111_IFP_R3A_OUTFMT_CTRL2_SWAP_YC	        BIT(1)
#define		    MT9V111_IFP_R3A_OUTFMT_CTRL2_SWAP_MASK	        GENMASK(2, 0)

#define MT9V111_IFP_RA5_HPAN				            0xa5
#define MT9V111_IFP_RA6_HZOOM				            0xa6
#define MT9V111_IFP_RA7_HOUT				            0xa7
#define MT9V111_IFP_RA8_VPAN				            0xa8
#define MT9V111_IFP_RA9_VZOOM				            0xa9
#define MT9V111_IFP_RAA_VOUT				            0xaa
#define         MT9V111_IFP_DECIMATION_MASK			            GENMASK(9, 0)
#define         MT9V111_IFP_DECIMATION_FREEZE			        BIT(15)

#define MT9V111_CORE_R03_WIN_HEIGHT			            0x03
#define		    MT9V111_CORE_R03_WIN_V_OFFS		                2

#define MT9V111_CORE_R04_WIN_WIDTH			            0x04
#define		    MT9V111_CORE_R04_WIN_H_OFFS		                114

#define MT9V111_CORE_R05_HBLANK				            0x05
#define		    MT9V111_CORE_R05_MIN_HBLANK		                0x09
#define		    MT9V111_CORE_R05_MAX_HBLANK		                GENMASK(9, 0)
#define		    MT9V111_CORE_R05_DEF_HBLANK		                0x26

#define MT9V111_CORE_R06_VBLANK				            0x06
#define		    MT9V111_CORE_R06_MIN_VBLANK		                0x03
#define		    MT9V111_CORE_R06_MAX_VBLANK		                GENMASK(11, 0)
#define		    MT9V111_CORE_R06_DEF_VBLANK		                0x04

#define MT9V111_CORE_R07_OUT_CTRL			            0x07
#define		    MT9V111_CORE_R07_OUT_CTRL_SAMPLE	            BIT(4)

#define MT9V111_CORE_R09_PIXEL_INT			            0x09
#define		    MT9V111_CORE_R09_PIXEL_INT_MASK		            GENMASK(11, 0)

#define MT9V111_CORE_R0D_CORE_RESET			            0x0d
#define		    MT9V111_CORE_R0D_CORE_RESET_MASK	            BIT(0)

#define MT9V111_CORE_CHIP_VERSION				        0x36            /* Repeated in 0xff */
#define	        MT9V111_CHIP_ID          	                    0x823A

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Return TRUE when an MT9V111 sensor is detected on the indicated bus */

bool
mt9v111_detect( sensor_t * sensor );

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Connect the MT9V111 driver to the sensor device */

int
mt9v111_init( sensor_t *sensor );

/* ~~~~~ End ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif // __MT9V111_H__
