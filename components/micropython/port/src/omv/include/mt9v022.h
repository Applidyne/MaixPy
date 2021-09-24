/**
 * \file    mt9v022.h
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

#ifndef __MT9V022_H__
#define __MT9V022_H__

#include <stdint.h>
#include <stdbool.h>

#include "sensor.h"
#include "sipeed_i2c.h"

/* 7-bit I2C slave addresses as controlled by S_CTRL_ADR0 and S_CTRL_ADR1
 * to write. For read address is +1  */
#define MT9V022_CONFIG_I2C_ADDRESS_0                        0x90    /* 0 0 */
#define MT9V022_CONFIG_I2C_ADDRESS_1                        0x98    /* 0 1 */
#define MT9V022_CONFIG_I2C_ADDRESS_2                        0xB0    /* 1 0 */
#define MT9V022_CONFIG_I2C_ADDRESS_3                        0xB8    /* 1 1 */

/* The first four rows are black rows. The active area spans 753x481 pixels. */
#define MT9V022_PIXEL_ARRAY_HEIGHT                          492
#define MT9V022_PIXEL_ARRAY_WIDTH                           782

/* Default pixel clock rate */
#define MT9V022_SYSCLK_FREQ_DEF                             26600000

/* Register addresses & default values */
#define MT9V022_CHIP_VERSION                        0x00
#define         MT9V022_CHIP_ID_REV_1                       0x1311
#define         MT9V022_CHIP_ID_REV_3                       0x1313
#define         MT9V034_CHIP_ID                             0x1324

#define MT9V022_COLUMN_START                        0x01
#define         MT9V022_COLUMN_START_MIN                    1
#define         MT9V022_COLUMN_START_DEF                    1
#define         MT9V022_COLUMN_START_MAX                    752

#define MT9V022_ROW_START                           0x02
#define         MT9V022_ROW_START_MIN                       4
#define         MT9V022_ROW_START_DEF                       5
#define         MT9V022_ROW_START_MAX                       482

#define MT9V022_WINDOW_HEIGHT                       0x03
#define         MT9V022_WINDOW_HEIGHT_MIN                   1
#define         MT9V022_WINDOW_HEIGHT_DEF                   480
#define         MT9V022_WINDOW_HEIGHT_MAX                   480

#define MT9V022_WINDOW_WIDTH                        0x04
#define         MT9V022_WINDOW_WIDTH_MIN                    1
#define         MT9V022_WINDOW_WIDTH_DEF                    752
#define         MT9V022_WINDOW_WIDTH_MAX                    752

/**
 *  Number of blank columns in a row. Minimum horizontal blanking is 61 for
 *  normal mode, 71 for column bin 2 mode and 91 for column bin 4 mode.
 */
#define MT9V022_HORIZONTAL_BLANKING                 0x05
#define         MT9V022_HORIZONTAL_BLANKING_MIN             61
#define         MT9V022_HORIZONTAL_BLANKING_BIN_1_DEF       61
#define         MT9V022_HORIZONTAL_BLANKING_BIN_2_DEF       71
#define         MT9V022_HORIZONTAL_BLANKING_BIN_4_DEF       91
#define         MT9V022_HORIZONTAL_BLANKING_MAX             1023

#define MT9V022_VERTICAL_BLANKING                   0x06
#define         MT9V022_VERTICAL_BLANKING_MIN               4
#define         MT9V022_VERTICAL_BLANKING_DEF               45
#define         MT9V022_VERTICAL_BLANKING_MAX               3000

#define MT9V022_CHIP_CONTROL                        0x07
#define         MT9V022_CHIP_CONTROL_MASTER_MODE            (1 << 3)
#define         MT9V022_CHIP_CONTROL_DOUT_ENABLE            (1 << 7)
#define         MT9V022_CHIP_CONTROL_SEQUENTIAL             (1 << 8)

#define MT9V022_SHUTTER_WIDTH1                      0x08

#define MT9V022_SHUTTER_WIDTH2                      0x09

#define MT9V022_SHUTTER_WIDTH_CONTROL               0x0a

/**
 *  Total integration time in number of rows. This value is used only when
 *  AEC is disabled (bit 0 of R0xAF). This register is not shadowed, but any
 *  change made does not take effect until the following new frame.
 */
#define MT9V022_TOTAL_SHUTTER_WIDTH                 0x0b
#define         MT9V022_TOTAL_SHUTTER_WIDTH_MIN             1
#define         MT9V034_TOTAL_SHUTTER_WIDTH_MIN             0
#define         MT9V022_TOTAL_SHUTTER_WIDTH_DEF             480
#define         MT9V022_TOTAL_SHUTTER_WIDTH_MAX             32767
#define         MT9V034_TOTAL_SHUTTER_WIDTH_MAX             32765

#define MT9V022_RESET                               0x0c
#define         MT9V022_RESET_DIGITAL                       (1 << 0)
#define         MT9V022_RESET_AGC                           (1 << 1)

#define MT9V022_READ_MODE                           0x0d
#define         MT9V022_READ_MODE_ROW_BIN_MASK              (3 << 0)
#define         MT9V022_READ_MODE_ROW_BIN_SHIFT             0
#define         MT9V022_READ_MODE_COLUMN_BIN_MASK           (3 << 2)
#define         MT9V022_READ_MODE_COLUMN_BIN_SHIFT          2
#define         MT9V022_READ_MODE_ROW_FLIP                  (1 << 4)
#define         MT9V022_READ_MODE_COLUMN_FLIP               (1 << 5)
#define         MT9V022_READ_MODE_DARK_COLUMNS              (1 << 6)
#define         MT9V022_READ_MODE_DARK_ROWS                 (1 << 7)
#define         MT9V022_READ_MODE_RESERVED                  0x0300

/**
 * The sensor goes into monitor mode when R0x0E bit 0 is set to HIGH.
 * In this mode, the sensor first captures a programmable number of frames
 * (R0xC0), then goes into a sleep period for five minutes. The cycle of
 * sleeping for five minutes and waking up to capture a number of frames
 * continues until R0x0E bit 0 is cleared to return to normal operation.
 *
 * In some applications when monitor mode is enabled, the purpose of capturing
 * frames is to calibrate the gain and exposure of the scene using automatic
 * gain and exposure control feature. This feature typically takes less than
 * 10 frames to settle. In case a larger number of frames is needed, the value
 * of R0xC0 may be increased to capture more frames.
 *
 * See R0xC0 MT9V022_MONITOR_MODE_CAPTURE_CONTROL
 *
 * During the sleep period, none of the analog circuitry and a very small
 * fraction of digital logic (including a five-minute timer) is powered.
 * The master clock (SYSCLK) is therefore always required.
 */
#define MT9V022_MONITOR_MODE                        0x0e
#define         MT9V022_MONITOR_MODE_ENABLE                 (1 << 0)
#define         MT9V022_MONITOR_MODE_DISABLE                0

#define MT9V022_PIXEL_OPERATION_MODE                0x0f
#define         MT9V022_PIXEL_OPERATION_MODE_COLOR          (1 << 2)
#define         MT9V022_PIXEL_OPERATION_MODE_HDR            (1 << 6)

/**
 * The formula for gain setting is:  Gain = Bits[6:0] x 0.0625 (EQ 7)
 *
 * The analog gain range supported in the MT9V022 is 1X–4X with a step size of
 * 6.25 percent. To control gain manually with this register, the sensor must
 * NOT be in AGC mode. When adjusting the luminosity of an image, it is
 * recommended to alter exposure first and yield to gain increases only
 * when the exposure value has reached a maximum limit.
 *
 * Analog gain = bits (6:0) x 0.0625 for values 16–31
 * Analog gain = bits (6:0)/2 x 0.125 for values 32–64
 *
 * For values 16–31: each LSB increases analog gain 0.0625v/v.
 * A value of 16 = 1X gain.
 *
 * Range: 1X to 1.9375X.
 * For values 32–64: each 2 LSB increases analog gain 0.125v/v
 * (that is, double the gain increase for 2 LSB).
 *
 * Range: 2X to 4X. Odd values do not result in gain increases; the gain
 * increases by 0.125 for values 32, 34, 36, and so on.
 */
#define MT9V022_ANALOG_GAIN                         0x35
#define         MT9V022_ANALOG_GAIN_MIN                     16
#define         MT9V022_ANALOG_GAIN_DEF                     16
#define         MT9V022_ANALOG_GAIN_MAX                     64

/**
 * When AGC is enabled (R0xAF[1] = 1), the maximum auto gain value is limited
 * by R0x36; minimum auto gain is fixed to 16 gain-units.
 */
#define MT9V022_MAX_ANALOG_GAIN                     0x36
#define         MT9V022_MAX_ANALOG_GAIN_MAX                 127

#define MT9V022_FRAME_DARK_AVERAGE                  0x42

#define MT9V022_DARK_AVG_THRESH                     0x46
#define         MT9V022_DARK_AVG_LOW_THRESH_MASK            (255 << 0)
#define         MT9V022_DARK_AVG_LOW_THRESH_SHIFT           0
#define         MT9V022_DARK_AVG_HIGH_THRESH_MASK           (255 << 8)
#define         MT9V022_DARK_AVG_HIGH_THRESH_SHIFT          8

#define MT9V022_BLACK_LEVEL_CALIB_CTRL              0x47

#define MT9V022_ROW_NOISE_CORR_CONTROL              0x70
#define         MT9V034_ROW_NOISE_CORR_ENABLE               (1 << 0)
#define         MT9V034_ROW_NOISE_CORR_USE_BLK_AVG          (1 << 1)
#define         MT9V022_ROW_NOISE_CORR_ENABLE               (1 << 5)
#define         MT9V022_ROW_NOISE_CORR_USE_BLK_AVG          (1 << 7)

#define MT9V034_PIXEL_CLOCK                         0x72
#define         MT9V022_PIXEL_CLOCK_INV_LINE                (1 << 0)
#define         MT9V022_PIXEL_CLOCK_INV_FRAME               (1 << 1)
#define         MT9V022_PIXEL_CLOCK_XOR_LINE                (1 << 2)
#define         MT9V022_PIXEL_CLOCK_CONT_LINE               (1 << 3)
#define         MT9V022_PIXEL_CLOCK_INV_PXL_CLK             (1 << 4)

#define MT9V022_TEST_PATTERN                        0x7f
#define         MT9V022_TEST_PATTERN_DATA_MASK              (1023 << 0)
#define         MT9V022_TEST_PATTERN_DATA_SHIFT             0
#define         MT9V022_TEST_PATTERN_NONE                   0
#define         MT9V022_TEST_PATTERN_USE_DATA               (1 << 10)
#define         MT9V022_TEST_PATTERN_GRAY_MASK              (3 << 11)
#define         MT9V022_TEST_PATTERN_GRAY_NONE              (0 << 11)
#define         MT9V022_TEST_PATTERN_GRAY_VERTICAL          (1 << 11)
#define         MT9V022_TEST_PATTERN_GRAY_HORIZONTAL        (2 << 11)
#define         MT9V022_TEST_PATTERN_GRAY_DIAGONAL          (3 << 11)
#define         MT9V022_TEST_PATTERN_ENABLE                 (1 << 13)
#define         MT9V022_TEST_PATTERN_FLIP                   (1 << 14)

/*
 * Value between 0 and 64. Default 58.
 */
#define MT9V022_AEGC_DESIRED_BIN                    0xa5
#define         MT9V022_AEGC_DESIRED_BIN_MIN                1
#define         MT9V022_AEGC_DESIRED_BIN_DEF                58
#define         MT9V022_AEGC_DESIRED_BIN_MAX                64

/*
 * Value between 0 and 15. This is the number of frames being skipped before
 * updating the auto exposure/gain.
 */
#define MT9V022_AEC_UPDATE_FREQUENCY                0xa6
#define         MT9V022_AEC_UPDATE_FREQUENCY_MIN            0
#define         MT9V022_AEC_UPDATE_FREQUENCY_DEF            2
#define         MT9V022_AEC_UPDATE_FREQUENCY_MAX            15

#define MT9V022_AGC_UPDATE_FREQUENCY                0xa9
#define         MT9V022_AGC_UPDATE_FREQUENCY_MIN            0
#define         MT9V022_AGC_UPDATE_FREQUENCY_DEF            2
#define         MT9V022_AGC_UPDATE_FREQUENCY_MAX            15

/*
 * LPF is the low pass filter capability of the chip. Both AEC and AGC have
 * this setting. This limits the speed in which AGC/AEC adjust their settings.
 * Possible values are 0-2. 0 means no LPF. For 1 and 2 this equation is used:
 *
 * if |(calculated new exp - current exp)| > (current exp / 4)
 *	next exp = calculated new exp
 * else
 *	next exp = current exp + ((calculated new exp - current exp) / 2^LPF)
 */
#define MT9V022_AEC_LPF                             0xa8
#define         MT9V022_AEC_LPF_MIN                         0
#define         MT9V022_AEC_LPF_DEF                         2
#define         MT9V022_AEC_LPF_MAX                         2

#define MT9V022_AGC_LPF                             0xaa
#define         MT9V022_AGC_LPF_MIN                         0
#define         MT9V022_AGC_LPF_DEF                         2
#define         MT9V022_AGC_LPF_MAX                         2

#define MT9V022_AEC_AGC_ENABLE                      0xaf
#define         MT9V022_AEC_ENABLE                          (1 << 0)
#define         MT9V022_AGC_ENABLE                          (1 << 1)

/**
 * When AEC is enabled (R0xAF[0] = 1), the maximum auto exposure value is
 * limited by R0xBD; minimum auto exposure is fixed at 1 row.
 */
#define MT9V022_AEC_MAX_SHUTTER_WIDTH               0xbd
#define         MT9V022_AEC_MAX_SHUTTER_WIDTH_MIN           0
#define         MT9V022_AEC_MAX_SHUTTER_WIDTH_DEF           480
#define         MT9V022_AEC_MAX_SHUTTER_WIDTH_MAX           32765

/**
 * In some applications when monitor mode is enabled, the purpose of capturing
 * frames is to calibrate the gain and exposure of the scene using automatic
 * gain and exposure control feature. This feature typically takes less than
 * 10 frames to settle. In case a larger number of frames is needed, the value
 * of R0xC0 may be increased to capture more frames.
 *
 * See also MT9V022_MONITOR_MODE
 */
#define MT9V022_MONITOR_MODE_CAPTURE_CONTROL        0xc0
#define         MT9V022_MONITOR_MODE_CAPTURE_CONTROL_DEF    10

#define MT9V022_THERMAL_INFO                        0xc1

/**
 *  This register combined with the Coarse/Total shutter width, defines the
 *  total integration time, This register is not shadowed, but any change
 *  made does not take effect until the following new frame. Register units
 *  are master clock cycles. Maximum HBLANK( R0x05) + 751 = 1023 + 751 = 1774.
 *
 *  Note: When Coarse/Total shutter width is zero,
 *        minimum fine shutter width = 260.
 */
#define MT9V022_FINE_SHUTTER_WIDTH                  0xd5
#define         MT9V022_FINE_SHUTTER_WIDTH_MIN              0
#define         MT9V022_FINE_SHUTTER_WIDTH_DEF              0
#define         MT9V022_FINE_SHUTTER_WIDTH_MAX              1774

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Return TRUE when an MT9V022 sensor is detected on the indicated bus */

bool
mt9v022_detect( sensor_t * sensor );

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Connect the MT9V022 driver to the sensor device */

int
mt9v022_init( sensor_t * sensor );

/* ~~~~~ End ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif // __MT9V022_H__

