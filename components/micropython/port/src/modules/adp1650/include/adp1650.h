/**
 * \file    adp1650.h
 *
 * \brief   Device driver for ADP1650 LED driver.
 *
 *          Adopted from a similar file copyrighted 2011 by Analog Devices Inc.
 *
 *          adp1650 driver is free software: you can redistribute it and/or
 *          modify it under the terms of the GNU Lesser General Public
 *          License as published by the Free Software Foundation, either
 *          version 3 of the License, or (at your option) any later version.
 *
 *          adp1650 driver is distributed in the hope that it will be useful,
 *          but WITHOUT ANY WARRANTY; without even the implied warranty of
 *          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *          GNU Lesser General Public License for more details.
 *
 *          You should have received a copy of the GNU Lesser General Public
 *          License along with adp1650 driver files.
 *
 *          If not, see <http://www.gnu.org/licenses/>.
 *
 * \author  Marco Hess <marcoh@applidyne.com.au>
 *
 * \date    24/09/2021
 */

#ifndef __ADP1650_H__
#define __ADP1650_H__

/* ~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/**
 * ADP1650 Registers & Settings
 */

#define ADP1650_CONFIG_I2C_ADDRESS                          0x30        /* 0x60 write 0x61 read */

#define ADP1650_REG_VERSION		                    0x00
#define         ADP1650_CHIP_ID                             b00100010   /* 0x22 */

#define ADP1650_REG_TIMER_IOCFG		                0x02
#define         ADP1650_IOCFG_IO2_HIGH_IMP	                (0 << 6) /* High Impedance */
#define         ADP1650_IOCFG_IO2_IND_LED	                (1 << 6) /* Indicator LED */
#define         ADP1650_IOCFG_IO2_TXMASK2	                (2 << 6) /* TxMASK2 operation mode */
#define         ADP1650_IOCFG_IO2_AIN		                (3 << 6) /* ADC analog input */
#define         ADP1650_IOCFG_IO1_HIGH_IMP	                (0 << 4) /* High Impedance */
#define         ADP1650_IOCFG_IO1_TORCH		                (1 << 4) /* Torch mode */
#define         ADP1650_IOCFG_IO1_TXMASK1	                (2 << 4) /* TxMASK1 operation mode */
#define         ADP1650_FL_TIMER_ms(x)		                ((((x) - 100) / 100) & 0xF) /* Timer */

#define ADP1650_REG_CURRENT_SET		                0x03
#define         ADP1650_I_FL_mA(x)		                    ((((x) - 300) / 50) << 3)
#define         ADP1650_I_TOR_mA(x)		                    ((((x) - 25) / 25) & 0x7)

#define ADP1650_REG_OUTPUT_MODE		                0x04
#define         ADP1650_IL_PEAK_1A75	    	            (0 << 6) /* Inductor peak current */
#define         ADP1650_IL_PEAK_2A25	    	            (1 << 6)
#define         ADP1650_IL_PEAK_2A75	    	            (2 << 6)
#define         ADP1650_IL_PEAK_3A00	    	            (3 << 6)
#define         ADP1650_STR_LV_EDGE		                    (0 << 5) /* Strobe edge or level sensitive */
#define         ADP1650_STR_LV_LEVEL	       	            (1 << 5)
#define         ADP1650_FREQ_FB_EN		                    (1 << 4) /* Frequency foldback allowed */
#define         ADP1650_OUTPUT_EN		                    (1 << 3) /* Output enable */
#define         ADP1650_STR_MODE_SW		                    (0 << 2) /* Strobe mode software */
#define         ADP1650_STR_MODE_HW		                    (1 << 2) /* Strobe mode hardware */
#define         ADP1650_LED_MODE_STBY		                (0 << 0) /* LED mode standby */
#define         ADP1650_LED_MODE_VOUT		                (1 << 0) /* LED mode VOUT ON */
#define         ADP1650_LED_MODE_ASSIST_LIGHT	            (2 << 0) /* LED mode TORCH */
#define         ADP1650_LED_MODE_FLASH		                (3 << 0) /* LED mode FLASH */

#define ADP1650_REG_FAULT		                    0x05
#define         ADP1650_FL_OVP			                    (1 << 7) /* Overvoltage FAULT */
#define         ADP1650_FL_SC			                    (1 << 6) /* Short Circuit FAULT */
#define         ADP1650_FL_OT			                    (1 << 5) /* Over Temperature FAULT */
#define         ADP1650_FL_TO			                    (1 << 4) /* Time Out FAULT */
#define         ADP1650_FL_TX1			                    (1 << 3) /* Tx MAKS1 FAULT */
#define         ADP1650_FL_IO2			                    (1 << 2) /* Tx MASK2 FAULT */
#define         ADP1650_FL_IL			                    (1 << 1) /* Inductor Peak Curent FAULT */
#define         ADP1650_FL_IDC			                    (1 << 0) /* Programmed Current Limit FAULT */

#define ADP1650_REG_CONTROL		                    0x06
#define         ADP1650_I_TX2_mA(x)		                    ((((x) - 100) / 50) << 4)  /* 100..850mA */
#define         ADP1650_I_TX1_mA(x)		                    ((((x) - 100) / 50) & 0xF) /* 100..850mA */

#define ADP1650_REG_AD_MODE		                    0x07
#define         ADP1650_DYN_OVP_EN			                (1 << 7) /* Dynamic OVP  */
#define         ADP1650_SW_LO_1MHz5			                (1 << 6) /* Force 1.5MHz */
#define         ADP1650_STR_POL_ACTIVE_HIGH		            (1 << 5) /* Strobe polarity active high */
#define         ADP1650_I_ILED_2mA75			            (0 << 4) /* Indicator LED current */
#define         ADP1650_I_ILED_5mA50			            (1 << 4)
#define         ADP1650_I_ILED_8mA25			            (2 << 4)
#define         ADP1650_I_ILED_11mA00			            (3 << 4)
#define         ADP1650_IL_DC_1A50			                (0 << 1) /* Input DC current limit LED */
#define         ADP1650_IL_DC_1A75			                (1 << 1)
#define         ADP1650_IL_DC_2A00			                (2 << 1)
#define         ADP1650_IL_DC_2A25			                (3 << 1)
#define         ADP1650_IL_DC_EN			                (1 << 0) /* Input DC current limit enable */

#define ADP1650_REG_ADC			                    0x08
#define         ADP1650_FL_VB_LO			                (1 << 6) /* VBAT low threshold status */
#define         ADP1650_ADC_VAL(x)			                (((x) & 0x3C) >> 2) /* Readback ADC value */
#define         ADP1650_ADC_DIS				                (0 << 0) /* ADC mode disabled */
#define         ADP1650_ADC_LED_VF			                (1 << 0) /* ADC mode LED Vf */
#define         ADP1650_ADC_DIE_TEMP			            (2 << 0) /* ADC mode Temp */
#define         ADP1650_ADC_EXT_VOLT			            (3 << 0) /* ADC mode Ext Voltage */

#define ADP1650_REG_BATT_LOW		                0x09
#define         ADP1650_CL_SOFT_EN			                (1 << 7) /* Soft inductor peal limit enable */
#define         ADP1650_I_VB_LO_mA(x)			            ((((x) - 300) / 50) << 3)   /* Current setting for VBAT low mode */
#define         ADP1650_V_VB_LO_DIS			                (0 << 0) /* VDD level for VBAT low disabled */
#define         ADP1650_V_VB_LO_3V30			            (1 << 0) /* VDD level 3.3V */
#define         ADP1650_V_VB_LO_3V35			            (2 << 0)
#define         ADP1650_V_VB_LO_3V40			            (3 << 0)
#define         ADP1650_V_VB_LO_3V45			            (4 << 0)
#define         ADP1650_V_VB_LO_3V50			            (5 << 0)
#define         ADP1650_V_VB_LO_3V55			            (6 << 0)
#define         ADP1650_V_VB_LO_3V60			            (7 << 0) /* VDD level 3.6V */

/* ~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/*
 * Flash Modes to use with led_mode_set function
 */
typedef enum
{
    FL_MODE_OFF			                   =     0, /* OFF */
    FL_MODE_TORCH_25mA		               =     1, /* SW trigged TORCH to FLASH */
    FL_MODE_TORCH_50mA		               =     2, /* TORCH Intensity XmA */
    FL_MODE_TORCH_75mA		               =     3,
    FL_MODE_TORCH_100mA		               =     4,
    FL_MODE_TORCH_125mA		               =     5,
    FL_MODE_TORCH_150mA		               =     6,
    FL_MODE_TORCH_175mA		               =     7,
    FL_MODE_TORCH_200mA		               =     8,
    FL_MODE_TORCH_TRIG_EXT_25mA	           =     9, /* HW/IO trigged TORCH to FLASH */
    FL_MODE_TORCH_TRIG_EXT_50mA	           =    10, /* TORCH Intensity XmA */
    FL_MODE_TORCH_TRIG_EXT_75mA	           =    11,
    FL_MODE_TORCH_TRIG_EXT_100mA	       =    12,
    FL_MODE_TORCH_TRIG_EXT_125mA	       =    13,
    FL_MODE_TORCH_TRIG_EXT_150mA	       =    14,
    FL_MODE_TORCH_TRIG_EXT_175mA	       =    15,
    FL_MODE_TORCH_TRIG_EXT_200mA	       =    16,
    FL_MODE_FLASH			               =   254, /* SW triggered FLASH */
    FL_MODE_FLASH_TRIG_EXT		           =   255, /* HW/Strobe trigged FLASH */
} AP1650_Mode_t;

/*
 * ADC Channels to read
 */
typedef enum
{
    ADC_CHANNEL_LED_VF,
    ADC_CHANNEL_TEMPERATURE,
    ADC_CHANNEL_EXT_VOLTAGE,
} AP1650_ADC_Channel_t;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Return TRUE when an ADP1650 is detected on the indicated bus */

bool
adp1650_detect( i2c_client * i2c );

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Init the ADP1650 to default settings */

int
adp1650_init( i2c_client * i2c );

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Configure the indicated mode */

int
adp1650_set_mode( i2c_client * i2c, AP1650_Mode_t mode );

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Set LED brightness  */

int
adp1650_set_brightness( i2c_client * i2c, AP1650_Mode_t mode );

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Return the ADC reading of the indicated channel */

int
adp1650_get_adc( i2c_client * i2c, AP1650_ADC_Channel_t adc_mode );

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** Return the current fault status mask */

int
adp1650_get_fault_status( i2c_client * i2c );

/* ~~~~~ End ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#endif // __ADP1650_H__