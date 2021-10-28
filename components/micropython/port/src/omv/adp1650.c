/**
 * \file    adp1650.c
 *
 * \brief   Device driver for ADP1650 LED driver.
 *
 *          Adopted from a similar file copyrighted 2011 by Analog Devices Inc.
 *
 * \author  Marco Hess <marcoh@applidyne.com.au>
 *
 * \date    28/10/2021
 */

/* -------------------------------------------------------------------------- */

#include "adp1650.h"

#include "mpprint.c"
#include "mphalport.h"

/* -------------------------------------------------------------------------- */

static int
__adp1650_write( i2c_device_number_t i2c,
                 uint8_t             reg,
                 uint8_t             data )
{
    uint8_t tx_data[2];
    tx_data[0] = reg;
    tx_data[1] = data;

    int ret = maix_i2c_send_data( i2c,
                                  ADP1650_CONFIG_I2C_ADDRESS,
                                  tx_data,
                                  2,
                                  100 );

    mp_printf( &mp_plat_print,
               "[adp1650]: write reg %d val 0x%X ret %d\n", reg, data, ret );

	return ret;
}

/* -------------------------------------------------------------------------- */

static int
__adp1650_read( i2c_device_number_t i2c,
                uint8_t             reg,
                uint8_t *           rx_data )
{
    uint8_t tx_data = reg;

    int ret = maix_i2c_recv_data( i2c,
                                  ADP1650_CONFIG_I2C_ADDRESS,
                                  &tx_data,
                                  1,
                                  rx_data,
                                  1,
                                  100 );

    mp_printf( &mp_plat_print,
               "[adp1650]: read  reg %d val 0x%X ret %d\n", reg, *rx_data, ret );

    return ret;
}

/* -------------------------------------------------------------------------- */

/** Return TRUE when an ADP1650 is detected on the indicated bus */

bool
adp1650_detect( i2c_device_number_t i2c )
{
    uint8_t id;
    int ret = __adp1650_read( i2c,
                              ADP1650_REG_VERSION,
                              &id );
    if( ret >= 0 )
    {
        return (id == ADP1650_CHIP_ID);
    }
    return false;
}

/* -------------------------------------------------------------------------- */

/** Init the ADP1650 to default settings */

int
adp1650_init( i2c_device_number_t i2c )
{
	/* I/O configuration:
	 * GPIO1 is flash control
	 * GPIO2 is ambient light ADC input
	 * Flash timer configuration 100 ms max
	 */
    int ret = __adp1650_write( i2c,
	                           ADP1650_REG_TIMER_IOCFG,
							   ADP1650_IOCFG_IO1_TORCH
							   | ADP1650_IOCFG_IO2_AIN
							   | ADP1650_FL_TIMER_ms( 100 ) );
	if( ret < 0 )
	{
		return ret;
	}

	/* Flash default current 100mA, torch current 50mA */
    ret = __adp1650_write( i2c,
	                       ADP1650_REG_CURRENT_SET,
	   			           ADP1650_I_FL_mA( 250 )
						   | ADP1650_I_TOR_mA( 50 ) );
	if( ret < 0 )
	{
		return ret;
	}

	/* Output mode:
	 * - inductor peak 1.75A
	 * - Strobe level sensitive
	 * - No frequency fold back
	 * - Output ENABLED
	 * - Hardware strobe mode
	 * - LED output mode FLASH
	 */
    ret = __adp1650_write( i2c,
						   ADP1650_REG_OUTPUT_MODE,
					       ADP1650_IL_PEAK_1A75
						   | ADP1650_STR_LV_LEVEL
						   | ADP1650_OUTPUT_EN
						   | ADP1650_STR_MODE_HW
						   | ADP1650_LED_MODE_FLASH );
	if( ret < 0 )
	{
		return ret;
	}

	/* TxMASK1 & Tx_MASK2 => not actually used */
    ret = __adp1650_write( i2c,
	                       ADP1650_REG_CONTROL,
	 				       ADP1650_I_TX1_mA( 100 )
					       | ADP1650_I_TX2_mA( 100 ) );
	if( ret < 0 )
	{
		return ret;
	}

	/* Additional mode:
	 * - Strobe active high
	 * - Max current limit 1.5A
	 * - Input DC limit enabled
	 */
    ret = __adp1650_write( i2c,
	                       ADP1650_REG_AD_MODE,
	  				       ADP1650_STR_POL_ACTIVE_HIGH
						   | ADP1650_IL_DC_1A50
						   | ADP1650_IL_DC_EN );
	if( ret < 0 )
	{
		return ret;
	}

    return 0;
}

/* -------------------------------------------------------------------------- */

/** Configure the indicated mode */

int
adp1650_set_mode( i2c_device_number_t i2c, AP1650_Mode_t mode )
{
    return 0;
}

/* -------------------------------------------------------------------------- */

/** Set LED brightness  */

int
adp1650_set_current( i2c_device_number_t i2c, uint16_t current_mA )
{
	if( current_mA <= 1000 )
	{
    	return __adp1650_write( i2c,
	                            ADP1650_REG_CURRENT_SET,
	   		      	            ADP1650_I_FL_mA( current_mA )
				    		    | ADP1650_I_TOR_mA( 50 ) );
	}

    return -1;
}

/* -------------------------------------------------------------------------- */

/** Return the ADC reading of the indicated channel */

int
adp1650_get_adc( i2c_device_number_t  i2c,
                 AP1650_ADC_Channel_t adc_channel )
{
    /* Map ADC channel to register bit settings */
	uint8_t channel = 0;
	switch( adc_channel )
	{
		case ADC_CHANNEL_LED_VF:
			channel = ADP1650_ADC_LED_VF;
			break;
		case ADC_CHANNEL_TEMPERATURE:
			channel = ADP1650_ADC_DIE_TEMP;
			break;
		case ADC_CHANNEL_EXT_VOLTAGE:
			channel = ADP1650_ADC_EXT_VOLT;
			break;
	}

    int ret = __adp1650_write( i2c, ADP1650_REG_ADC, channel );

    if( ret >= 0 )
    {
		/* Allow 1ms for conversion */
	    mp_hal_delay_ms( 1 );

        /* Read ADC register again for result */
        uint8_t adc_val = 0;
        ret = __adp1650_read( i2c, ADP1650_REG_ADC, &adc_val );

        if( ret >= 0 )
        {
            return ADP1650_ADC_VAL( adc_val );
        }
    }

    return ret;
}

/* -------------------------------------------------------------------------- */

/** Return the current fault status mask */

int
adp1650_get_fault_status( i2c_device_number_t i2c )
{
    uint8_t fault_status = 0;

    int ret = __adp1650_read( i2c,
                              ADP1650_REG_FAULT,
                              &fault_status );

    if( ret >= 0 )
    {
        return fault_status;
    }

    return ret;
}

/* ----- End ---------------------------------------------------------------- */
