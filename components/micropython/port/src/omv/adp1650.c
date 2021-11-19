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

/** Sets the torch current in mA (25-200mA in 25mA increments) */
static int
__adp1650_set_torch_current( i2c_device_number_t i2c, uint8_t current )
{
    uint8_t val;
    int ret = __adp1650_read( i2c,
                              ADP1650_REG_CURRENT_SET,
                              &val );
    if( ret != 0 )
    {
        return ret;
    }
    ret = __adp1650_write( i2c,
                           ADP1650_REG_CURRENT_SET,
                           ( val & ~0x07 ) | ADP1650_I_TOR_mA( current ) );
    if( ret != 0 )
    {
        return ret;
    }
    return 0;
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
     * GPIO1 is torch control control
     * GPIO2 is ambient light ADC input
     * Flash timer configuration 1000 ms max
     */
    int ret = __adp1650_write( i2c,
                               ADP1650_REG_TIMER_IOCFG,
                               ADP1650_IOCFG_IO1_TORCH
                               | ADP1650_IOCFG_IO2_AIN
                               | ADP1650_FL_TIMER_ms( 1000 ) );
    if( ret < 0 )
    {
        return ret;
    }

    /* Flash default current 500mA, torch current 50mA */
    ret = __adp1650_write( i2c,
                           ADP1650_REG_CURRENT_SET,
                              ADP1650_I_FL_mA( 500 )
                           | ADP1650_I_TOR_mA( 50 ) );
    if( ret < 0 )
    {
        return ret;
    }

    /* Output mode:
     * - inductor peak 3.0A
     * - Strobe level sensitive
     * - No frequency fold back
     * - Output DISABLED
     * - Hardware strobe mode
     * - LED output mode STANDBY
     */
    ret = __adp1650_write( i2c,
                           ADP1650_REG_OUTPUT_MODE,
                           ADP1650_IL_PEAK_2A25
                           | ADP1650_STR_LV_LEVEL
                           | ADP1650_STR_MODE_HW
                           | ADP1650_LED_MODE_STBY );
    if( ret < 0 )
    {
        return ret;
    }

    /* TxMASK1 & Tx_MASK2 => not actually used */
    ret = __adp1650_write( i2c,
                           ADP1650_REG_CONTROL,
                            ADP1650_I_TX1_mA( 400 )
                           | ADP1650_I_TX2_mA( 400 ) );
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
                           | ADP1650_I_ILED_2mA75
                           | ADP1650_IL_DC_1A50
                           | ADP1650_IL_DC_EN );
    if( ret < 0 )
    {
        return ret;
    }

    /* Battery low mode:
     * Foll back LED current when VIN falls below 3.3V
     * (which should not happen in our Felixer camera)
     */
    ret = __adp1650_write( i2c,
                           ADP1650_REG_BATT_LOW,
                             ADP1650_CL_SOFT_EN
                           | ADP1650_I_VB_LO_mA( 400 )
                           | ADP1650_V_VB_LO_3V30 );
    if( ret < 0 )
    {
        return ret;
    }

    return 0;
}

/* -------------------------------------------------------------------------- */

/** Sets the output enable */

int
adp1650_set_output( i2c_device_number_t i2c, bool output )
{
    uint8_t val;
    int ret = __adp1650_read( i2c,
                              ADP1650_REG_AD_MODE,
                              &val );
    if( ret != 0 )
    {
        return ret;
    }
    if( output )
    {
        ret = __adp1650_write( i2c,
                               ADP1650_REG_AD_MODE,
                               val | ADP1650_OUTPUT_EN );
    }
    else
    {
        ret = __adp1650_write( i2c,
                               ADP1650_REG_AD_MODE,
                               val & ~ADP1650_OUTPUT_EN );
    }
    if( ret != 0 )
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
    int ret;
    switch( mode )
    {
        case FL_MODE_OFF:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_HW
                                   | ADP1650_LED_MODE_STBY );
        break;
        case FL_MODE_TORCH_25mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 25 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_SW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_50mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 50 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_SW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_75mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 75 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_SW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_100mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 100 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_SW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_125mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 125 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_SW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_150mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 150 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_SW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_175mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 175 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_SW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_200mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 200 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_SW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_TRIG_EXT_25mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 25 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_HW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_TRIG_EXT_50mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 50 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_HW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_TRIG_EXT_75mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 75 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_HW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_TRIG_EXT_100mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 100 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_HW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_TRIG_EXT_125mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 125 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_HW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_TRIG_EXT_150mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 150 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_HW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_TRIG_EXT_175mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 175 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_HW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_TORCH_TRIG_EXT_200mA:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 200 );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_HW
                                   | ADP1650_LED_MODE_ASSIST_LIGHT );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_FLASH:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_SW
                                   | ADP1650_LED_MODE_FLASH );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        case FL_MODE_FLASH_TRIG_EXT:
            ret = adp1650_set_output( i2c, false );
            if( ret != 0 )
            {
                return ret;
            }
            ret = __adp1650_set_torch_current( i2c, 25 );
            ret = __adp1650_write( i2c,
                                   ADP1650_REG_OUTPUT_MODE,
                                   ADP1650_IL_PEAK_2A25
                                   | ADP1650_STR_LV_LEVEL
                                   | ADP1650_STR_MODE_HW
                                   | ADP1650_LED_MODE_FLASH );
            if( ret != 0 )
            {
                return ret;
            }
            adp1650_set_output( i2c, true );
        break;
        default:
            return -1;
        break;
    }
    if( ret != 0 )
    {
        return ret;
    }
    return 0;
}

/* -------------------------------------------------------------------------- */

/** Set LED brightness  */

int
adp1650_set_current( i2c_device_number_t i2c, uint16_t current_mA )
{
    /* The LED we are using is limited to about 1A. Minimum for the flash
    current is 300mA. */
    if( ( current_mA <= 1000 ) && ( current_mA >= 300 ) )
    {
        return __adp1650_write( i2c,
                                ADP1650_REG_CURRENT_SET,
                                     ADP1650_I_FL_mA( current_mA )
                                | ADP1650_I_TOR_mA( 100 ) );
    }
    else if( current_mA < 300 )
    {
        return __adp1650_write( i2c,
                                ADP1650_REG_CURRENT_SET,
                                ADP1650_I_FL_mA( 300 )
                              | ADP1650_I_TOR_mA( 100 ) );
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
    uint8_t fault = 0;

    int ret = __adp1650_read( i2c,
                              ADP1650_REG_FAULT,
                              &fault );

    if( ret >= 0 )
    {
        mp_printf( &mp_plat_print,
                   "[adp1650]: fault 0x%x: %s%s%s%s%s%s%s%s\n",
                   fault,
                   fault & ADP1650_FL_OVP ? "FL_OVP " : "",
                   fault & ADP1650_FL_SC  ? "FL_SC "  : "",
                   fault & ADP1650_FL_OT  ? "FL_OT "  : "",
                   fault & ADP1650_FL_TO  ? "FL_TO "  : "",
                   fault & ADP1650_FL_TX1 ? "FL_TX1 " : "",
                   fault & ADP1650_FL_IO2 ? "FL_IO2 " : "",
                   fault & ADP1650_FL_IL  ? "FL_IL "  : "",
                   fault & ADP1650_FL_IDC ? "FL_IDC " : "");
        return fault;
    }

    return ret;
}

/* ----- End ---------------------------------------------------------------- */
