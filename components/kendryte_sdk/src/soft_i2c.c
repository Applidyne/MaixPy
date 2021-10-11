
#include "i2c.h"
#include "soft_i2c.h"
#include "platform.h"
#include "sysctl.h"
#include "utils.h"
#include "math.h"
#include "fpioa.h"
#include "sleep.h"
#include "gpiohs.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

typedef struct
{
    uint8_t  gpiohs_scl;
    uint8_t  gpiohs_sda;
    uint8_t  address_width;
    uint32_t clk_speed;
    uint32_t bit_delay;
} soft_i2c_context_t;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

soft_i2c_context_t soft_i2c;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

static inline void SCL_HIGH( soft_i2c_context_t * ctx )
{
    gpiohs_set_pin( ctx->gpiohs_scl, GPIO_PV_HIGH );
}

static inline void SCL_LOW( soft_i2c_context_t * ctx )
{
    gpiohs_set_pin( ctx->gpiohs_scl, GPIO_PV_LOW );
}

static inline void SDA_HIGH( soft_i2c_context_t * ctx )
{
    gpiohs_set_drive_mode( ctx->gpiohs_sda, GPIO_DM_OUTPUT );
    gpiohs_set_pin( ctx->gpiohs_sda, GPIO_PV_HIGH );
}

static inline void SDA_LOW( soft_i2c_context_t * ctx )
{
    gpiohs_set_drive_mode( ctx->gpiohs_sda, GPIO_DM_OUTPUT );
    gpiohs_set_pin( ctx->gpiohs_sda, GPIO_PV_LOW );
}

static inline void SDA_INPUT_MODE( soft_i2c_context_t * ctx )
{
    gpiohs_set_drive_mode( ctx->gpiohs_sda, GPIO_DM_INPUT_PULL_UP );
}

static inline int SDA_READ( soft_i2c_context_t * ctx )
{
    return gpiohs_get_pin( ctx->gpiohs_sda );
}

static inline void I2C_BEGIN( soft_i2c_context_t * ctx )
{
    usleep( ctx->bit_delay/2 );
    SDA_LOW( ctx );
    usleep( ctx->bit_delay/2 );
    SCL_LOW( ctx );
}

static inline void I2C_END( soft_i2c_context_t * ctx )
{
    SDA_LOW( ctx );
    usleep( ctx->bit_delay/2 );
    SCL_HIGH( ctx );
    usleep( ctx->bit_delay/2 );
    SDA_HIGH( ctx );
    usleep( ctx->bit_delay/2 );
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/**
 * @return 1: ACK
 *         0: NAK
 */

int soft_i2c_byte_out( uint8_t b )
{
    for( int i = 0; i < 8; i++ )
    {
        if( b & 0x80 )
        {
            SDA_HIGH( &soft_i2c );
        }
        else
        {
            SDA_LOW( &soft_i2c );
        }
        usleep( soft_i2c.bit_delay/2 );
        SCL_HIGH( &soft_i2c );
        usleep( soft_i2c.bit_delay );
        SCL_LOW( &soft_i2c );
        b <<= 1;
    }

    // Read ACK
    SDA_INPUT_MODE( &soft_i2c );
    usleep( soft_i2c.bit_delay/2 );
    SCL_HIGH( &soft_i2c );
    usleep( soft_i2c.bit_delay/2 );
    int ack = SDA_READ( &soft_i2c );
    usleep( soft_i2c.bit_delay/2 );
    SCL_LOW( &soft_i2c );
    usleep( soft_i2c.bit_delay/2 );
    SDA_LOW( &soft_i2c );

    return (ack == 0) ? 1 : 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/**
 * @return 1: ACK
 *         0: NAK
 */

uint8_t soft_i2c_byte_in( bool last )
{
    uint8_t b = 0;

    SDA_INPUT_MODE( &soft_i2c );

    for( int i = 0; i < 8; i++ )
    {
        usleep( soft_i2c.bit_delay );
        SCL_HIGH( &soft_i2c );
        b <<= 1;
        if( SDA_READ( &soft_i2c ) )
        {
            b |= 1;
        }
        usleep( soft_i2c.bit_delay );
        SCL_LOW( &soft_i2c );
    }

    if( !last )
    {
        SDA_LOW( &soft_i2c );   /* Send ACK */
    }

    usleep( soft_i2c.bit_delay );
    SCL_HIGH( &soft_i2c );
    usleep( soft_i2c.bit_delay );
    SDA_INPUT_MODE( &soft_i2c );
    SCL_LOW( &soft_i2c );

    return b;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void soft_i2c_init( uint8_t  pin_scl,
                    uint8_t  pin_sda,
                    uint32_t address_width,
                    uint32_t i2c_clk_speed )
{
    configASSERT(address_width == 7 || address_width == 10);

    // pin_scl 41 -> gpiohs 25
    // pin_sda 40 -> gpiohs 24

    soft_i2c.gpiohs_scl = 25;  /* HACK */
    soft_i2c.gpiohs_sda = 24;  /* HACK */
    soft_i2c.address_width = address_width;
    soft_i2c.clk_speed = i2c_clk_speed;
    soft_i2c.bit_delay = 500000 / i2c_clk_speed;

    fpioa_set_function( pin_scl, FUNC_GPIOHS0 + soft_i2c.gpiohs_scl );
    fpioa_set_function( pin_sda, FUNC_GPIOHS0 + soft_i2c.gpiohs_sda );

    gpiohs_set_drive_mode( soft_i2c.gpiohs_scl, GPIO_DM_OUTPUT );
    gpiohs_set_drive_mode( soft_i2c.gpiohs_sda, GPIO_DM_OUTPUT );

    gpiohs_set_pin( soft_i2c.gpiohs_scl, GPIO_PV_HIGH );
    gpiohs_set_pin( soft_i2c.gpiohs_sda, GPIO_PV_HIGH );
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/**
 * @return 0: success
 *        <0: error
 */
int soft_i2c_send_data( uint32_t            slave_address,
                        const uint8_t *     send_buf,
                        size_t              send_buf_len,
                        uint16_t            timeout_ms )
{
    int ret = -1;

    I2C_BEGIN( &soft_i2c );

    if( soft_i2c_byte_out( ( slave_address << 1 ) ) ) /* Write */
    {
        for( int i = 0; i < send_buf_len; i++ )
        {
            ret = soft_i2c_byte_out( send_buf[i] );
            if( ret == 0 )   /* 1 == ACK */
            {
                ret = -1;
                break;
            }
        }
    }

    I2C_END( &soft_i2c );

    return ret;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/**
 * @return 0: success
 *        <0: error
 */

int soft_i2c_recv_data( uint32_t            slave_address,
                        const uint8_t *     send_buf,
                        size_t              send_buf_len,
                        uint8_t *           receive_buf,
                        size_t              receive_buf_len,
                        uint16_t            timeout_ms )
{
    int ret = -1;

    if( send_buf_len > 0 )
    {
        soft_i2c_send_data( slave_address, send_buf, send_buf_len, timeout_ms );
    }

    I2C_BEGIN( &soft_i2c );

    if( soft_i2c_byte_out( ( slave_address << 1 ) | 1 ) ) /* Read */
    {
        ret = 0;
        for( int i = 0; i < receive_buf_len; i++ )
        {
            receive_buf[i] = soft_i2c_byte_in( i == ( receive_buf_len - 1 ) );
        }
    }

    I2C_END( &soft_i2c );

    return ret;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void soft_i2c_deinit( i2c_device_number_t i2c_num )
{
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

