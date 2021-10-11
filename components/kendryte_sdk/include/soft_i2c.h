#ifndef __SOFT_I2C_H__
#define __SOFT_I2C_H__

#include "i2c.h"

void soft_i2c_init( uint8_t   pin_scl,
                    uint8_t   pin_sda,
                    uint32_t  address_width,
                    uint32_t  i2c_clk_speed );

int soft_i2c_send_data( uint32_t            slave_address,
                        const uint8_t *     send_buf,
                        size_t              send_buf_len,
                        uint16_t            timeout_ms );

int soft_i2c_recv_data( uint32_t            slave_address,
                        const uint8_t *     send_buf,
                        size_t              send_buf_len,
                        uint8_t *           receive_buf,
                        size_t              receive_buf_len,
                        uint16_t            timeout_ms);

void soft_i2c_deinit( i2c_device_number_t i2c_num );

#endif //__SOFT_I2C_H__
