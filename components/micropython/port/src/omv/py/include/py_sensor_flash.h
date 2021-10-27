/**
 * \file    py_sensor_flash.c
 *
 * \brief   Micropython wrapper for sensor IR flash control driver
 *
 *          Example usage:
 *
 *              sensor_flash.reset()
 *              print( sensor_flash.get_current() )
 *              sensor_flash.set_current( 25 )
 *              print( sensor_flash.get_current() )
 *              print( sensor_flash.get_ambient() )
 *              sensor_flash.enable( True )
 *              sensor_flash.enable( False )
 *
 * \author  Marco Hess <marcoh@applidyne.com.au>
 *
 * \date    28/10/2021
 */

/* -------------------------------------------------------------------------- */

#ifndef __PY_SENSOR_FLASH_H__
#define __PY_SENSOR_FLASH_H__

/* MH Not clear where the py_sensor_flash_init is or comes from */
const mp_obj_module_t *py_sensor_flash_init();

#endif // __PY_SENSOR_FLASH_H__
