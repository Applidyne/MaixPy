/**
 * \file    py_sensor_flash.c
 *
 * \brief   Micropython wrapper for sensor IR flash control driver
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
