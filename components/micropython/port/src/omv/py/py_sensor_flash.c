/**
 * \file    py_sensor_flash.c
 *
 * \brief   Micropython wrapper for sensor IR flash control driver
 *
 * \author  Marco Hess <marcoh@applidyne.com.au>
 *
 * \date    28/10/2021
 */

#include "mp.h"
#include "imlib.h"
#include "py_assert.h"
#include "py_sensor_flash.h"
#include "sensor_flash.h"
#include "omv_boardconfig.h"
#include "py_helper.h"
#include "framebuffer.h"
#include "mphalport.h"
#include "global_config.h"

/* -------------------------------------------------------------------------- */

static mp_obj_t py_sensor_flash_reset( size_t n_args, const mp_obj_t *args, mp_map_t *kw_args )
{
    PY_ASSERT_FALSE_MSG( sensor_flash_reset() != 0, "Sensor flash reset failed!" );
    return mp_const_none;
}

/* -------------------------------------------------------------------------- */

static mp_obj_t py_sensor_flash_enable( mp_obj_t enable )
{
    sensor_flash_enable( mp_obj_get_int( enable ) );
    return mp_const_none;
}

/* -------------------------------------------------------------------------- */

static mp_obj_t py_sensor_flash_set_current( mp_obj_t current )
{
    sensor_flash_set_current( mp_obj_get_int( current ) );
    return mp_const_none;
}

/* -------------------------------------------------------------------------- */

static mp_obj_t py_sensor_flash_get_current( void )
{
    return mp_obj_new_int( sensor_flash_get_current() );
}

/* -------------------------------------------------------------------------- */

static mp_obj_t py_sensor_flash_get_ambient( void )
{
    return mp_obj_new_int( sensor_flash_get_ambient() );
}

/* -------------------------------------------------------------------------- */

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(py_sensor_flash_reset_obj, 0,   py_sensor_flash_reset);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_sensor_flash_enable_obj,      py_sensor_flash_enable);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_sensor_flash_set_current_obj, py_sensor_flash_set_current);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_sensor_flash_get_current_obj, py_sensor_flash_get_current);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_sensor_flash_get_ambient_obj, py_sensor_flash_get_ambient);

STATIC const mp_map_elem_t sensor_flash_module_globals_table[] =
{
    // ModuleFlash functions
    {MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_sensor_flash)},

    // Flash functions
    {MP_OBJ_NEW_QSTR(MP_QSTR_reset),       (mp_obj_t)&py_sensor_flash_reset_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_enable),      (mp_obj_t)&py_sensor_flash_enable_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_set_current), (mp_obj_t)&py_sensor_flash_set_current_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_get_current), (mp_obj_t)&py_sensor_flash_get_current_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_get_ambient), (mp_obj_t)&py_sensor_flash_get_ambient_obj},
};

/* -------------------------------------------------------------------------- */

STATIC MP_DEFINE_CONST_DICT(sensor_flash_module_globals, sensor_flash_module_globals_table);

const mp_obj_module_t sensor_flash_module = {
    .base    = {&mp_type_module},
    .globals = (mp_obj_t)&sensor_flash_module_globals,
};

/* -------------------------------------------------------------------------- */
