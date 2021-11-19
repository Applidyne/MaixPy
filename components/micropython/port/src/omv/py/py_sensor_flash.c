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

#include "mp.h"
#include "imlib.h"
#include "py_assert.h"
#include "py_sensor_flash.h"
#include "sensor_flash.h"
#include "omv_boardconfig.h"
#include "py_helper.h"
#include "mphalport.h"
#include "global_config.h"
#include "adp1650.h"

/* -------------------------------------------------------------------------- */

static mp_obj_t py_sensor_flash_setup( size_t n_args, const mp_obj_t *args, mp_map_t *kw_args )
{
    PY_ASSERT_FALSE_MSG( sensor_flash_init() != 0, "Sensor flash init failed!" );
    return mp_const_none;
}

/* -------------------------------------------------------------------------- */

static mp_obj_t py_sensor_flash_enable( mp_obj_t enable )
{
    PY_ASSERT_FALSE_MSG( sensor_flash_enable( mp_obj_get_int( enable ) ) != 0, "Sensor flash enable failed!" );;
    return mp_const_none;
}

/* -------------------------------------------------------------------------- */

static mp_obj_t py_sensor_flash_torch( mp_obj_t enable )
{
    sensor_flash_torch( mp_obj_get_int( enable ) );
    return mp_const_none;
}

/* -------------------------------------------------------------------------- */

static mp_obj_t py_sensor_flash_output( mp_obj_t enable )
{
    sensor_flash_output( mp_obj_get_int( enable ) );
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

static mp_obj_t py_sensor_flash_get_temperature( void )
{
    return mp_obj_new_int( sensor_flash_get_temperature() );
}

/* -------------------------------------------------------------------------- */

static mp_obj_t py_sensor_flash_get_fault( void )
{
    return mp_obj_new_int( sensor_flash_get_fault() );
}

/* -------------------------------------------------------------------------- */

static mp_obj_t py_sensor_flash_set_mode( size_t n_args, const mp_obj_t *args, mp_map_t *kw_args )
{
    if(n_args != 1)
    {
        mp_raise_ValueError( "Flash Set Mode Arg Err" );
    }

    int ret = sensor_flash_set_mode( mp_obj_get_int( args[0] ) );
    if( ret == -1 )
    {
        PY_ASSERT_TRUE_MSG( 0, "Flash mode not supported" );
    }
    else if ( ret != 0 )
    {
        PY_ASSERT_TRUE_MSG( 0, "Set Flash Mode Error" );
    }
    return 0;
}

/* -------------------------------------------------------------------------- */

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(py_sensor_flash_setup_obj, 0,       py_sensor_flash_setup);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_sensor_flash_enable_obj,          py_sensor_flash_enable);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_sensor_flash_torch_obj,           py_sensor_flash_torch);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_sensor_flash_output_obj,          py_sensor_flash_output);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_sensor_flash_set_current_obj,     py_sensor_flash_set_current);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_sensor_flash_get_current_obj,     py_sensor_flash_get_current);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_sensor_flash_get_ambient_obj,     py_sensor_flash_get_ambient);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_sensor_flash_get_temperature_obj, py_sensor_flash_get_temperature);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_sensor_flash_get_fault_obj,       py_sensor_flash_get_fault);
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(py_sensor_flash_set_mode_obj, 1,    py_sensor_flash_set_mode);

STATIC const mp_map_elem_t sensor_flash_module_globals_table[] =
{
    // ModuleFlash functions
    {MP_OBJ_NEW_QSTR(MP_QSTR___name__),         MP_OBJ_NEW_QSTR(MP_QSTR_sensor_flash)},

    // Flash Modes
    {MP_OBJ_NEW_QSTR(MP_QSTR_OFF),              MP_OBJ_NEW_SMALL_INT(FL_MODE_OFF)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_25),         MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_25mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_50),         MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_50mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_75),         MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_75mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_100),        MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_100mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_125),        MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_125mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_150),        MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_150mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_175),        MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_175mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_200),        MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_200mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_EXT_25),     MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_TRIG_EXT_25mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_EXT_50),     MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_TRIG_EXT_50mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_EXT_75),     MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_TRIG_EXT_75mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_EXT_100),    MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_TRIG_EXT_100mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_EXT_125),    MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_TRIG_EXT_125mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_EXT_150),    MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_TRIG_EXT_150mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_EXT_175),    MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_TRIG_EXT_175mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_TORCH_EXT_200),    MP_OBJ_NEW_SMALL_INT(FL_MODE_TORCH_TRIG_EXT_200mA)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_FLASH),            MP_OBJ_NEW_SMALL_INT(FL_MODE_FLASH)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_FLASH_EXT),        MP_OBJ_NEW_SMALL_INT(FL_MODE_FLASH_TRIG_EXT)},

    // Flash functions
    {MP_OBJ_NEW_QSTR(MP_QSTR_setup),           (mp_obj_t)&py_sensor_flash_setup_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_enable),          (mp_obj_t)&py_sensor_flash_enable_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_torch),           (mp_obj_t)&py_sensor_flash_torch_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_output),          (mp_obj_t)&py_sensor_flash_output_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_set_current),     (mp_obj_t)&py_sensor_flash_set_current_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_get_current),     (mp_obj_t)&py_sensor_flash_get_current_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_get_ambient),     (mp_obj_t)&py_sensor_flash_get_ambient_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_get_temperature), (mp_obj_t)&py_sensor_flash_get_temperature_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_get_fault),       (mp_obj_t)&py_sensor_flash_get_fault_obj},
    {MP_OBJ_NEW_QSTR(MP_QSTR_set_mode),        (mp_obj_t)&py_sensor_flash_set_mode_obj},
};

/* -------------------------------------------------------------------------- */

STATIC MP_DEFINE_CONST_DICT(sensor_flash_module_globals, sensor_flash_module_globals_table);

const mp_obj_module_t sensor_flash_module = {
    .base    = {&mp_type_module},
    .globals = (mp_obj_t)&sensor_flash_module_globals,
};

/* -------------------------------------------------------------------------- */
