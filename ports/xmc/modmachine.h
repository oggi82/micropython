#ifndef MICROPY_INCLUDED_XMC_MODMACHINE_H
#define MICROPY_INCLUDED_XMC_MODMACHINE_H

#include "py/obj.h"


//extern const mp_obj_type_t machine_adc_type;
extern const mp_obj_type_t machine_timer_type;

void machine_init(void);
void machine_deinit(void);

MP_DECLARE_CONST_FUN_OBJ_VAR_BETWEEN(machine_info_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_unique_id_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_reset_obj);
MP_DECLARE_CONST_FUN_OBJ_VAR_BETWEEN(machine_bootloader_obj);
MP_DECLARE_CONST_FUN_OBJ_VAR_BETWEEN(machine_freq_obj);
MP_DECLARE_CONST_FUN_OBJ_VAR_BETWEEN(machine_lightsleep_obj);
MP_DECLARE_CONST_FUN_OBJ_VAR_BETWEEN(machine_deepsleep_obj);


#endif // MICROPY_INCLUDED_XMC_MODMACHINE_H
