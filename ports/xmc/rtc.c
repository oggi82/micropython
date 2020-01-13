/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>

#include "py/runtime.h"
//#include "extint.h"
#include "lib/timeutils/timeutils.h"
#include "rtc.h"
#include "irq.h"
#include "xmc_scu.h"
#include "xmc_rtc.h"


XMC_RTC_CONFIG_t rtc_config =
{
  .alarm.minutes = 1U,
  .prescaler = 0x7fffU
};

/// \moduleref machine
/// \class RTC - real time clock
///
/// The RTC is and independent clock that keeps track of the date
/// and time.
///
/// Example usage:
///
///     rtc = pyb.RTC()
///     rtc.datetime((2014, 5, 1, 4, 13, 0, 0, 0))
///     print(rtc.datetime())

XMC_RTC_CONFIG_t RTCHandle;

// rtc_info indicates various things about RTC startup
// it's a bit of a hack at the moment
static mp_uint_t rtc_info;

//STATIC void MACHINE_RTC_Init(XMC_RTC_TIME_t *hrtc);
//STATIC void RTC_CalendarConfig(void);

//STATIC uint32_t rtc_startup_tick;
STATIC bool rtc_need_init_finalise = false;

void rtc_init_start(bool force_init) {
    XMC_RTC_STATUS_t status;
    (void)status;
    XMC_RTC_Init(&rtc_config);
}

void rtc_init_finalise() {
    if (!rtc_need_init_finalise) {
        return;
    }
    rtc_info = 0;
    XMC_RTC_Start();
}

// STATIC void RTC_CalendarConfig(void) {
//     // set the date to 1st Jan 2015
//     XMC_RTC_TIME_t date;
//     date.year = 15;
//     date.month = 1;
//     date.days = 1;
//     date.daysofweek = XMC_RTC_WEEKDAY_THURSDAY;
//     date.hours = 0;
//     date.minutes = 0;
//     date.seconds = 0;

//     XMC_RTC_SetTime(&date);
// }

/******************************************************************************/
// MicroPython bindings

typedef struct _machine_rtc_obj_t {
    mp_obj_base_t base;
} machine_rtc_obj_t;

STATIC const machine_rtc_obj_t machine_rtc_obj = {{&machine_rtc_type}};

/// \classmethod \constructor()
/// Create an RTC object.
STATIC mp_obj_t machine_rtc_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 0, 0, false);

    // return constant object
    return MP_OBJ_FROM_PTR(&machine_rtc_obj);
}

// force rtc to re-initialise
mp_obj_t machine_rtc_init(mp_obj_t self_in) {
    rtc_init_start(true);
    rtc_init_finalise();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(machine_rtc_init_obj, machine_rtc_init);

/// \method info()
/// Get information about the startup time and reset source.
///
///  - The lower 0xffff are the number of milliseconds the RTC took to
///    start up.
///  - Bit 0x10000 is set if a power-on reset occurred.
///  - Bit 0x20000 is set if an external reset occurred
mp_obj_t machine_rtc_info(mp_obj_t self_in) {
    return mp_obj_new_int(rtc_info);
}
MP_DEFINE_CONST_FUN_OBJ_1(machine_rtc_info_obj, machine_rtc_info);

/// \method datetime([datetimetuple])
/// Get or set the date and time of the RTC.
///
/// With no arguments, this method returns an 7-tuple with the current
/// date and time.  With 1 argument (being an 7-tuple) it sets the date
/// and time.
///
/// The 7-tuple has the following format:
///
///     (year, month, day, weekday, hours, minutes, seconds)
///
/// `weekday` is 1-7 for Monday through Sunday.
mp_obj_t machine_rtc_datetime(size_t n_args, const mp_obj_t *args) {
    rtc_init_finalise();
    if (n_args == 1) {
        // get date and time
        // note: need to call get time then get date to correctly access the registers
        XMC_RTC_TIME_t _time;
        XMC_RTC_GetTime(&_time);
        mp_obj_t tuple[8] = {
            mp_obj_new_int(2000 + _time.year),
            mp_obj_new_int(_time.month),
            mp_obj_new_int(_time.days),
            mp_obj_new_int(_time.hours),
            mp_obj_new_int(_time.minutes),
            mp_obj_new_int(_time.seconds),
            mp_obj_new_int(_time.daysofweek - 1),
            mp_obj_new_int(timeutils_year_day(2000 + _time.year, _time.month, _time.days)),
        
        };
        return mp_obj_new_tuple(8, tuple);
    } else {
        // set date and time
        mp_obj_t *items;
        mp_obj_get_array_fixed_n(args[1], 8, &items);

        XMC_RTC_TIME_t _time;
        _time.year = mp_obj_get_int(items[0]) - 2000;
        _time.month = mp_obj_get_int(items[1]);
        _time.days = mp_obj_get_int(items[2]);
        _time.daysofweek = mp_obj_get_int(items[3]);
        _time.hours = mp_obj_get_int(items[4]);
        _time.minutes = mp_obj_get_int(items[5]);
        _time.seconds = mp_obj_get_int(items[6]);
        XMC_RTC_Disable();
        XMC_RTC_SetTime(&_time);
        XMC_RTC_Enable();
        XMC_RTC_Start();        
        return mp_const_none;
    }
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_rtc_datetime_obj, 1, 2, machine_rtc_datetime);

// wakeup(None)
// wakeup(ms, callback=None)
// wakeup(wucksel, wut, callback)
mp_obj_t machine_rtc_wakeup(size_t n_args, const mp_obj_t *args) {
    // wut is wakeup counter start value, wucksel is clock source
    // counter is decremented at wucksel rate, and wakes the MCU when it gets to 0
    // wucksel=0b000 is RTC/16 (RTC runs at 32768Hz)
    // wucksel=0b001 is RTC/8
    // wucksel=0b010 is RTC/4
    // wucksel=0b011 is RTC/2
    // wucksel=0b100 is 1Hz clock
    // wucksel=0b110 is 1Hz clock with 0x10000 added to wut
    // so a 1 second wakeup could be wut=2047, wucksel=0b000, or wut=4095, wucksel=0b001, etc

    rtc_init_finalise();

    bool enable = false;
    mp_int_t wucksel;
    mp_int_t wut;
    mp_obj_t callback = mp_const_none;
    if (n_args <= 3) {
        if (args[1] == mp_const_none) {
            // disable wakeup
        } else {
            // time given in ms
            mp_int_t ms = mp_obj_get_int(args[1]);
            mp_int_t div = 2;
            wucksel = 3;
            while (div <= 16 && ms > 2000 * div) {
                div *= 2;
                wucksel -= 1;
            }
            if (div <= 16) {
                wut = 32768 / div * ms / 1000;
            } else {
                // use 1Hz clock
                wucksel = 4;
                wut = ms / 1000;
                if (wut > 0x10000) {
                    // wut too large for 16-bit register, try to offset by 0x10000
                    wucksel = 6;
                    wut -= 0x10000;
                    if (wut > 0x10000) {
                        // wut still too large
                        mp_raise_ValueError("wakeup value too large");
                    }
                }
            }
            // wut register should be 1 less than desired value, but guard against wut=0
            if (wut > 0) {
                wut -= 1;
            }
            enable = true;
        }
        if (n_args == 3) {
            callback = args[2];
        }
    } else {
        // config values given directly
        wucksel = mp_obj_get_int(args[1]);
        wut = mp_obj_get_int(args[2]);
        callback = args[3];
        enable = true;
    }
    (void)callback;
    // set the callback
 //   MP_STATE_PORT(pyb_extint_callback)[EXTI_RTC_WAKEUP] = callback;

    if (enable) {
    } else {
    }

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_rtc_wakeup_obj, 2, 4, machine_rtc_wakeup);

STATIC const mp_rom_map_elem_t machine_rtc_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&machine_rtc_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&machine_rtc_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_datetime), MP_ROM_PTR(&machine_rtc_datetime_obj) },
    { MP_ROM_QSTR(MP_QSTR_wakeup), MP_ROM_PTR(&machine_rtc_wakeup_obj) },
};
STATIC MP_DEFINE_CONST_DICT(machine_rtc_locals_dict, machine_rtc_locals_dict_table);

const mp_obj_type_t machine_rtc_type = {
    { &mp_type_type },
    .name = MP_QSTR_RTC,
    .make_new = machine_rtc_make_new,
    .locals_dict = (mp_obj_dict_t*)&machine_rtc_locals_dict,
};
