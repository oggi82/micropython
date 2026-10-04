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

#include <stdint.h>
#include <string.h>

#include "py/obj.h"
#include "py/runtime.h"
#include "py/gc.h"
#include "timer.h"
#include "pin.h"
#include "irq.h"
#include "xmc_ccu4.h"
#include "xmc_ccu8.h"
#include "xmc_scu.h"

/// \moduleref machine
/// \class Timer - generate/measure signals using the CCU4/CCU8 peripheral
///
/// Unlike STM32's TIMx (one counter shared by up to 4 compare channels),
/// every CCU4/CCU8 *slice* is already a complete, independent timer with its
/// own counter/period/prescaler. So here, one `machine.Timer` id addresses
/// exactly one slice, not a whole CCU4/CCU8 module:
///
///   - Timer(0..15)  -> CCU40.0 .. CCU43.3  (4 modules x 4 slices)
///   - Timer(16..23) -> CCU80.0 .. CCU81.3  (2 modules x 4 slices)
///
/// A CCU4 slice has one compare output and one capture input, so it only
/// ever has `channel(1, ...)`. A CCU8 slice has two independent compare
/// channels (and correspondingly up to 4 physical output pins, two per
/// channel for normal/inverted polarity), so both `channel(1, ...)` and
/// `channel(2, ...)` are valid there, each on whichever pin the board wires
/// up for that channel.
///
///     tim = machine.Timer(16)                       # CCU80 slice 0
///     tim.init(freq=1000)
///     ch = tim.channel(1, machine.Timer.PWM, pin=machine.Pin.board.P1_15, pulse_width_percent=25)

typedef enum {
    CHANNEL_MODE_PWM_NORMAL,
    CHANNEL_MODE_PWM_INVERTED,
    CHANNEL_MODE_IC,
} pyb_channel_mode;

enum {
    TIMER_RISING = 0,
    TIMER_FALLING,
    TIMER_BOTH,
};

// Flat slice-id -> (module, slice) lookup tables. See TIMER_ID_CCU4x()/
// TIMER_ID_CCU8x() in pin_defs_xmc.h for how ids 0..23 map here.
static XMC_CCU4_MODULE_t *const ccu4_module_tbl[4] = {CCU40, CCU41, CCU42, CCU43};
static XMC_CCU4_SLICE_t *const ccu4_slice_tbl[4][4] = {
    {CCU40_CC40, CCU40_CC41, CCU40_CC42, CCU40_CC43},
    {CCU41_CC40, CCU41_CC41, CCU41_CC42, CCU41_CC43},
    {CCU42_CC40, CCU42_CC41, CCU42_CC42, CCU42_CC43},
    {CCU43_CC40, CCU43_CC41, CCU43_CC42, CCU43_CC43},
};
static XMC_CCU8_MODULE_t *const ccu8_module_tbl[2] = {CCU80, CCU81};
static XMC_CCU8_SLICE_t *const ccu8_slice_tbl[2][4] = {
    {CCU80_CC80, CCU80_CC81, CCU80_CC82, CCU80_CC83},
    {CCU81_CC80, CCU81_CC81, CCU81_CC82, CCU81_CC83},
};
static const IRQn_Type timer_irqn_tbl[MICROPY_HW_MAX_TIMER] = {
    CCU40_0_IRQn, CCU40_1_IRQn, CCU40_2_IRQn, CCU40_3_IRQn,
    CCU41_0_IRQn, CCU41_1_IRQn, CCU41_2_IRQn, CCU41_3_IRQn,
    CCU42_0_IRQn, CCU42_1_IRQn, CCU42_2_IRQn, CCU42_3_IRQn,
    CCU43_0_IRQn, CCU43_1_IRQn, CCU43_2_IRQn, CCU43_3_IRQn,
    CCU80_0_IRQn, CCU80_1_IRQn, CCU80_2_IRQn, CCU80_3_IRQn,
    CCU81_0_IRQn, CCU81_1_IRQn, CCU81_2_IRQn, CCU81_3_IRQn,
};

typedef struct _pyb_timer_channel_obj_t {
    mp_obj_base_t base;
    struct _pyb_timer_obj_t *timer;
    uint8_t channel;  // 1 or 2 (CCU4 slices only ever use 1)
    uint8_t mode;     // pyb_channel_mode
    mp_obj_t callback;
    struct _pyb_timer_channel_obj_t *next;
} pyb_timer_channel_obj_t;

typedef struct _pyb_timer_obj_t {
    mp_obj_base_t base;
    uint8_t tim_id;      // flat slice id, 0..MICROPY_HW_MAX_TIMER-1
    bool is_ccu8;
    uint8_t module_idx;  // 0..3 (CCU4) or 0..1 (CCU8)
    uint8_t slice_idx;   // 0..3
    union {
        XMC_CCU4_MODULE_t *ccu4;
        XMC_CCU8_MODULE_t *ccu8;
    } module;
    union {
        XMC_CCU4_SLICE_t *ccu4;
        XMC_CCU8_SLICE_t *ccu8;
    } slice;
    uint32_t period;  // cached period register value (ticks - 1)
    pyb_timer_channel_obj_t *channel;
} pyb_timer_obj_t;

#define PYB_TIMER_OBJ_ALL_NUM MP_ARRAY_SIZE(MP_STATE_PORT(pyb_timer_obj_all))

static const mp_obj_type_t pyb_timer_channel_type;

static mp_obj_t machine_timer_deinit(mp_obj_t self_in);
static mp_obj_t machine_timer_channel_callback(mp_obj_t self_in, mp_obj_t callback);
static void timer_channel_stop(pyb_timer_channel_obj_t *chan);

void timer_init0(void) {
    for (uint i = 0; i < PYB_TIMER_OBJ_ALL_NUM; i++) {
        MP_STATE_PORT(pyb_timer_obj_all)[i] = NULL;
    }
}

// unregister all interrupt sources
void timer_deinit(void) {
    for (uint i = 0; i < PYB_TIMER_OBJ_ALL_NUM; i++) {
        pyb_timer_obj_t *tim = MP_STATE_PORT(pyb_timer_obj_all)[i];
        if (tim != NULL) {
            machine_timer_deinit(MP_OBJ_FROM_PTR(tim));
        }
    }
}

/******************************************************************************/
/* MicroPython bindings                                                       */

// Module-level XMC_CCU4_Init()/XMC_CCU8_Init() + StartPrescaler() must only
// run once per physical module, not once per slice (every Timer id that
// maps to the same module would otherwise re-run it and disturb slices that
// are already configured).
static bool ccu4_module_inited[4];
static bool ccu8_module_inited[2];

static uint32_t timer_source_freq(pyb_timer_obj_t *self) {
    (void)self;
    // CCU4 and CCU8 are both fed from the same fCCU clock domain on XMC4500.
    return XMC_SCU_CLOCK_GetCcuClockFrequency();
}

// Computes a (power-of-2 prescaler exponent, period) pair so the slice's
// timer triggers at freq-Hz. CCU4/8 only offer a binary 1..32768 prescaler
// (XMC_CCU4_SLICE_PRESCALER_t), unlike STM32's arbitrary 16-bit divider.
static uint32_t compute_prescaler_period_from_freq(pyb_timer_obj_t *self, mp_obj_t freq_in, uint32_t *period_out) {
    uint32_t source_freq = timer_source_freq(self);
    mp_float_t freq;
    #if MICROPY_PY_BUILTINS_FLOAT
    freq = mp_obj_get_float(freq_in);
    #else
    freq = mp_obj_get_int(freq_in);
    #endif
    if (freq <= 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("must have positive freq"));
    }

    uint32_t prescaler_exp = 0;
    uint64_t period;
    for (;;) {
        period = (uint64_t)((mp_float_t)source_freq / ((mp_float_t)(1UL << prescaler_exp) * freq));
        if (period <= 0x10000 || prescaler_exp >= 15) {
            break;
        }
        prescaler_exp++;
    }
    if (period < 1) {
        period = 1;
    }
    if (period > 0x10000) {
        period = 0x10000;
    }
    *period_out = (uint32_t)(period - 1) & 0xffff;
    return prescaler_exp;
}

static void pyb_timer_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pyb_timer_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "Timer(%u)", self->tim_id);
}

// Shared by init(), freq() and make_new(): (re)configure this slice's
// prescaler and period, without touching any channel already running on it.
static void timer_apply_prescaler_period(pyb_timer_obj_t *self, uint32_t prescaler_exp, uint32_t period) {
    if (prescaler_exp > 15) {
        mp_raise_ValueError(MP_ERROR_TEXT("prescaler out of range"));
    }
    self->period = period & 0xffff;

    if (self->is_ccu8) {
        XMC_CCU8_SLICE_COMPARE_CONFIG_t cfg = {0};
        cfg.timer_mode = XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA;
        cfg.monoshot = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT;
        cfg.prescaler_initval = prescaler_exp;
        cfg.mcm_ch1_enable = true;
        cfg.mcm_ch2_enable = true;
        XMC_CCU8_SLICE_CompareInit(self->slice.ccu8, &cfg);
        XMC_CCU8_SLICE_SetTimerPeriodMatch(self->slice.ccu8, (uint16_t)self->period);
        XMC_CCU8_EnableShadowTransfer(self->module.ccu8, (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0 << self->slice_idx));
    } else {
        XMC_CCU4_SLICE_COMPARE_CONFIG_t cfg = {0};
        cfg.timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA;
        cfg.monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT;
        cfg.prescaler_initval = prescaler_exp;
        XMC_CCU4_SLICE_CompareInit(self->slice.ccu4, &cfg);
        XMC_CCU4_SLICE_SetTimerPeriodMatch(self->slice.ccu4, (uint16_t)self->period);
        XMC_CCU4_EnableShadowTransfer(self->module.ccu4, (uint32_t)(XMC_CCU4_SHADOW_TRANSFER_SLICE_0 << self->slice_idx));
    }
}

// init() and make_new(): (re)configure this slice's prescaler and period
// from freq=/prescaler=+period= kwargs.
static mp_obj_t machine_timer_init_helper(pyb_timer_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_freq, MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_prescaler, MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_period, MP_ARG_INT, {.u_int = -1} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t prescaler_exp;
    uint32_t period;
    if (args[0].u_obj != mp_const_none) {
        prescaler_exp = compute_prescaler_period_from_freq(self, args[0].u_obj, &period);
    } else if (args[1].u_int >= 0 && args[2].u_int >= 0) {
        prescaler_exp = (uint32_t)args[1].u_int;
        period = (uint32_t)args[2].u_int;
    } else {
        // No frequency/period given: leave the slice running at its
        // current (or power-on default) rate.
        return mp_const_none;
    }
    timer_apply_prescaler_period(self, prescaler_exp, period);
    return mp_const_none;
}

/// \classmethod \constructor(id, ...)
/// Construct a new timer object addressing the given CCU4/CCU8 slice.  If
/// additional arguments are given, the timer is initialised by `init(...)`.
/// `id` can be 0 to 23 (0..15 -> CCU40.0..CCU43.3, 16..23 -> CCU80.0..CCU81.3).
static mp_obj_t pyb_timer_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    mp_int_t tim_id = mp_obj_get_int(args[0]);
    if (tim_id < 0 || tim_id >= MICROPY_HW_MAX_TIMER) {
        mp_raise_ValueError(MP_ERROR_TEXT("Timer doesn't exist"));
    }

    pyb_timer_obj_t *tim;
    if (MP_STATE_PORT(pyb_timer_obj_all)[tim_id] == NULL) {
        tim = m_new_obj(pyb_timer_obj_t);
        memset(tim, 0, sizeof(*tim));
        tim->base.type = &machine_timer_type;
        tim->tim_id = tim_id;
        tim->is_ccu8 = TIMER_ID_IS_CCU8(tim_id);

        if (tim->is_ccu8) {
            mp_uint_t id8 = tim_id - 16;
            tim->module_idx = id8 / 4;
            tim->slice_idx = id8 % 4;
            tim->module.ccu8 = ccu8_module_tbl[tim->module_idx];
            tim->slice.ccu8 = ccu8_slice_tbl[tim->module_idx][tim->slice_idx];
            if (!ccu8_module_inited[tim->module_idx]) {
                XMC_CCU8_Init(tim->module.ccu8, XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
                XMC_CCU8_StartPrescaler(tim->module.ccu8);
                ccu8_module_inited[tim->module_idx] = true;
            }
            XMC_CCU8_EnableClock(tim->module.ccu8, tim->slice_idx);
        } else {
            tim->module_idx = tim_id / 4;
            tim->slice_idx = tim_id % 4;
            tim->module.ccu4 = ccu4_module_tbl[tim->module_idx];
            tim->slice.ccu4 = ccu4_slice_tbl[tim->module_idx][tim->slice_idx];
            if (!ccu4_module_inited[tim->module_idx]) {
                XMC_CCU4_Init(tim->module.ccu4, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
                XMC_CCU4_StartPrescaler(tim->module.ccu4);
                ccu4_module_inited[tim->module_idx] = true;
            }
            XMC_CCU4_EnableClock(tim->module.ccu4, tim->slice_idx);
        }
        MP_STATE_PORT(pyb_timer_obj_all)[tim_id] = tim;
    } else {
        tim = MP_STATE_PORT(pyb_timer_obj_all)[tim_id];
    }

    if (n_args > 1 || n_kw > 0) {
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        machine_timer_init_helper(tim, n_args - 1, args + 1, &kw_args);
    }

    return MP_OBJ_FROM_PTR(tim);
}

static mp_obj_t machine_timer_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    return machine_timer_init_helper(MP_OBJ_TO_PTR(args[0]), n_args - 1, args + 1, kw_args);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(machine_timer_init_obj, 1, machine_timer_init);

// timer.deinit(): stop just this slice (other slices of the same physical
// module, i.e. other Timer ids, are left running).
static mp_obj_t machine_timer_deinit(mp_obj_t self_in) {
    pyb_timer_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pyb_timer_channel_obj_t *chan = self->channel;
    self->channel = NULL;
    while (chan != NULL) {
        pyb_timer_channel_obj_t *next = chan->next;
        timer_channel_stop(chan);
        chan->next = NULL;
        chan = next;
    }

    if (self->is_ccu8) {
        XMC_CCU8_SLICE_StopTimer(self->slice.ccu8);
    } else {
        XMC_CCU4_SLICE_StopTimer(self->slice.ccu4);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_deinit_obj, machine_timer_deinit);

// Looks up this pin's AF entry for the given Timer, restricted to either
// output (PWM) or input (capture) type entries -- pin_find_af() alone can't
// disambiguate when a pin has both (e.g. P3.0, P3.4).
static const pin_af_obj_t *find_timer_af(const pin_obj_t *pin, pyb_timer_obj_t *self, bool want_output) {
    uint8_t fn = self->is_ccu8 ? AF_FN_TIM8 : AF_FN_TIM;
    for (mp_uint_t i = 0; i < pin->num_af; i++) {
        const pin_af_obj_t *af = &pin->af[i];
        if (af->fn != fn || af->unit != self->tim_id) {
            continue;
        }
        bool is_in = self->is_ccu8 ? (af->type == AF_PIN_TYPE_TIM8_IN) : (af->type == AF_PIN_TYPE_TIM_IN);
        if (is_in != want_output) {
            return af;
        }
    }
    return NULL;
}

static void timer_channel_configure_pwm_pin(pyb_timer_obj_t *self, pyb_timer_channel_obj_t *chan, const pin_obj_t *pin, uint8_t polarity) {
    const pin_af_obj_t *af = find_timer_af(pin, self, true);
    if (af == NULL) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("Pin(%q) has no PWM output for Timer(%d)"), pin->name, self->tim_id);
    }

    uint8_t expect_channel = self->is_ccu8 ? AF_TIM8_OUT_CHANNEL(af->type) : 1;
    if (expect_channel != chan->channel) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("Pin(%q) is channel %d of Timer(%d), not channel %d"),
            pin->name, expect_channel, self->tim_id, chan->channel);
    }

    XMC_GPIO_SetMode(pin->gpio, pin->pin, (XMC_GPIO_MODE_t)(XMC_GPIO_MODE_OUTPUT_PUSH_PULL | (af->idx << PORT0_IOCR0_PC0_Pos)));

    XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_t level4 =
        polarity == CHANNEL_MODE_PWM_INVERTED ? XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH : XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW;
    if (self->is_ccu8) {
        XMC_CCU8_SLICE_OUTPUT_t out = (XMC_CCU8_SLICE_OUTPUT_t)(1U << AF_TIM8_OUT_N(af->type));
        XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_t level8 =
            polarity == CHANNEL_MODE_PWM_INVERTED ? XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH : XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW;
        XMC_CCU8_SLICE_SetPassiveLevel(self->slice.ccu8, out, level8);
    } else {
        XMC_CCU4_SLICE_SetPassiveLevel(self->slice.ccu4, level4);
    }
}

static void timer_channel_configure_capture_pin(pyb_timer_obj_t *self, const pin_obj_t *pin, XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_t edge) {
    const pin_af_obj_t *af = find_timer_af(pin, self, false);
    if (af == NULL) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("Pin(%q) has no capture input for Timer(%d)"), pin->name, self->tim_id);
    }

    XMC_GPIO_SetMode(pin->gpio, pin->pin, XMC_GPIO_MODE_INPUT_TRISTATE);

    if (self->is_ccu8) {
        XMC_CCU8_SLICE_EVENT_CONFIG_t ev = {0};
        ev.mapped_input = af->idx;
        ev.edge = (XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_t)edge;
        ev.level = XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH;
        ev.duration = XMC_CCU8_SLICE_EVENT_FILTER_DISABLED;
        XMC_CCU8_SLICE_ConfigureEvent(self->slice.ccu8, XMC_CCU8_SLICE_EVENT_0, &ev);
        XMC_CCU8_SLICE_Capture0Config(self->slice.ccu8, XMC_CCU8_SLICE_EVENT_0);
    } else {
        XMC_CCU4_SLICE_EVENT_CONFIG_t ev = {0};
        ev.mapped_input = af->idx;
        ev.edge = edge;
        ev.level = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH;
        ev.duration = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED;
        XMC_CCU4_SLICE_ConfigureEvent(self->slice.ccu4, XMC_CCU4_SLICE_EVENT_0, &ev);
        XMC_CCU4_SLICE_Capture0Config(self->slice.ccu4, XMC_CCU4_SLICE_EVENT_0);
    }
}

static void timer_channel_enable_irq(pyb_timer_channel_obj_t *chan, XMC_CCU4_SLICE_IRQ_ID_t event) {
    pyb_timer_obj_t *self = chan->timer;
    IRQn_Type irqn = timer_irqn_tbl[self->tim_id];
    if (self->is_ccu8) {
        XMC_CCU8_SLICE_SetInterruptNode(self->slice.ccu8, (XMC_CCU8_SLICE_IRQ_ID_t)event, (XMC_CCU8_SLICE_SR_ID_t)self->slice_idx);
        XMC_CCU8_SLICE_EnableEvent(self->slice.ccu8, (XMC_CCU8_SLICE_IRQ_ID_t)event);
    } else {
        XMC_CCU4_SLICE_SetInterruptNode(self->slice.ccu4, event, (XMC_CCU4_SLICE_SR_ID_t)self->slice_idx);
        XMC_CCU4_SLICE_EnableEvent(self->slice.ccu4, event);
    }
    // irq.h's IRQ_PRI_TIMX assumes STM32 HAL's NVIC_PRIORITYGROUP_4 constant,
    // which doesn't exist on this port; XMC4500 has 6 priority bits
    // (__NVIC_PRIO_BITS), so just pick a plain mid-range priority directly.
    NVIC_SetPriority(irqn, 32);
    NVIC_EnableIRQ(irqn);
}

static void timer_channel_stop(pyb_timer_channel_obj_t *chan) {
    machine_timer_channel_callback(MP_OBJ_FROM_PTR(chan), mp_const_none);
}

/// \method channel(channel, mode, ...)
///
/// If only a channel number is passed, a previously initialised channel
/// object is returned (or `None` if there's no previous channel).
///
/// Each CCU4 slice only has channel 1. Each CCU8 slice has channel 1 and 2
/// (its two independent compare channels); which physical pin serves which
/// channel is fixed by the board's wiring (see the pin's alternate-function
/// table), not chosen here.
///
///   - `mode` is one of:
///     - `Timer.PWM` - PWM output, active high.
///     - `Timer.PWM_INVERTED` - PWM output, active low.
///     - `Timer.IC` - input capture (channel 1 only).
///   - `callback` - as per TimerChannel.callback().
///   - `pin` - the Pin to drive/capture on. Required for PWM and IC.
///
/// Timer.PWM/PWM_INVERTED keyword arguments:
///   - `pulse_width` - initial pulse width, in timer ticks.
///   - `pulse_width_percent` - initial pulse width, as a percentage (0-100).
///
/// Timer.IC keyword arguments:
///   - `polarity` - one of `Timer.RISING` (default), `Timer.FALLING`, `Timer.BOTH`.
///   Read the captured value with `channel.capture()`; the callback (if any)
///   only signals that a new capture is available, same as on the STM32 port.
static mp_obj_t machine_timer_channel(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,                MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_callback,            MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_pin,                 MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_pulse_width,         MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_pulse_width_percent, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_polarity,            MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = TIMER_RISING} },
    };

    pyb_timer_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_int_t channel_num = mp_obj_get_int(pos_args[1]);
    if (channel_num != 1 && channel_num != 2) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid channel (must be 1 or 2)"));
    }
    if (channel_num == 2 && !self->is_ccu8) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("Timer(%d) only has channel 1"), self->tim_id);
    }

    pyb_timer_channel_obj_t *chan = self->channel;
    pyb_timer_channel_obj_t *prev_chan = NULL;
    while (chan != NULL) {
        if (chan->channel == channel_num) {
            break;
        }
        prev_chan = chan;
        chan = chan->next;
    }

    // If only the channel number is given, return the previously allocated
    // channel (or None if there isn't one).
    if (n_args == 2 && kw_args->used == 0) {
        return chan ? MP_OBJ_FROM_PTR(chan) : mp_const_none;
    }

    // Replace any existing channel on this channel number. Order matters so
    // this appears atomic to the IRQ handler.
    if (chan) {
        timer_channel_stop(chan);
        if (prev_chan) {
            prev_chan->next = chan->next;
        } else {
            self->channel = chan->next;
        }
        chan->next = NULL;
    }

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 2, pos_args + 2, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    chan = m_new_obj(pyb_timer_channel_obj_t);
    memset(chan, 0, sizeof(*chan));
    chan->base.type = &pyb_timer_channel_type;
    chan->timer = self;
    chan->channel = channel_num;
    chan->mode = args[0].u_int;
    chan->callback = mp_const_none;

    mp_obj_t pin_obj = args[2].u_obj;
    const pin_obj_t *pin = NULL;
    if (pin_obj != mp_const_none) {
        if (!mp_obj_is_type(pin_obj, &pin_type)) {
            mp_raise_ValueError(MP_ERROR_TEXT("pin argument needs to be a Pin type"));
        }
        pin = MP_OBJ_TO_PTR(pin_obj);
    }

    switch (chan->mode) {
        case CHANNEL_MODE_PWM_NORMAL:
        case CHANNEL_MODE_PWM_INVERTED: {
            if (pin == NULL) {
                mp_raise_ValueError(MP_ERROR_TEXT("PWM mode needs a pin"));
            }
            timer_channel_configure_pwm_pin(self, chan, pin, chan->mode);

            uint32_t pulse;
            if (args[4].u_obj != mp_const_none) {
                mp_float_t percent = mp_obj_get_float(args[4].u_obj);
                if (percent <= 0) {
                    pulse = 0;
                } else if (percent >= 100) {
                    pulse = self->period;
                } else {
                    pulse = (uint32_t)(percent / (mp_float_t)100 * (mp_float_t)self->period);
                }
            } else {
                pulse = (uint32_t)args[3].u_int;
            }

            if (self->is_ccu8) {
                if (chan->channel == 1) {
                    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(self->slice.ccu8, (uint16_t)pulse);
                } else {
                    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(self->slice.ccu8, (uint16_t)pulse);
                }
                XMC_CCU8_EnableShadowTransfer(self->module.ccu8, (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0 << self->slice_idx));
                XMC_CCU8_SLICE_StartTimer(self->slice.ccu8);
            } else {
                XMC_CCU4_SLICE_SetTimerCompareMatch(self->slice.ccu4, (uint16_t)pulse);
                XMC_CCU4_EnableShadowTransfer(self->module.ccu4, (uint32_t)(XMC_CCU4_SHADOW_TRANSFER_SLICE_0 << self->slice_idx));
                XMC_CCU4_SLICE_StartTimer(self->slice.ccu4);
            }
            break;
        }

        case CHANNEL_MODE_IC: {
            if (pin == NULL) {
                mp_raise_ValueError(MP_ERROR_TEXT("IC mode needs a pin"));
            }
            if (chan->channel != 1) {
                mp_raise_ValueError(MP_ERROR_TEXT("IC mode only supports channel 1"));
            }
            mp_int_t polarity = args[5].u_int;
            XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_t edge;
            switch (polarity) {
                case TIMER_RISING:  edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE; break;
                case TIMER_FALLING: edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE; break;
                case TIMER_BOTH:    edge = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_DUAL_EDGE; break;
                default:
                    mp_raise_ValueError(MP_ERROR_TEXT("invalid polarity"));
            }
            timer_channel_configure_capture_pin(self, pin, edge);

            if (self->is_ccu8) {
                XMC_CCU8_SLICE_CAPTURE_CONFIG_t cfg = {0};
                XMC_CCU8_SLICE_CaptureInit(self->slice.ccu8, &cfg);
                XMC_CCU8_SLICE_StartTimer(self->slice.ccu8);
            } else {
                XMC_CCU4_SLICE_CAPTURE_CONFIG_t cfg = {0};
                XMC_CCU4_SLICE_CaptureInit(self->slice.ccu4, &cfg);
                XMC_CCU4_SLICE_StartTimer(self->slice.ccu4);
            }
            break;
        }

        default:
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("mode %d not implemented"), chan->mode);
    }

    // Link the channel in before returning; the write is atomic so this is
    // safe with respect to the IRQ handler.
    chan->next = self->channel;
    self->channel = chan;

    mp_obj_t callback = args[1].u_obj;
    if (callback != mp_const_none) {
        machine_timer_channel_callback(MP_OBJ_FROM_PTR(chan), callback);
    }

    return MP_OBJ_FROM_PTR(chan);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(machine_timer_channel_obj, 2, machine_timer_channel);

/// \method source_freq()
/// Get the frequency of the clock that feeds this timer's prescaler.
static mp_obj_t machine_timer_source_freq(mp_obj_t self_in) {
    pyb_timer_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(timer_source_freq(self));
}
static MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_source_freq_obj, machine_timer_source_freq);

/// \method freq([value])
/// Get or set the frequency for the timer (changes prescaler and period if set).
static mp_obj_t machine_timer_freq(size_t n_args, const mp_obj_t *args) {
    pyb_timer_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args == 1) {
        // PSC holds the prescaler exponent (XMC_CCU4/8_SLICE_PRESCALER_t) in
        // its low 4 bits; self->period is cached since PR is a read-only
        // shadow-transfer target, not something we can just read back.
        uint32_t psc = self->is_ccu8 ? (self->slice.ccu8->PSC & 0xf) : (self->slice.ccu4->PSC & 0xf);
        uint32_t divide_a = 1UL << psc;
        uint32_t divide_b = self->period + 1;
        uint32_t source_freq = timer_source_freq(self);
        #if MICROPY_PY_BUILTINS_FLOAT
        return mp_obj_new_float((mp_float_t)source_freq / (mp_float_t)divide_a / (mp_float_t)divide_b);
        #else
        return mp_obj_new_int(source_freq / divide_a / divide_b);
        #endif
    } else {
        uint32_t period;
        uint32_t prescaler_exp = compute_prescaler_period_from_freq(self, args[1], &period);
        timer_apply_prescaler_period(self, prescaler_exp, period);
        return mp_const_none;
    }
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_timer_freq_obj, 1, 2, machine_timer_freq);

/// \method period([value])
/// Get or set the period of the timer, in timer ticks.
static mp_obj_t machine_timer_period(size_t n_args, const mp_obj_t *args) {
    pyb_timer_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args == 1) {
        return mp_obj_new_int(self->period);
    } else {
        self->period = mp_obj_get_int(args[1]) & 0xffff;
        if (self->is_ccu8) {
            XMC_CCU8_SLICE_SetTimerPeriodMatch(self->slice.ccu8, (uint16_t)self->period);
            XMC_CCU8_EnableShadowTransfer(self->module.ccu8, (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0 << self->slice_idx));
        } else {
            XMC_CCU4_SLICE_SetTimerPeriodMatch(self->slice.ccu4, (uint16_t)self->period);
            XMC_CCU4_EnableShadowTransfer(self->module.ccu4, (uint32_t)(XMC_CCU4_SHADOW_TRANSFER_SLICE_0 << self->slice_idx));
        }
        return mp_const_none;
    }
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_timer_period_obj, 1, 2, machine_timer_period);

static const mp_rom_map_elem_t pyb_timer_locals_dict_table[] = {
    // instance methods
    {MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&machine_timer_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&machine_timer_deinit_obj)},
    {MP_ROM_QSTR(MP_QSTR_channel), MP_ROM_PTR(&machine_timer_channel_obj)},
    {MP_ROM_QSTR(MP_QSTR_source_freq), MP_ROM_PTR(&machine_timer_source_freq_obj)},
    {MP_ROM_QSTR(MP_QSTR_freq), MP_ROM_PTR(&machine_timer_freq_obj)},
    {MP_ROM_QSTR(MP_QSTR_period), MP_ROM_PTR(&machine_timer_period_obj)},
    // mode constants for channel()
    { MP_ROM_QSTR(MP_QSTR_PWM), MP_ROM_INT(CHANNEL_MODE_PWM_NORMAL) },
    { MP_ROM_QSTR(MP_QSTR_PWM_INVERTED), MP_ROM_INT(CHANNEL_MODE_PWM_INVERTED) },
    { MP_ROM_QSTR(MP_QSTR_IC), MP_ROM_INT(CHANNEL_MODE_IC) },
    // polarity constants for IC mode
    { MP_ROM_QSTR(MP_QSTR_RISING), MP_ROM_INT(TIMER_RISING) },
    { MP_ROM_QSTR(MP_QSTR_FALLING), MP_ROM_INT(TIMER_FALLING) },
    { MP_ROM_QSTR(MP_QSTR_BOTH), MP_ROM_INT(TIMER_BOTH) },
};
static MP_DEFINE_CONST_DICT(pyb_timer_locals_dict, pyb_timer_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_timer_type,
    MP_QSTR_Timer,
    MP_TYPE_FLAG_NONE,
    make_new, pyb_timer_make_new,
    print, pyb_timer_print,
    locals_dict, &pyb_timer_locals_dict
    );

/// \moduleref machine
/// \class TimerChannel - a PWM output or capture input on a Timer's slice.
///
/// TimerChannel objects are created using the Timer.channel() method.
static void pyb_timer_channel_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pyb_timer_channel_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "TimerChannel(timer=%u, channel=%u)", self->timer->tim_id, self->channel);
}

/// \method callback(fun)
/// Set the function to be called when the timer channel triggers (PWM
/// period match, or a new input capture). `fun` is passed 1 argument, the
/// owning Timer object. If `fun` is `None` the callback is disabled.
static mp_obj_t machine_timer_channel_callback(mp_obj_t self_in, mp_obj_t callback) {
    pyb_timer_channel_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pyb_timer_obj_t *timer = self->timer;
    if (callback == mp_const_none) {
        self->callback = mp_const_none;
        if (timer->is_ccu8) {
            XMC_CCU8_SLICE_DisableEvent(timer->slice.ccu8, XMC_CCU8_SLICE_IRQ_ID_EVENT0);
        } else {
            XMC_CCU4_SLICE_DisableEvent(timer->slice.ccu4, XMC_CCU4_SLICE_IRQ_ID_EVENT0);
        }
    } else if (mp_obj_is_callable(callback)) {
        self->callback = callback;
        XMC_CCU4_SLICE_IRQ_ID_t event = self->mode == CHANNEL_MODE_IC ? XMC_CCU4_SLICE_IRQ_ID_EVENT0 : XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH;
        timer_channel_enable_irq(self, event);
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("callback must be None or a callable object"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(machine_timer_channel_callback_obj, machine_timer_channel_callback);

/// \method capture()
/// Read the last captured timer value (IC mode only).
static mp_obj_t machine_timer_channel_capture(mp_obj_t self_in) {
    pyb_timer_channel_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pyb_timer_obj_t *timer = self->timer;
    if (self->mode != CHANNEL_MODE_IC) {
        mp_raise_ValueError(MP_ERROR_TEXT("capture() is only valid in IC mode"));
    }
    uint32_t value = timer->is_ccu8
        ? XMC_CCU8_SLICE_GetCaptureRegisterValue(timer->slice.ccu8, 0)
        : XMC_CCU4_SLICE_GetCaptureRegisterValue(timer->slice.ccu4, 0);
    return mp_obj_new_int(value & 0xffff);
}
static MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_channel_capture_obj, machine_timer_channel_capture);

/// \method pulse_width([value])
/// Get or set the PWM pulse width, in timer ticks (PWM modes only).
static mp_obj_t machine_timer_channel_pulse_width(size_t n_args, const mp_obj_t *args) {
    pyb_timer_channel_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    pyb_timer_obj_t *timer = self->timer;
    if (self->mode != CHANNEL_MODE_PWM_NORMAL && self->mode != CHANNEL_MODE_PWM_INVERTED) {
        mp_raise_ValueError(MP_ERROR_TEXT("pulse_width is only valid in PWM mode"));
    }
    if (n_args == 1) {
        uint16_t value = timer->is_ccu8
            ? XMC_CCU8_SLICE_GetTimerCompareMatch(timer->slice.ccu8, self->channel == 1 ? XMC_CCU8_SLICE_COMPARE_CHANNEL_1 : XMC_CCU8_SLICE_COMPARE_CHANNEL_2)
            : XMC_CCU4_SLICE_GetTimerCompareMatch(timer->slice.ccu4);
        return mp_obj_new_int(value);
    } else {
        uint16_t value = (uint16_t)mp_obj_get_int(args[1]);
        if (timer->is_ccu8) {
            if (self->channel == 1) {
                XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(timer->slice.ccu8, value);
            } else {
                XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(timer->slice.ccu8, value);
            }
            XMC_CCU8_EnableShadowTransfer(timer->module.ccu8, (uint32_t)(XMC_CCU8_SHADOW_TRANSFER_SLICE_0 << timer->slice_idx));
        } else {
            XMC_CCU4_SLICE_SetTimerCompareMatch(timer->slice.ccu4, value);
            XMC_CCU4_EnableShadowTransfer(timer->module.ccu4, (uint32_t)(XMC_CCU4_SHADOW_TRANSFER_SLICE_0 << timer->slice_idx));
        }
        return mp_const_none;
    }
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_timer_channel_pulse_width_obj, 1, 2, machine_timer_channel_pulse_width);

/// \method pulse_width_percent([value])
/// Get or set the PWM duty cycle as a percentage 0-100 (PWM modes only).
static mp_obj_t machine_timer_channel_pulse_width_percent(size_t n_args, const mp_obj_t *args) {
    pyb_timer_channel_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    pyb_timer_obj_t *timer = self->timer;
    if (n_args == 1) {
        mp_obj_t get_args[1] = { args[0] };
        mp_obj_t cmp_obj = machine_timer_channel_pulse_width(1, get_args);
        mp_float_t cmp = mp_obj_get_float(cmp_obj);
        mp_float_t period = (mp_float_t)timer->period;
        return mp_obj_new_float(period > 0 ? (cmp * (mp_float_t)100 / period) : (mp_float_t)0);
    } else {
        mp_float_t percent = mp_obj_get_float(args[1]);
        uint32_t cmp;
        if (percent <= 0) {
            cmp = 0;
        } else if (percent >= 100) {
            cmp = timer->period;
        } else {
            cmp = (uint32_t)(percent / (mp_float_t)100 * (mp_float_t)timer->period);
        }
        mp_obj_t set_args[2] = { args[0], mp_obj_new_int(cmp) };
        machine_timer_channel_pulse_width(2, set_args);
        return mp_const_none;
    }
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_timer_channel_pulse_width_percent_obj, 1, 2, machine_timer_channel_pulse_width_percent);

static const mp_rom_map_elem_t machine_timer_channel_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_callback), MP_ROM_PTR(&machine_timer_channel_callback_obj) },
    { MP_ROM_QSTR(MP_QSTR_capture), MP_ROM_PTR(&machine_timer_channel_capture_obj) },
    { MP_ROM_QSTR(MP_QSTR_pulse_width), MP_ROM_PTR(&machine_timer_channel_pulse_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_pulse_width_percent), MP_ROM_PTR(&machine_timer_channel_pulse_width_percent_obj) },
};
static MP_DEFINE_CONST_DICT(machine_timer_channel_locals_dict, machine_timer_channel_locals_dict_table);

static MP_DEFINE_CONST_OBJ_TYPE(
    pyb_timer_channel_type,
    MP_QSTR_TimerChannel,
    MP_TYPE_FLAG_NONE,
    print, pyb_timer_channel_print,
    locals_dict, &machine_timer_channel_locals_dict
    );

// Called from the CCU4x_y_IRQHandler / CCU8x_y_IRQHandler functions in
// xmc_it.c (one per slice's dedicated SR line). Checks which of the two
// events we ever enable on a slice (period-match for PWM channels,
// capture-event0 for an IC channel) actually fired, clears it, and
// schedules the matching channel's Python callback to run at the next safe
// point, passing it the owning Timer object.
void timer_irq_handler(uint tim_id) {
    if (tim_id >= MICROPY_HW_MAX_TIMER) {
        return;
    }
    pyb_timer_obj_t *tim = MP_STATE_PORT(pyb_timer_obj_all)[tim_id];
    if (tim == NULL) {
        return;
    }

    bool period_fired, capture_fired;
    if (tim->is_ccu8) {
        period_fired = XMC_CCU8_SLICE_GetEvent(tim->slice.ccu8, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
        capture_fired = XMC_CCU8_SLICE_GetEvent(tim->slice.ccu8, XMC_CCU8_SLICE_IRQ_ID_EVENT0);
        if (period_fired) {
            XMC_CCU8_SLICE_ClearEvent(tim->slice.ccu8, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
        }
        if (capture_fired) {
            XMC_CCU8_SLICE_ClearEvent(tim->slice.ccu8, XMC_CCU8_SLICE_IRQ_ID_EVENT0);
        }
    } else {
        period_fired = XMC_CCU4_SLICE_GetEvent(tim->slice.ccu4, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
        capture_fired = XMC_CCU4_SLICE_GetEvent(tim->slice.ccu4, XMC_CCU4_SLICE_IRQ_ID_EVENT0);
        if (period_fired) {
            XMC_CCU4_SLICE_ClearEvent(tim->slice.ccu4, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
        }
        if (capture_fired) {
            XMC_CCU4_SLICE_ClearEvent(tim->slice.ccu4, XMC_CCU4_SLICE_IRQ_ID_EVENT0);
        }
    }

    for (pyb_timer_channel_obj_t *chan = tim->channel; chan != NULL; chan = chan->next) {
        if (chan->callback == mp_const_none) {
            continue;
        }
        bool chan_wants_capture = chan->mode == CHANNEL_MODE_IC;
        if (chan_wants_capture ? capture_fired : period_fired) {
            mp_sched_schedule(chan->callback, MP_OBJ_FROM_PTR(tim));
        }
    }
}

MP_REGISTER_ROOT_POINTER(struct _pyb_timer_obj_t *pyb_timer_obj_all[MICROPY_HW_MAX_TIMER]);
