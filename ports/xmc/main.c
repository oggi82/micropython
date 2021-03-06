/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013-2018 Damien P. George
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
#include <stdio.h>
#include <string.h>

#include "lib/utils/pyexec.h"
#include "py/compile.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "py/repl.h"
#include "py/runtime.h"

#include "lib/mp-readline/readline.h"
#include "py/mphal.h"
#include "py/stackctrl.h"
// #include "lib/oofatfs/ff.h"
// #include "extmod/vfs.h"
//#include "extmod/vfs_fat.h"

#include "VirtualSerial.h"
#include "irq.h"
#include "modmachine.h"
#include "pin.h"
#include "rtc.h"

#if MICROPY_ENABLE_COMPILER
void do_str(const char *src, mp_parse_input_kind_t input_kind) {
  nlr_buf_t nlr;
  if (nlr_push(&nlr) == 0) {
    mp_lexer_t *lex =
        mp_lexer_new_from_str_len(MP_QSTR__lt_stdin_gt_, src, strlen(src), 0);
    qstr source_name = lex->source_name;
    mp_parse_tree_t parse_tree = mp_parse(lex, input_kind);
    mp_obj_t module_fun = mp_compile(&parse_tree, source_name, true);
    mp_call_function_0(module_fun);
    nlr_pop();
  } else {
    // uncaught exception
    mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
  }
}
#endif

static char *stack_top;
#if MICROPY_ENABLE_GC
static char heap[16384];
#endif

int main(int argc, char **argv) {
  int stack_dummy;
  stack_top = (char *)&stack_dummy;
  SysTick_Config(SystemCoreClock / 1000);
  USB_Init();
  machine_init();
soft_reset:
#if MICROPY_ENABLE_GC
  gc_init(heap, heap + sizeof(heap));
#endif

  readline_init0();
  pin_init0();
  mp_init();
#if MICROPY_ENABLE_COMPILER
#if MICROPY_REPL_EVENT_DRIVEN
  pyexec_event_repl_init();
  for (;;) {
    int c = mp_hal_stdin_rx_chr();
    if (pyexec_event_repl_process_char(c)) {
      break;
    }
  }
#else
#if MICROPY_ENABLE_COMPILER
  // Main script is finished, so now go into REPL mode.
  // The REPL mode can change, or it can request a soft reset.
  for (;;) {
    if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
      if (pyexec_raw_repl() != 0) {
        break;
      }
    } else {
      if (pyexec_friendly_repl() != 0) {
        break;
      }
    }
  }
#endif
#endif
// do_str("print('hello world!', list(x+1 for x in range(10)), end='eol\\n')",
// MP_PARSE_SINGLE_INPUT); do_str("for i in range(10):\r\n  print(i)",
// MP_PARSE_FILE_INPUT);
#else
  pyexec_frozen_module("frozentest.py");
#endif
  // soft_reset_exit:
  printf("MPY: soft reboot\n");
  mp_deinit();
  machine_deinit();
  gc_sweep_all();
  goto soft_reset;
  return 0;
}

void gc_collect(void) {
  // WARNING: This gc_collect implementation doesn't try to get root
  // pointers from CPU registers, and thus may function incorrectly.
  void *dummy;
  gc_collect_start();
  gc_collect_root(&dummy, ((mp_uint_t)stack_top - (mp_uint_t)&dummy) /
                              sizeof(mp_uint_t));
  gc_collect_end();
  gc_dump_info();
}

mp_lexer_t *mp_lexer_new_from_file(const char *filename) {
  mp_raise_OSError(MP_ENOENT);
}

mp_import_stat_t mp_import_stat(const char *path) {
  return MP_IMPORT_STAT_NO_EXIST;
}

mp_obj_t mp_builtin_open(size_t n_args, const mp_obj_t *args,
                         mp_map_t *kwargs) {
  return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(mp_builtin_open_obj, 1, mp_builtin_open);

void nlr_jump_fail(void *val) {
  while (1)
    ;
}

void NORETURN __fatal_error(const char *msg) {
  while (1)
    ;
}

#ifndef NDEBUG
void MP_WEAK __assert_func(const char *file, int line, const char *func,
                           const char *expr) {
  printf("Assertion '%s' failed, at file %s:%d\n", expr, file, line);
  __fatal_error("Assertion failed");
}
#endif
