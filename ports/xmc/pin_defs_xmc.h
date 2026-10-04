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
#include "xmc_common.h"
//#include "xmc4_gpio_map.h"
// This file contains pin definitions that are specific to the xmc port.
// This file should only ever be #included by pin.h and not directly.

#define XMC_ANALOG_MODE 0x1fUL << PORT0_IOCR0_PC0_Pos

enum {
  PORT_0, // more to add
  PORT_1,
  PORT_2,
  PORT_3,
  PORT_4,
  PORT_5,
  PORT_6,
  PORT_14,
  PORT_15,
};

// Must have matching entries in SUPPORTED_FN in boards/make-pins.py
enum {
  AF_FN_TIM,   // CCU4 slice, unit = flat slice id 0..15 (module*4 + slice)
  AF_FN_TIM8,  // CCU8 slice, unit = flat slice id 16..23 (16 + module*4 + slice)
//   AF_FN_I2C,
//   AF_FN_USART,
//   AF_FN_UART = AF_FN_USART,
//   AF_FN_SPI,
//   AF_FN_I2S,
//   AF_FN_SDMMC,
//   AF_FN_CAN,
};

// CCU4 (AF_FN_TIM): idx is the GPIO alternate-function number (XMC_GPIO_MODE_OUTPUT_ALTn)
// for TIM_OUT, or the raw CC4yINS input-select code (from xmc4_ccu4_map.h) for TIM_IN.
enum {
  AF_PIN_TYPE_TIM_OUT = 0,
  AF_PIN_TYPE_TIM_IN,
};

// CCU8 (AF_FN_TIM8): idx is still the GPIO ALTn for outputs / CC8yINS select
// code for TIM8_IN. Each CCU8 slice has 4 named output pins OUT0..OUT3: OUT0
// and OUT1 both carry compare channel 1's status (as a passive-level choice,
// not two different signals), OUT2 and OUT3 both carry channel 2's. A given
// board pin is wired to exactly one of these four, so the type itself must
// carry which one (not just "it's an output") so timer.c knows which
// compare channel (TIM8_OUT0/1 -> channel 1, TIM8_OUT2/3 -> channel 2) and
// which XMC_CCU8_SLICE_OUTPUT_t to pass to XMC_CCU8_SLICE_SetPassiveLevel().
enum {
  AF_PIN_TYPE_TIM8_OUT0 = 0,
  AF_PIN_TYPE_TIM8_OUT1,
  AF_PIN_TYPE_TIM8_OUT2,
  AF_PIN_TYPE_TIM8_OUT3,
  AF_PIN_TYPE_TIM8_IN,
};
#define AF_TIM8_OUT_CHANNEL(af_type) (((af_type) < AF_PIN_TYPE_TIM8_OUT2) ? 1 : 2)
#define AF_TIM8_OUT_N(af_type)       (af_type)

// Flat timer-id scheme: each machine.Timer id addresses exactly one CCU4/CCU8
// slice (its own independent counter/period), not a 4-channel STM32-style
// timer. Timer(0..15) = CCU40.0..CCU43.3, Timer(16..23) = CCU80.0..CCU81.3.
#define TIMER_ID_CCU40(slice) (0  + (slice))
#define TIMER_ID_CCU41(slice) (4  + (slice))
#define TIMER_ID_CCU42(slice) (8  + (slice))
#define TIMER_ID_CCU43(slice) (12 + (slice))
#define TIMER_ID_CCU80(slice) (16 + (slice))
#define TIMER_ID_CCU81(slice) (20 + (slice))
#define TIMER_ID_IS_CCU8(id)  ((id) >= 16)

// enum {
//   AF_PIN_TYPE_TIM_CH1 = 0,
//   AF_PIN_TYPE_TIM_CH2,
//   AF_PIN_TYPE_TIM_CH3,
//   AF_PIN_TYPE_TIM_CH4,
//   AF_PIN_TYPE_TIM_CH1N,
//   AF_PIN_TYPE_TIM_CH2N,
//   AF_PIN_TYPE_TIM_CH3N,
//   AF_PIN_TYPE_TIM_CH1_ETR,
//   AF_PIN_TYPE_TIM_ETR,
//   AF_PIN_TYPE_TIM_BKIN,

//   AF_PIN_TYPE_I2C_SDA = 0,
//   AF_PIN_TYPE_I2C_SCL,

//   AF_PIN_TYPE_USART_TX = 0,
//   AF_PIN_TYPE_USART_RX,
//   AF_PIN_TYPE_USART_CTS,
//   AF_PIN_TYPE_USART_RTS,
//   AF_PIN_TYPE_USART_CK,
//   AF_PIN_TYPE_UART_TX  = AF_PIN_TYPE_USART_TX,
//   AF_PIN_TYPE_UART_RX  = AF_PIN_TYPE_USART_RX,
//   AF_PIN_TYPE_UART_CTS = AF_PIN_TYPE_USART_CTS,
//   AF_PIN_TYPE_UART_RTS = AF_PIN_TYPE_USART_RTS,

//   AF_PIN_TYPE_SPI_MOSI = 0,
//   AF_PIN_TYPE_SPI_MISO,
//   AF_PIN_TYPE_SPI_SCK,
//   AF_PIN_TYPE_SPI_NSS,

//   AF_PIN_TYPE_I2S_CK = 0,
//   AF_PIN_TYPE_I2S_MCK,
//   AF_PIN_TYPE_I2S_SD,
//   AF_PIN_TYPE_I2S_WS,
//   AF_PIN_TYPE_I2S_EXTSD,

//   AF_PIN_TYPE_SDMMC_CK = 0,
//   AF_PIN_TYPE_SDMMC_CMD,
//   AF_PIN_TYPE_SDMMC_D0,
//   AF_PIN_TYPE_SDMMC_D1,
//   AF_PIN_TYPE_SDMMC_D2,
//   AF_PIN_TYPE_SDMMC_D3,

//   AF_PIN_TYPE_CAN_TX = 0,
//   AF_PIN_TYPE_CAN_RX,
// };

// // The HAL uses a slightly different naming than we chose, so we provide
// // some #defines to massage things. Also I2S and SPI share the same
// // peripheral.

// #define GPIO_AF5_I2S2   GPIO_AF5_SPI2
// #define GPIO_AF5_I2S3   GPIO_AF5_I2S3ext
// #define GPIO_AF6_I2S2   GPIO_AF6_I2S2ext
// #define GPIO_AF6_I2S3   GPIO_AF6_SPI3
// #define GPIO_AF7_I2S2   GPIO_AF7_SPI2
// #define GPIO_AF7_I2S3   GPIO_AF7_I2S3ext

// #define I2S2  SPI2
// #define I2S3  SPI3

// #if defined(STM32H7)
// // Make H7 FDCAN more like CAN
// #define CAN1 FDCAN1
// #define CAN2 FDCAN2
// #define GPIO_AF9_CAN1 GPIO_AF9_FDCAN1
// #define GPIO_AF9_CAN2 GPIO_AF9_FDCAN2
// #endif

// enum {
//   PIN_ADC1  = (1 << 0),
//   PIN_ADC2  = (1 << 1),
//   PIN_ADC3  = (1 << 2),
// };

typedef XMC_GPIO_PORT_t pin_gpio_t;
