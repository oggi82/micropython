#include "py/obj.h"
#include "machine_pin.h"
#include "xmc_scu.h"
#include "xmc_vadc.h"
// Returns the pin mode. This value returned by this macro should be one of:
// GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_OUTPUT_OD,
// GPIO_MODE_AF_PP, GPIO_MODE_AF_OD, or GPIO_MODE_ANALOG.

uint32_t pin_get_mode(const pin_obj_t *pin) {
    XMC_GPIO_PORT_t *gpio = pin->gpio;
    // check for pdis
    uint32_t mode = 0;
    if (XMC_GPIO_CHECK_OUTPUT_PORT(gpio))
    {
        mode = gpio->IOCR[pin->pin>>2] >> ((uint32_t)PORT_IOCR_PC_Size * ((uint32_t)pin->pin & 0x3));
    }else if(XMC_GPIO_CHECK_ANALOG_PORT(gpio))
    {
        mode = XMC_ANALOG_MODE;
    }
    //uint32_t mode = (gpio->MODER >> (pin->pin * 2)) & 3;
    //if (mode != GPIO_MODE_ANALOG) {
    //     if (gpio->OTYPER & pin->pin_mask) {
    //         mode |= 1 << 4;
    //     }
    // }
    // return mode;
    return mode;
}

// Returns the pin pullup/pulldown. The value returned by this macro should
// be one of GPIO_NOPULL, GPIO_PULLUP, or GPIO_PULLDOWN.
uint32_t pin_get_pull(const pin_obj_t *pin) {
    XMC_GPIO_PORT_t *gpio = pin->gpio;
    return gpio->IOCR[pin->pin>>2] >> ((uint32_t)PORT_IOCR_PC_Size * ((uint32_t)pin->pin & 0x3));
}

// Returns the af (alternate function) index currently set for a pin.
uint32_t pin_get_af(const pin_obj_t *pin) {
    XMC_GPIO_PORT_t *gpio = pin->gpio;
    return gpio->IOCR[pin->pin>>2] & 0x3; // add check for state
    //return (pin->gpio->AFR[pin->pin >> 3] >> ((pin->pin & 7) * 4)) & 0xf;
}

