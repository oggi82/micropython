# machine.Timer input-capture test for the XMC4500 Relax Lite Kit.
#
# Wiring needed: a single jumper wire from P3.0 to P2.1 (both on header X1/X2,
# see boards/RELAX_LITE_KIT/pins.csv). Nothing else required.
#
#   - P3.0 (Timer(8) = CCU42 slice 0) outputs a steady 2 kHz PWM square wave.
#   - P2.1 (Timer(0) = CCU40 slice 0) captures that signal's rising edges.
#
# The capture timer free-runs at 1 MHz (1 tick = 1 us) and never reaches its
# period (it's left at the 16-bit max), so consecutive capture() values are
# just a 1 MHz running clock sampled on each rising edge: the difference
# between two consecutive captures is the PWM period in microseconds, which
# should print as approximately 500 (1 / 2 kHz = 500 us), repeatedly.
#
# If the jumper isn't connected, the callback will simply never fire and
# nothing will print -- that's also a useful sanity check.

import machine
import time

pwm_tim = machine.Timer(8)
pwm_tim.init(freq=2000)
pwm_tim.channel(1, machine.Timer.PWM, pin=machine.Pin.board.P3_0, pulse_width_percent=50)

cap_tim = machine.Timer(0)
cap_tim.init(freq=1_000_000)  # aim for 1 tick ~= 1 us (actual rate depends on fCCU / the
                              # nearest power-of-2 prescaler, see source_freq() below)
cap_ch = cap_tim.channel(1, machine.Timer.IC, pin=machine.Pin.board.P2_1, polarity=machine.Timer.RISING)

print("source_freq:", cap_tim.source_freq(), "Hz")

last = None


def on_capture(timer):
    global last
    value = cap_ch.capture()
    if last is not None:
        delta = (value - last) & 0xffff
        print("period:", delta, "ticks")
    last = value


cap_ch.callback(on_capture)

while True:
    time.sleep_ms(1000)
