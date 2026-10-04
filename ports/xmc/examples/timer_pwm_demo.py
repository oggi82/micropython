# machine.Timer PWM smoke test for the XMC4500 Relax Lite Kit.
#
# Uses the two onboard LEDs so no extra wiring is needed:
#   - LED2 (P1.0, Timer(3) = CCU40 slice 3) blinks slowly at 50% duty,
#     to prove the PWM output + period actually toggle the pin.
#   - LED1 (P1.1, Timer(2) = CCU40 slice 2) runs fast (not visibly
#     flickering) while its duty cycle is ramped 0 -> 100 -> 0 %, to prove
#     pulse_width_percent() actually changes brightness.
#
# Expected result: LED2 blinks about twice a second; LED1 breathes smoothly
# from off to full brightness and back, repeatedly.

import machine
import time

blink = machine.Timer(3)
blink.init(freq=2)
blink_ch = blink.channel(1, machine.Timer.PWM, pin=machine.Pin.board.P1_0, pulse_width_percent=50)

breathe = machine.Timer(2)
breathe.init(freq=1000)
breathe_ch = breathe.channel(1, machine.Timer.PWM, pin=machine.Pin.board.P1_1, pulse_width_percent=0)

print("blink_ch:", blink_ch)
print("breathe_ch:", breathe_ch)

direction = 1
percent = 0
while True:
    breathe_ch.pulse_width_percent(percent)
    percent += direction * 2
    if percent >= 100:
        percent = 100
        direction = -1
    elif percent <= 0:
        percent = 0
        direction = 1
    time.sleep_ms(20)
