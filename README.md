# PsNee-ESP32

This is an ESP32 implementation of [PsNee](https://github.com/kalymos/PsNee) as used on PS1HDMI.

The PsOne PAL BIOS Patch is not included.

PsNee-ESP32 uses [esp-idf v3.3.1](https://github.com/espressif/esp-idf/tree/release/v3.3)

The file containing PsNee is: [`main/psneeTask.h`](https://github.com/chriz2600/PsNee-ESP32/blob/master/main/psneeTask.h)

The main goal was to use as little resources as possible. Thus the original code was rewritten using interrupts and a freertos task.

The minimum freertos tick rate is 250 Hz (1000 Hz recommended), because it uses `vTaskDelay` to achieve the 250 bits/s injection rate.

Demo-Project pinout is:

- `SQCK:      22`
- `GATE_WFCK: 32`
- `DATA:      25`
- `SUBQ:      21`
