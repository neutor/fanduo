# fanduo

Smart dual PWM fan controller based on Arduino Nano.

## Features

- Reading of CPU PWM control signal
- 25 kHz PWM output signals on pins 9 and 10 of Nano board
- Auto calibration of new fans on power on
- EEPROM storage for calibration profiles
- Works without USB connection after install

## Hardware

- Arduino Nano board
- CPU Fan cable with 4 pin female connector
- 2x Fan Cable with 4 pin male connector
- USB from Nano to USB header on motherboard cable

Nano is powered by USB 5V. Connect 12V from CPU Fan cable to 12V of Fan Cables directly, bypassing Nano. Nano overheats working on 12V.


## Installation

Use Arduino IDE to flash Nano board
