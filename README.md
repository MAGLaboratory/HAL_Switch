# HAL Switch

This is the software for the HAL Switch, a wall-mounted capacitive-sensing
switching solution developed for MAG Laboratory.

The switch detects a press on the capacive pads mounted on the front and 
activates when a press is detected.

Are we sure that it works?  No.  But it will be fun to test.

## Acronyms, Abbreviations, and Definitions
| Term | Definition |
| ---- | ---------- |
| ACMP | Analog Comparator |
| ARM | Advanced RISC Machines |
| EFM | Energy Friendly Microcontroller |
| LED | Light Emitting Diode |
| MAG | Makers, Artists, and Gadgeteers |
| SM | State Machine |
| UART | Universal Asynchronous Receiver / Transmitter |
| XCVR | Transciever |

## Project Description
This is a capacitive switcher aimed at providing a way for intelligent
switching of indoor lights.  Normal people would just use a light switch,
but normal people also leave lights on.

### Hardware Description
There are user interfaces used with this application.
They are listed in the table below.

| Interface | Description |
| --------- | ----------- |
| Capacitive Touch | Senses the capacitance change on touch pads and performs
an action |
| Relays | Turns the light on. | 
| RGB LEDs | Consumes part of the creator's purchase of 1000 LEDs that have no
use at the moment |
| RS485 | Implements a high-speed half-duplex connection over which modbus
can be used |
| SWDIO | A programming interface for ARM devices | 

### Pinout
| Pin Number | Description | Port | Function |
| ---------- | ----------- | ---- | -------- |
| 1 | led1r | PA0 | GPIO |
| 2 | +3V3 | IOVDD_0 | PWR |
| 3 | touch0 | PC0 | ACMP0_CH0 |
| 4 | touch1 | PC1 | ACMP0_CH1 |
| 5 | X | PB7 | X | 
| 6 | X | PB8 | X |
| 7 | RESET | RESET | RESET |
| 8 | led0b | PB11 | GPIO |
| 9 | +3.3VA | AVDD_2 | PWR | 
| 10 | led1b | PB13 | GPIO | 
| 11 | X | PB14 | X |
| 12 | +3.3VA | AVDD_0 | PWR |
| 13 | RX | PD6 | US1_RX #3 |
| 14 | TX | PD7 | US1_TX #3 |
| 15 | +3V3 | VDD_DREG | PWR |
| 16 | DECOUPLE | DECOUPLE | DECOUPLE |
| 17 | TXEN | PC14 | US1_CS #3 |
| 18 | led0g | PC15 | GPIO |
| 19 | SWCLK | PF0 | SWCLK |
| 20 | SWDIO | PF1 | SWCLK |
| 21 | relay1 | PF2 | TIM2_CC0 #3 |
| 22 | +3V3 | IOVDD_5 | PWR | 
| 23 | relay0 | PE12 | TIM2_CC1 #3 | 
| 24 | led1g | PE13 | GPIO |

## Software Description
This section describes the software that goes onto the microcontroller.


### Coil Drive
The relays are driven through a PWM system which aims to reduce the power
consumption and therefore temperature of the relays.

### Kirisaki Capsense
The Kirisaki Capsense library is a re-write and extension of the EFM32 capsense
library which was not designed to run on a fixed timer loop.

Kirisaki Capsense aims to provide more filtering than the original capacitive
sense library including a robust "maximum cycles" to compensate for capacitance
changes through varied conditions.

Kirisaki Capsense does not handle debouncing; that should be handled by
external application code since debouncing is an extremely common

### Machining Light
The machining area light is controlled through two states.  A rising edge puts
the light on the "on" state which is motion-activated.  A long press puts the 
light on the "off" state which keeps it off until a rising edge is detected.
A rising edge after the inital short press refreshes the light timer in case
of PIR sensor malfunction.

### Outside Light
The outside light is controlled through a similar two state system with the
exception where a shutdown would put the light into the "off" state.
