# AT21CS11 EEPROM Emulation on PY32F0

This repository contains a SWI (Single-Wire Interface) EEPROM emulator for the AT21CS11 device, running on the PUYA PY32F0 microcontroller (Cortex-M0+ @ 24 MHz) using the LL (Low-Layer) peripheral library.

## Features

- **SWI Protocol Emulation**: Implements host-driven bit timing, reset/discovery sequence, ACK/NACK, and data transfer over a single open-drain line.  
- **EEPROM & Manufacturer ID**: Emulates both the main EEPROM (128 bytes) and manufacturer ID responses per AT21CS11 spec.  
- **Timing Accuracy**: Uses TIM1 as a 1 µs free-running timer; pulse-width measurement and generation for sub-10 µs timings.  
- **Minimal Polling**: Main loop polls GPIO input for edges; timing-critical output (“logic 0” pulses) driven directly in the edge handlers.  
- **Enable Pin Gating**: External `ENABLE` pin (active-low) blocks all emulation until released.  
- **Optional Debug**: Compile-time flags to enable GPIO toggling (DBG_PIN) and UART debug output.

## Hardware Requirements

- **Microcontroller**: PUYA PY32F0 series in SOP8 (24 MHz HSI clock).  
- **Pull-up Resistor**: 1 kΩ pull-up on the SWI line.  
- **Flash Layout**: Last page of flash (e.g. `0x08004C00`) reserved for `eeprom_buffer`.

## Software Requirements

- **Toolchain**: ARM GCC (arm-none-eabi-gcc) with `-Os` optimization.  
- **Makefile**: supports `--section-start` for placing the EEPROM data section.  
- **Linker Flags**: `-Wl,--section-start,.eeprom_data=0x08004C00` to pin the buffer.

## Build & Flash:

- **Clone the repository**:
git clone https://github.com/<your-user>/py32f0-eeprom-emulator.git
cd py32f0-eeprom-emulator

- **Build**:
make all

- **Flash** the resulting swi_eeprom.elf (or .hex/.bin) to your PY32F0 device via your programmer of choice

## Configuration: – 
- **To change EEPROM contents**, edit eeprom_data.h (the buffer is tagged with attribute((section(".eeprom_data"))))
- **Enable-pin defaults to PA1**; modify ENABLE_PIN/ENABLE_PORT in main.c if needed
- **To turn on debug toggling or UART prints**, define ENABLE_DEBUG_PIN and/or ENABLE_UART_DEBUG in main.c
