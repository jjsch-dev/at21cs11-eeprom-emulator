/**
 * @file eeprom_data.h
 * @brief Emulated EEPROM memory layout for AT21CS11 device over SWI
 *
 * Contains:
 * - Manufacturer ID (fixed 24-bit response)
 * - Emulated EEPROM buffer with default content
 *
 * The EEPROM buffer should be placed in a special section (via linker script or attribute)
 * so it is not overwritten during resets and remains persistent.
 */
 
#ifndef EEPROM_DATA_H
#define EEPROM_DATA_H

#include <stdint.h>

/**
 * @brief Manufacturer ID response array (3 bytes = 24 bits).
 *
 * When the host sends OPCODE_MANUFACTURER_ID, this array returns the manufacturer,
 * density, and revision information as described in the AT21CS11 datasheet.
 *
 * Format:
 * - Byte 0: Manufacturer ID (I²C assigned value for Microchip)
 * - Byte 1: Density encoding
 * - Byte 2: Revision or part number encoding
 */
uint8_t manuf_id[3] = {0x00, 0xD3, 0x80};	// AT21CS11.

/**
 * @brief Emulated EEPROM memory buffer (128 bytes).
 *
 * This buffer simulates the internal EEPROM of the AT21CS11 device.
 * It is marked with:
 * - `section(".eeprom_data")`: Ensures it can be in the last page (20Kbytes) of the PY32.
 * - `used`: Prevents removal by optimizer when not referenced directly
 * - `aligned(4)`: Helps with memory access efficiency and potential DMA use
 *
 * Initialized with test pattern:
 * - First few bytes contain fixed ASCII string "0010942-294241633735835", followed by FFs.
 */
__attribute__((section(".eeprom_data"), used, aligned(4)))
uint8_t eeprom_buffer[128] = {
	0x02, 0x20, 0x00, 0x1D, 0xCE, 0xCF, 0x03, 0x66,
	0x30, 0x30, 0x31, 0x30, 0x39, 0x34, 0x32, 0x2D,
	0x32, 0x39, 0x34, 0x32, 0x34, 0x34, 0x31, 0x36,
	0x33, 0x33, 0x37, 0x33, 0x35, 0x38, 0x33, 0x35,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

#endif // EEPROM_DATA_H

