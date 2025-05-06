#!/usr/bin/env python3
"""
puya_write_option_byte.py

This script programs the user option bytes on a Puya PY32F002Ax5 device.
The option bytes are stored as a 32-bit word; the lower 16 bits hold the user
configuration while the upper 16 bits are automatically computed by the hardware
(as typically the complement of the lower half).

The lower 16 bits are organized as follows:
  - Bits [7:0]   : RDP (Read Protection). Allowed values:
       • 0xAA: Level 0 (inactive)
       • 0x55 (typically): Level 1 (active)
  - Bit  [8]     : BOR_EN (Brown-Out Reset enable; 0 = disabled, 1 = enabled)
  - Bits [11:9]  : BOR_LEV (Brown-Out Reset level). Voltage thresholds:
       • 000: 1.8V rising, 1.7V falling
       • 001: 2.0V rising, 1.9V falling
       • 010: 2.2V rising, 2.1V falling
       • 011: 2.4V rising, 2.3V falling
       • 100: 2.6V rising, 2.5V falling
       • 101: 2.8V rising, 2.7V falling
       • 110: 3.0V rising, 2.9V falling
       • 111: 3.2V rising, 3.1V falling
  - Bit  [12]    : IWDG_SW (Watchdog mode; 0 = hardware, 1 = software)
  - Bit  [13]    : Reserved (should be 1 per factory default)
  - Bit  [14]    : NRST_MODE (NRST pin mode; 0 = GPIO mode (NRST disabled), 1 = reset mode)
  - Bit  [15]    : nBOOT1 (boot configuration; typically 1)

For instance, the factory default lower‐half value 0xBEAA is computed as:
  nBOOT1=1, NRST_MODE=0, Reserved=1, IWDG_SW=1, BOR_LEV=111, BOR_EN=0, RDP=0xAA,
yielding: (1<<15)|(0<<14)|(1<<13)|(1<<12)|(7<<9)|(0<<8)|0xAA = 0xBEAA.
After reset, the hardware computes the upper 16 bits (the complement, e.g. 0x0155),
so the live option bytes read become 0x0155BEAA.

Programming sequence:
  1) Unlock FLASH_CR by writing:
         KEY1 = 0x45670123
         KEY2 = 0xCDEF89AB
     to FLASH_KEYR.
  2) If option bytes are locked, unlock them by writing:
         OPTKEY1 = 0x08192A3B
         OPTKEY2 = 0x4C5D6E7F
     to FLASH_OPTKEYR.
  3) Wait until any flash operation completes (check BSY in FLASH_SR).
  4) Write the lower 16-bit option value to the option byte register.
  5) Set the OPTSTRT bit in FLASH_CR.
  6) Trigger programming by writing a dummy 32-bit value to the main flash trigger address.
  7) Wait for completion and then reset the MCU.

WARNING:
- Incorrect programming of option bytes may disable debugging or brick the device.
- Always have a recovery method (e.g., BOOT0 jumper/bootloader mode) available.
- Verify all register addresses and bit definitions against your reference manual.

Register definitions (from py32f002ax5.h):
  FLASH_BASE          = 0x40022000
  FLASH_KEYR          = FLASH_BASE + 0x08
  FLASH_OPTKEYR       = FLASH_BASE + 0x0C
  FLASH_SR            = FLASH_BASE + 0x10   (BSY flag at bit 16)
  FLASH_CR            = FLASH_BASE + 0x14   (OPTSTRT at bit 17, OPTLOCK at bit 30)
  FLASH_OPTR_ADDR     = FLASH_BASE + 0x20
  OB_BASE             = 0x1FFF0E80
  MAIN_FLASH_TRIGGER  = FLASH_BASE + 0x80

Key values:
  FLASH_KEY1          = 0x45670123
  FLASH_KEY2          = 0xCDEF89AB
  FLASH_OPTKEY1       = 0x08192A3B
  FLASH_OPTKEY2       = 0x4C5D6E7F

"""

import time
import argparse
from pyocd.core.helpers import ConnectHelper
from pyocd.core.exceptions import ProbeError, TransferTimeoutError

# ----------------------------------------------------------------------------
# Register base and offsets (from py32f002ax5.h)
# ----------------------------------------------------------------------------
FLASH_BASE       = 0x40022000
FLASH_KEYR       = FLASH_BASE + 0x08      # FLASH Key register
FLASH_OPTKEYR    = FLASH_BASE + 0x0C      # FLASH Option Key register
FLASH_SR         = FLASH_BASE + 0x10      # FLASH Status register (BSY flag at bit 16)
FLASH_CR         = FLASH_BASE + 0x14      # FLASH Control register (OPTSTRT at bit 17, OPTLOCK at bit 30)
FLASH_OPTR_ADDR  = FLASH_BASE + 0x20      # FLASH Option register (live copy)

MAIN_FLASH_TRIGGER_ADDR = FLASH_BASE + 0x80  # 0x40022080

# ----------------------------------------------------------------------------
# Control bits (from header definitions)
# ----------------------------------------------------------------------------
FLASH_CR_OPTSTRT = (1 << 17)  # 0x00020000
FLASH_CR_OPTLOCK = (1 << 30)  # 0x40000000
FLASH_CR_LOCK    = (1 << 31)  # 0x80000000
FLASH_SR_BSY     = (1 << 16)  # 0x00010000

# ----------------------------------------------------------------------------
# Unlock keys (from header file)
# ----------------------------------------------------------------------------
FLASH_KEY1     = 0x45670123
FLASH_KEY2     = 0xCDEF89AB
FLASH_OPTKEY1  = 0x08192A3B
FLASH_OPTKEY2  = 0x4C5D6E7F

# ----------------------------------------------------------------------------
# Option byte field positions (lower 16 bits):
#  Bits [7:0]   : RDP
#  Bit  [8]     : BOR_EN
#  Bits [11:9]  : BOR_LEV
#  Bit  [12]    : IWDG_SW
#  Bit  [13]    : Reserved
#  Bit  [14]    : NRST_MODE
#  Bit  [15]    : nBOOT1
def build_option_value(rdp, bor_en, bor_lev, iwdg_sw, nrst_mode, nboot1, reserved):
    return ((nboot1 & 0x1) << 15) | ((nrst_mode & 0x1) << 14) | ((reserved & 0x1) << 13) | \
           ((iwdg_sw & 0x1) << 12) | ((bor_lev & 0x7) << 9) | ((bor_en & 0x1) << 8) | (rdp & 0xFF)

# Default factory values (from manufacturer defaults)
DEFAULT_RDP      = 0xAA  # 0xAA = Level 0 (inactive); 0x55 = Level 1 (active)
DEFAULT_BOR_EN   = 0     # BOR disabled by default
DEFAULT_BOR_LEV  = 7     # 111: BOR rising = 3.2V, falling = 3.1V
DEFAULT_IWDG_SW  = 1     # Software watchdog enabled
DEFAULT_NRST_MODE= 0     # 0 = GPIO mode (NRST disabled)
DEFAULT_NBOOT1   = 1     # Typically 1
DEFAULT_RESERVED = 1     # Reserved bit default is 1

def wait_for_not_busy(target):
    """Wait until the BSY flag in FLASH_SR is cleared."""
    while target.read32(FLASH_SR) & FLASH_SR_BSY:
        time.sleep(0.01)

def puya_write_option_byte(option_value):
    """Program the lower 16-bit option byte value into the device."""
    with ConnectHelper.session_with_chosen_probe(target_override="py32f002ax5", connect_mode="under-reset") as session:
        target = session.target
        target.halt()

        current_live = target.read32(FLASH_OPTR_ADDR)
        stored_ob  = target.read32(0x1FFF0E80)
        print("Live FLASH_OPTR before programming: 0x{:08X}".format(current_live))
        print("Stored option bytes before programming (raw): 0x{:08X}".format(stored_ob))

        # Step 1: Unlock FLASH_CR
        print("Unlocking FLASH_CR...")
        target.write32(FLASH_KEYR, FLASH_KEY1)
        target.write32(FLASH_KEYR, FLASH_KEY2)

        # Step 2: Unlock Option Bytes if locked
        cr_val = target.read32(FLASH_CR)
        if cr_val & FLASH_CR_OPTLOCK:
            print("Option bytes are locked; performing OPTKEY unlock sequence...")
            try:
                target.write32(FLASH_OPTKEYR, FLASH_OPTKEY1)
                target.write32(FLASH_OPTKEYR, FLASH_OPTKEY2)
            except Exception as e:
                print("Error during OPTKEY unlock sequence:", e)
                return
            cr_val = target.read32(FLASH_CR)
            if cr_val & FLASH_CR_OPTLOCK:
                print("Failed to unlock option bytes. They may be permanently protected via SWD.")
                return
            else:
                print("Option bytes unlocked successfully.")
        else:
            print("Option bytes already unlocked.")

        wait_for_not_busy(target)

        # Step 3: Write the lower 16-bit option value.
        print("Writing option bytes value: 0x{0:04X}".format(option_value))
        try:
            target.write32(FLASH_OPTR_ADDR, option_value)
        except Exception as e:
            print("Error writing option bytes value:", e)
            return

        # Step 4: Set the OPTSTRT bit in FLASH_CR
        cr_val = target.read32(FLASH_CR)
        cr_val |= FLASH_CR_OPTSTRT
        target.write32(FLASH_CR, cr_val)

        # Step 5: Trigger programming
        print("Triggering option byte programming...")
        try:
            target.write32(MAIN_FLASH_TRIGGER_ADDR, 0xFFFFFFFF)
        except TransferTimeoutError as te:
            print("Warning: Triggering programming timed out:", te)
        except Exception as e:
            print("Error triggering programming:", e)
            return

        wait_for_not_busy(target)
        print("Option bytes programming sequence complete.")

        # Step 6: Reset the MCU
        print("Resetting device to reload option bytes...")
        target.reset()
        time.sleep(1)

        session.close()
        with ConnectHelper.session_with_chosen_probe(target_override="py32f002ax5", connect_mode="under-reset") as new_session:
            new_target = new_session.target
            new_target.halt()
            new_live   = new_target.read32(FLASH_OPTR_ADDR)
            new_stored = new_target.read32(0x1FFF0E80)
            print("Live FLASH_OPTR after programming: 0x{:08X}".format(new_live))
            print("Stored option bytes after programming (raw): 0x{:08X}".format(new_stored))
            new_session.close()

        print("Done.")

def main():
    parser = argparse.ArgumentParser(description="Program option bytes on a Puya PY32F002Ax5 device.")
    parser.add_argument("--rdp", type=lambda s: int(s, 0), default=DEFAULT_RDP,
                        help="8-bit RDP value: 0xAA = Level 0 (inactive), 0x55 = Level 1 (active). (default: 0x{:X})".format(DEFAULT_RDP))
    parser.add_argument("--bor_en", type=int, choices=[0, 1], default=DEFAULT_BOR_EN,
                        help="BOR enable (0 = disabled, 1 = enabled). (default: {0})".format(DEFAULT_BOR_EN))
    parser.add_argument("--bor_lev", type=int, choices=range(0, 8), default=DEFAULT_BOR_LEV,
                        help=("3-bit BOR level. Voltage thresholds:\n"
                              "  000: 1.8V/1.7V, 001: 2.0V/1.9V, 010: 2.2V/2.1V, 011: 2.4V/2.3V,\n"
                              "  100: 2.6V/2.5V, 101: 2.8V/2.7V, 110: 3.0V/2.9V, 111: 3.2V/3.1V. (default: {0})"
                              .format(DEFAULT_BOR_LEV)))
    parser.add_argument("--iwdg_sw", type=int, choices=[0, 1], default=DEFAULT_IWDG_SW,
                        help="IWDG_SW (0 = hardware, 1 = software). (default: {0})".format(DEFAULT_IWDG_SW))
    parser.add_argument("--nrst_mode", type=int, choices=[0, 1], default=DEFAULT_NRST_MODE,
                        help="NRST_MODE (0 = GPIO mode, 1 = reset mode). (default: {0})".format(DEFAULT_NRST_MODE))
    parser.add_argument("--nboot1", type=int, choices=[0, 1], default=DEFAULT_NBOOT1,
                        help="nBOOT1 (0 or 1). (default: {0})".format(DEFAULT_NBOOT1))
    parser.add_argument("--reserved", type=int, choices=[0, 1], default=DEFAULT_RESERVED,
                        help="Reserved bit (default: {0})".format(DEFAULT_RESERVED))
    args = parser.parse_args()

    option_value = build_option_value(args.rdp, args.bor_en, args.bor_lev,
                                      args.iwdg_sw, args.nrst_mode, args.nboot1, args.reserved)
    print("Programming lower 16-bit option value: 0x{0:04X}".format(option_value))
    try:
        puya_write_option_byte(option_value)
    except ProbeError as pe:
        print("Probe error during session initialization:", pe)
    except Exception as e:
        print("An unexpected error occurred:", e)

if __name__ == "__main__":
    main()

