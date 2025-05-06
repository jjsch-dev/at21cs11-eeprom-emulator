#!/usr/bin/env python3
import time
import argparse
from pyocd.core.helpers import ConnectHelper

# Option‐bytes address (from py32f002ax5.h)
OPTION_BYTES_ADDR = 0x1FFF0E80

# USER‐byte bit‐definitions
BOR_EN_BIT     = 1 << 8
BOR_LEV_SHIFT  = 9
BOR_LEV_MASK   = 0x7 << BOR_LEV_SHIFT
IWDG_SW_BIT    = 1 << 12
NRST_MODE_BIT  = 1 << 14
NBOOT1_BIT     = 1 << 15

BOR_LEVELS = {
    0: "1.8 V↑ / 1.7 V↓",
    1: "2.0 V↑ / 1.9 V↓",
    2: "2.2 V↑ / 2.1 V↓",
    3: "2.4 V↑ / 2.3 V↓",
    4: "2.6 V↑ / 2.5 V↓",
    5: "2.8 V↑ / 2.7 V↓",
    6: "3.0 V↑ / 2.9 V↓",
    7: "3.2 V↑ / 3.1 V↓",
}

def parse_option_bytes(raw):
    """Break raw 0xAABBCCDD into its 8‐bit components and interpret."""
    rdp      = raw & 0xFF
    user     = (raw >> 8) & 0xFF
    nrdp     = (raw >> 16) & 0xFF
    nuser    = (raw >> 24) & 0xFF

    parsed = {}
    parsed['RDP'] = {
        'level': 0 if rdp == 0xAA else 1,
        'value': f"0x{rdp:02X}",
    }
    parsed['BOR_EN']    = bool(user & BOR_EN_BIT)
    lev = (user & BOR_LEV_MASK) >> BOR_LEV_SHIFT
    parsed['BOR_LEVEL'] = BOR_LEVELS.get(lev, f"reserved({lev})")
    parsed['IWDG']      = "SW" if (user & IWDG_SW_BIT) else "HW"
    parsed['NRST_MODE'] = "GPIO" if (user & NRST_MODE_BIT) else "RESET"
    parsed['nBOOT1']    = "System-mem" if (user & NBOOT1_BIT) else "Flash-mem"

    # sanity
    parsed['nRDP (comp)']  = f"0x{nrdp:02X}"
    parsed['nUSER (comp)']= f"0x{nuser:02X}"
    return parsed

def main():
    p = argparse.ArgumentParser(
        prog="read_option_bytes.py",
        description="Dump and decode the PY32F0 option bytes."
    )
    p.add_argument("--config", "-c", default="pyocd_local.yaml",
                   help="pyocd config YAML file")
    p.add_argument("--target", "-t", default="py32f002ax5",
                   help="PyOCD target_override")
    args = p.parse_args()

    with ConnectHelper.session_with_chosen_probe(
            target_override=args.target,
            config_file=args.config,
            connect_mode="under-reset") as sess:

        sess.open()
        tgt = sess.target
        tgt.halt()

        raw = tgt.read32(OPTION_BYTES_ADDR)
        print(f"\nRaw Option-bytes word: 0x{raw:08X}\n")

        fields = parse_option_bytes(raw)
        print("→ Read-Protection (RDP):")
        lvl = fields['RDP']['level']
        print(f"   level {lvl} ({'inactive' if lvl==0 else 'active'}), byte={fields['RDP']['value']}\n")

        print("→ Brown-Out Reset (BOR):")
        print(f"   enabled: {fields['BOR_EN']}")
        print(f"   threshold: {fields['BOR_LEVEL']}\n")

        print("→ Independent WDG:")
        print(f"   mode: {fields['IWDG']} (SW=software, HW=hardware)\n")

        print("→ NRST pin:")
        print(f"   function: {fields['NRST_MODE']}\n")

        print("→ nBOOT1:")
        print(f"   boot area: {fields['nBOOT1']}\n")

        # show complements for sanity
        print("→ Complement bytes:")
        print(f"   nRDP:  {fields['nRDP (comp)']}")
        print(f"   nUSER: {fields['nUSER (comp)']}\n")

        sess.close()

if __name__ == "__main__":
    main()

