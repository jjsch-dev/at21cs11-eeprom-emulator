# AT21CS11 EEPROM Emulation on Puya PY32F0xx

This repository contains an **AT21CS11 EEPROM emulator over SWI (Single-Wire Interface)** implemented on the **Puya PY32F0xx** Cortex-M0+ microcontroller using **LL (Low Layer)** drivers.

It uses precise timing and GPIO polling to emulate the behavior of the original IC, responding to commands like manufacturer ID query and memory access. The internal EEPROM buffer persists across resets by placing it in a dedicated flash section that isn't erased during firmware updates.

🔗 [GitHub Repository](https://github.com/jjsch-dev/at21cs11-eeprom-emulator)

The project emulates basic commands like:
- EEPROM read/write
- Manufacturer ID query
- Start/stop condition detection for host communication
- ACK/NACK handling
- Bit-level timing control via polling and manual GPIO toggling

All global state transitions are handled in the main loop with precise µs-level timing to mimic the behavior of the original IC.

---

## 📦 Features

- ✅ Bit-banged SWI protocol implementation
- ✅ Supports command decoding for:
  - `OPCODE_MANUFACTURER_ID`
  - `OPCODE_EEPROM_ACCESS`
- ✅ ACK/NACK handling without external hardware
- ✅ Precise µs-level timing control using TIM1
- ✅ Optional UART logging via `ENABLE_UART_DEBUG`
- ✅ Debug pin support via `ENABLE_DEBUG_PIN`
- ✅ EEPROM data stored in last flash page to survive firmware updates
- ✅ No HAL or RTOS dependencies — minimal and fast

---

## 🧰 Hardware Requirements

| Component       | Description |
|------------------|-------------|
| MCU              | Puya PY32F0 series (tested on SOP8 package) |
| Clock Source     | Internal HSI oscillator set to 24 MHz |
| SWI Pin          | PA10 (open-drain output) |
| Enable Pin       | PA1 (active-low input) |
| Debug GPIO (opt) | PA14 (toggled during activity for signal tracing) |
| UART TX (opt)    | PA2 (for debug logging at 115200 bps) |
| Pull-up resistor | 1 kΩ on the SWI line |

> [!IMPORTANT]
> Last page of flash (e.g. `0x08004C00`) reserved for `eeprom_buffer`.

> ⚠️ Important: The host must follow the expected bit timing. This code does not currently adapt dynamically to varying speeds.

---

## ⚙️ Software Requirements

- ARM GCC toolchain (`arm-none-eabi-gcc`)
- Makefile-based build system
- Programmer compatible with PY32 MCUs (e.g., `st-flash`, `pyOCD`, etc.)
- Linker support for section placement:
  ```bash
  -Wl,--section-start,.eeprom_data=0x08004C00

> [!NOTE]
> Ensure your startup code does NOT zero-initialize the .eeprom_data section.
> This allows you to persist EEPROM values across resets and firmware updates. 

---

## 📁 File Structure
├── README.md
├── Makefile                # Set compiler, flags, memory layout
├── main.c                  # Main logic, SWI protocol, state machine
├── debug.h / debug.c       # Optional UART logging and debug pin support
├── eeprom_data.h           # Contains manuf_id[] and eeprom_buffer[]

---

## ⚙️ Configuration Options
ENABLE_START_CONDITION_DETECT   # Enables timeout-based start condition detection
ENABLE_UART_DEBUG               # Enables UART-based logging via USART1
ENABLE_DEBUG_PIN                # Toggles a debug GPIO pin during operation

To configure pins:
// In main.c
#define SWI_PIN             LL_GPIO_PIN_10
#define SWI_GPIO_Port       GPIOA

#define ENABLE_PIN          LL_GPIO_PIN_1
#define ENABLE_GPIO_Port    GPIOA

#define DBG_PIN             LL_GPIO_PIN_14
#define DBG_GPIO_Port       GPIOA

---

### 📑 Build & Flash Instructions

## 🛠 Build & Flash

1. **Clone the repository**

```bash
git clone https://github.com/jjsch-dev/at21cs11-eeprom-emulator.git
cd at21cs11-eeprom-emulator
```
2. **Build the project**

```bash
make
```
3. **Flash to the PY32F0xx device by SWD**

```bash
make flash
```

---

### 📑 Known Limitations

## ⚠️ Known Limitations

- ❗ Polling-based design – could be improved with DMA or interrupts
- ❗ Only one device address supported
- ❗ EEPROM writes do not include checksum or persistence management
- ❗ Host timing must be predictable and match expected thresholds

---

## 🚀 Future Improvements (Ideas)

- [ ] Add adaptive bit timing calibration
- [ ] Improve EEPROM persistence with CRC or wear leveling
- [ ] Support for security register access
- [ ] Implement proper USART TX FIFO for non-blocking debug prints
- [ ] Add Python script for EEPROM image generation
- [ ] Provide STM32CubeIDE project structure

---

## 📜 License

[MIT License](LICENSE) – © jjsch-dev (2025)

You can freely use and modify this code as long as you include the original license.

## 🧑‍💻 Contributing

For contributions or improvements:
- Fork the project
- Create a new branch
- Submit a pull request

Or open issues or feature requests directly on the repository.

---

## 📬 Contact & Credits

- **Author**: jjsch-dev  
- **Repository**: [GitHub Link](https://github.com/jjsch-dev/at21cs11-eeprom-emulator)

---
