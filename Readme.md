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

## 🖼️ Hardware Setup

The following image shows the physical implementation of the AT21CS11 EEPROM emulator on the PY32F0xx board:

![Hardware Setup](images/hardware_setup.png)

> 💡 Key components visible:
> - **PY32F002AL15S6T MCU**: Core processing unit
> - **SWD Pins**: SWCLK (SWC) and SWDIO (SWO) for debugging and flashing
> - **EEPROM Programming Header**: Used for programming the device
> - **Pull-up Resistors**: Ensures proper logic levels on critical pins

This setup demonstrates how the emulator is integrated into a real-world application.

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

## 🖼️ Hardware Schematic

The following schematic illustrates the hardware connections for the AT21CS11 EEPROM emulator on the PY32F0xx microcontroller. This includes key components like the SWI interface, enable pin, debug pins, and pull-up resistors.

![AT21CS11 EEPROM Emulator Schematic](images/schematic.png)

> **Note**: The final version may adjust some details (e.g., power-on RC delay), but this provides a clear overview of the current design.

---

## 🧰 Hardware Requirements

| Component         | Description |
|------------------|-------------|
| MCU              | Puya PY32F002 series (tested on SOP8 package) |
| Clock Source     | Internal HSI oscillator set to 24 MHz |
| SWI Pin          | PA10 (open-drain output) |
| Enable Pin       | PA1 (active-low input) |
| Debug GPIO (opt) | PA3 (toggled during bit transitions for tracing) |
| UART TX (opt)    | PA2 (for debug logging at 115200 bps) |
| SWCLK / SWC      | PA14 (SWD clock for flashing and debugging) |
| SWDIO / SWD      | PA13 (SWD data I/O for flashing and debugging) |
| Pull-up resistor | 1 kΩ on the SWI line |

> [!IMPORTANT]
> Last page of flash (e.g. `0x08004C00`) reserved for `eeprom_buffer`.

> ⚠️ Important: The host must follow the expected bit timing. This code does not currently adapt dynamically to varying speeds.

---

## 🛠 Software Requirements

This project assumes all development tools are placed under a local `toolchain/` directory inside your repo root:


This helps ensure consistent builds across different systems and avoids conflicts with system-wide installations.

---

### 1. ARM GCC Toolchain (arm-none-eabi-gcc)

Used to compile embedded C code for Cortex-M0+ architecture.

#### 🔽 Installation Steps:

1. **Download** the latest version of ARM GCC toolchain (`arm-none-eabi-gcc`):
   - [Arm GNU Toolchain Downloads](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

2. **Verify** the download using SHA256 checksum:
   ```bash
   sha256sum -c arm-gnu-toolchain-*.tar.xz.sha256asc
   ```
3. **Extract** into the toolchain/gcc-arm/ folder
   ```bash
   mkdir -p toolchain/gcc-arm/
   tar -xvf arm-gnu-toolchain-*.tar.xz -C toolchain/gcc-arm/
   ```	
4. **Add** to PATH temporarily (for building):
   ```bash
   export PATH="$(pwd)/toolchain/gcc-arm/arm-gnu-toolchain-<version>-x86_64-arm-none-eabi/bin:$PATH"
   ```
> [!NOTE]
> Replace <version> with the actual extracted folder name.

5. **Verify** installation:
   ```bash
   arm-none-eabi-gcc --version
   ```
---

### 2. py32f0-template (PY32F0xx LL support library)

This project uses the [IOsetting/py32f0-template](https://github.com/IOsetting/py32f0-template) library to simplify register-level access to the PY32F0xx peripherals.

#### 🧩 Installation Steps

1. **Clone** or extract the library inside **toolchain/py32f0/**

   Option A: Clone via Git
   ```bash
   git clone https://github.com/IOsetting/py32f0-template.git toolchain/py32f0
   ```

   Option B: Download [ZIP](https://codeload.github.com/IOsetting/py32f0-template/zip/refs/heads/main) and extract
   ```bash
   unzip py32f0-template-main.zip -d toolchain/py32f0
   ```
   
2. **Include** path in Makefile
   ```bash
   LIB_ROOT := toolchain/py32f0/py32f0-template-main/
   ```
> [!NOTE]
> This project includes a tested version of the **py32f0-template** for reference only. It’s recommended to use the latest version from GitHub for bug fixes and improvements. 
   	
---

### 3. pyOCD (for flashing and debugging via SWD)

We use [pyOCD](https://pyocd.io/) to program the PY32F0xx microcontroller over SWD using an ST-Link or CMSIS-DAP programmer.

### 🧰 Install pyOCD Locally:

1. **Create** a virtual environment (recommended):
   ```bash
   python3 -m venv toolchain/pyocd/venv
   source toolchain/pyocd/venv/bin/activate
   ```
   
2. **Install** pyOCD in the virtual environment :
   ```bash
   echo 'source $(pwd)/toolchain/pyocd/venv/bin/activate' > toolchain/pyocd/activate.sh
   chmod +x toolchain/pyocd/activate.sh
   ```
---

### 4. Device Support for PY32F0xx
**pyOCD** does not natively support Puya PY32 MCUs, so we must provide a device pack file .

### 📦 Add PY32F0xx Support

1. **Download** the device family pack:

- From: [Keil Device Family Pack for PY32F0xx](https://www.keil.arm.com/packs/py32f0xx_dfp-puya/versions/)
- File used in this project: **Puya.PY32F0xx_DFP.1.1.0.pack**

2. **Place** it in the `toolchain/pyocd/` folder:
   ```bash
   cp Puya.PY32F0xx_DFP.1.1.0.pack toolchain/pyocd/
   ```
   
3. **Create** a pyocd.yaml configuration file in your project root:
   ```ymal
   pack:
     - ./toolchain/pyocd/Puya.PY32F0xx_DFP.1.1.0.pack 
   ```

4. **Use** the config when flashing :
   ```bash
   # Load firmware
   pyocd load ./swi_eeprom.hex -t py32f002ax5 --config ./pyocd.yaml

   # Erase chip
   pyocd erase --chip -t py32f002ax5 --config ./pyocd.yaml
   ```

> [!WARNING]
> Make sure the target name (py32f002ax5) matches your specific MCU variant.

---

### 5. Optional: Programmer Hardware

To flash and debug the device, you'll need an **SWD-compatible programmer**. This project has been tested using:

✅ **ST-Link V2 clone** (widely available and low-cost)  
Other supported options:
- CMSIS-DAP compatible debugger
- Black Magic Probe (alternative)

Ensure the following connections are made between the programmer and the PY32 MCU:

| Programmer Pin | Target MCU (PY32F0xx) |
|----------------|----------------------|
| SWCLK          | PA14                 |
| SWDIO          | PA13                 |
| GND            | GND                  |
| 3.3V (optional) | 3.3V               |

> [!NOTE]
> Some clones may require firmware updates to work reliably with non-STM32 devices like the Puya PY32 series.

---

## 📁 File Structure
    .
    ├── build                   	# Compiled files (ELF, HEX, BIN files)
    ├── docs                    	# Documentation folder 
    │ └── Microchip-AT21CS11.pdf 	# Official AT21CS11 Datasheet from Microchip
    ├── toolchain               	# Local tools installation (optional / project-specific)
    │ ├── gcc-arm                   # ARM GCC toolchain (arm-none-eabi-gcc)
    │ ├── py32f0                    # PY32F0xx LL support library (py32f0-template)
    │ └── pyocd                     # pyOCD + config files for flashing via SWD
    ├── at21cs11_emulator.c     	# Core logic for SWI EEPROM emulation
    ├── debug.h / debug.c       	# Optional UART logging and debug pin support
    ├── eeprom_data.h           	# Declaration of EEPROM buffer and Manufacturer ID
    ├── Makefile / rules.mk     	# Build configuration and linker flags
    └── README.md               	# Project overview and instructions

---

## ⚙️ Configuration Options

| Macro                         | Description |
|-------------------------------|-------------|
| ENABLE_START_CONDITION_DETECT | Enables timeout-based start condition detection
| ENABLE_UART_DEBUG             | Enables UART-based logging via USART1
| ENABLE_DEBUG_PIN              | Toggles a debug GPIO pin during operation
| SWI_PIN                       | Set the BUS pin, by default LL_GPIO_PIN_10
| ENABLE_PIN                    | Set the enable pin, by default LL_GPIO_PIN_1
| DBG_PIN                       | Set the debug toggle pin, by default LL_GPIO_PIN_14

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

## 📝 SWI Protocol Overview

The **AT21CS01/11 EEPROM device** operates as a slave device using a **single-wire digital serial interface (SWI)** to communicate with a host controller. The protocol is designed for simplicity and efficiency, utilizing an 8-bit data structure where power is provided via the SI/O pin.

During communication, the state of the SI/O pin during specific time intervals determines the interpretation of data. Each bit frame transmits one data bit, and after eight bits (one byte), the receiving device must respond with either an **Acknowledge (ACK)** or **No Acknowledge (NACK)** response bit during a ninth bit window.

---

### 📊 Types of Data Transmitted Over the SI/O Line

The following types of data are transmitted over the single-wire interface:

1. **Reset and Discovery Response**
   - Used by the master to reset the device and perform a general bus call to determine if any devices are present on the bus.
   - **Waveform**: ![Figure 4-1: Reset and Discovery Response Waveform](images/figure_4-1.png)

2. **Logic ‘0’ or Acknowledge (ACK)**
   - Indicates a logic '0' or an ACK response.
   - **Waveform**: ![Figure 4-2: Logic ‘0’ Input Condition Waveform](images/figure_4-2.png)

3. **Logic ‘1’ or No Acknowledge (NACK)**
   - Indicates a logic '1' or a NACK response.
   - **Waveform**: ![Figure 4-3: Logic ‘1’ Input Condition Waveform](images/figure_4-3.png)

4. **Start Condition**
   - Marks the beginning of a command sequence.
   - **Waveform**: ![Figure 4-4: Start Condition Waveform](images/figure_4-4.png)

5. **Stop Condition**
   - Marks the end of a command sequence.
   - **Waveform**: ![Figure 4-5: Stop Condition Waveform](images/figure_4-5.png)

---

## 🧠 Microcontroller Overview: PUYA PY32F002AL15S6TU

The **PUYA PY32F002AL15S6TU** is a low-cost, 32-bit ARM® Cortex®-M0+ microcontroller in an SOP-8 package. It provides all the necessary features to emulate the AT21CS11 EEPROM over a single-wire interface using bit-banged GPIO and precise timing control.

![PY32F0xx Functional Modules](images/py32f002_block_diagram.png)

This diagram illustrates how the various peripherals are interconnected.

### 🔍 Key Features

| Feature               | Specification |
|-----------------------|---------------|
| **Core**              | ARM Cortex-M0+ @ up to 24 MHz |
| **Flash Memory**      | 20 KB (enough to store firmware + emulated EEPROM) |
| **RAM**               | 3 KB SRAM |
| **GPIO**              | Configurable open-drain pins with internal pull-ups |
| **Timers**            | Two 16-bit timers for sub-microsecond edge detection and pulse generation |
| **Communication**     | USART, SPI, I²C interfaces |
| **ADC & Comparators** | 12-bit ADC, 2 comparators (not used in this project) |
| **Voltage Range**     | 1.7 V – 5.5 V |
| **Temperature Range** | -40 °C to +85 °C |
| **Low-Power Modes**  | Sleep and Stop modes supported |

> 📦 [Official Datasheet](docs/PY32F002A_datasheet_Rev.0.2_EN.pdf)

---

### ✅ Why This MCU Is Ideal for EEPROM Emulation

- **Accurate Bit-Bang Timing**:  
  The dual 16-bit timers and deterministic Cortex-M0+ instruction timing allow sub-microsecond delays required by the AT21CS11 protocol.

- **Open-Drain GPIO Support**:  
  Dynamic switching between input (high impedance) and output modes with internal pull-ups matches the single-wire, open-drain requirements of SWI communication.

- **Sufficient Flash & RAM**:  
  With 20 KB Flash and 3 KB SRAM, the chip comfortably supports:
  - Bit-banged SWI logic
  - Timer-based edge detection
  - Optional debug logging
  - Persistent memory emulation in flash

- **Robust Communication Peripherals**:  
  Built-in USART, SPI, and I²C are useful for debugging or future expansions (e.g., capturing external EEPROM signals).

- **Cost-Effective & Compact**:  
  At under $0.10 in volume and housed in a small SOP-8 package, it’s ideal for prototyping or high-volume deployments.

By leveraging its precise timers, flexible GPIO, and ample memory, the PY32F002AL15S6TU becomes a highly capable platform for emulating an AT21CS11 EEPROM entirely in software.

---
## 📚 Datasheet Reference

For full details on the **AT21CS11 EEPROM** and its **Single-Wire Interface (SWI)**, please refer to the official Microchip datasheet:

📄 [AT21CS11 Datasheet](docs/Microchip-AT21CS11.pdf)

This document includes:
- Full electrical specifications
- Timing diagrams for all communication states (reset, discovery, ACK/NACK, start/stop)
- Memory map and command set
- Power-up and security register behavior

> 💡 Tip: The timing thresholds used in this project (`THRESHOLD_RESET`, `THRESHOLD_BIT`, etc.) are derived from the timing diagrams in the datasheet. You may need to adjust them slightly depending on your host device's timing behavior.

---

### 📑 Known Limitations

## ⚠️ Known Limitations

This implementation has several limitations that are important to understand before using or extending the project:

- ❗ **Polling-based design** – The current implementation uses GPIO polling for edge detection instead of interrupts or DMA.  
  This can be improved by using a faster CPU like the [PY32L020](docs/PY32L020_Datasheet_V1.0.pdf) (48 MHz) for better timing resolution and lower CPU usage.

- ❗ **Single device address support** – Only one device address is currently supported (`AT21_DEVICE_ADDR`).  
  Extending this to support multiple addresses would allow coexistence of multiple devices on the same SWI bus.

- ❗ **EEPROM write command not implemented** – While read operations work, the EEPROM write command is not yet supported.  
  This limits full bidirectional communication with the host until implemented.

- ❗ **Timing sensitivity** – Host communication must follow expected bit timing precisely.  
  Any deviation may cause synchronization issues due to the polling-based nature of the code.

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
