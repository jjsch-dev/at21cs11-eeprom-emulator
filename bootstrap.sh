#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "================================================================="
echo " Deploying Local Environment for Puya PY32F0xx "
echo "================================================================="

# 1. Base directories definitions
BASE_DIR=$(pwd)
TOOLCHAIN_DIR="$BASE_DIR/toolchain"
GCC_DIR="$TOOLCHAIN_DIR/gcc-arm"
PUYA_TEMPLATE_DIR="$TOOLCHAIN_DIR/py32f0"
PYOCD_DIR="$TOOLCHAIN_DIR/pyocd"
SCRIPTS_DIR="$TOOLCHAIN_DIR/scripts"

# ARM GCC 14.2.rel1 Toolchain configuration (Cortex-M0+ compatible)
GCC_VERSION="14.2.rel1"
GCC_FOLDER="arm-gnu-toolchain-$GCC_VERSION-x86_64-arm-none-eabi"
GCC_TARBALL="$GCC_FOLDER.tar.xz"
GCC_URL="https://developer.arm.com/-/media/Files/downloads/gnu/$GCC_VERSION/binrel/$GCC_TARBALL"

echo "➡️ Creating directory structure under /toolchain..."
mkdir -p "$GCC_DIR"
mkdir -p "$PUYA_TEMPLATE_DIR"
mkdir -p "$PYOCD_DIR"
mkdir -p "$SCRIPTS_DIR"

# 2. Download and extract ARM GCC toolchain
echo "➡️ Verifying local ARM GCC compiler..."
if [ ! -d "$GCC_DIR/$GCC_FOLDER" ]; then
    echo "   Downloading ARM GNU Toolchain ($GCC_VERSION)..."
    cd "$GCC_DIR"
    wget --show-progress "$GCC_URL"
    
    echo "   Extracting toolchain..."
    tar -xf "$GCC_TARBALL"
    rm "$GCC_TARBALL"
    cd "$BASE_DIR"
    echo "   [OK] Compiler installed locally."
else
    echo "   [OK] Compiler already exists locally."
fi

# 3. Clone the Puya LL support library (py32f0-template)
echo "➡️ Verifying PY32F0xx LL support library..."
if [ ! -d "$PUYA_TEMPLATE_DIR/py32f0-template" ]; then
    echo "   Cloning IOsetting/py32f0-template..."
    cd "$PUYA_TEMPLATE_DIR"
    git clone https://github.com/IOsetting/py32f0-template.git py32f0-template
    cd "$BASE_DIR"
    echo "   [OK] Puya template repository cloned."
else
    echo "   [OK] Puya template library already exists."
fi

# 4. Set up isolated Python virtual environment for pyOCD
echo "➡️ Configuring virtual environment for flashing utilities..."
if [ ! -d "$PYOCD_DIR/venv" ]; then
    echo "   Creating Python venv at toolchain/pyocd/venv..."
    python3 -m venv "$PYOCD_DIR/venv"
    
    echo "   Installing and upgrading pyOCD..."
    source "$PYOCD_DIR/venv/bin/activate"
    pip install -U pip setuptools
    pip install -U pyocd
    deactivate
    echo "   [OK] pyOCD successfully installed in isolated venv."
else
    echo "   [OK] pyOCD virtual environment already exists."
fi

# 5. Populate local udev rules file for ST-Link permissions
echo "➡️ Generating ST-Link V2 local udev rules file..."
cat << 'EOF' > "$SCRIPTS_DIR/99-stlinkv2.rules"
# STMicroelectronics ST-LINK/V2 permissions for pyOCD access
SUBSYSTEMS=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3748", MODE="0666"
EOF
echo "   [OK] Rule created under toolchain/scripts/99-stlinkv2.rules"

# 6. Automatically generate unified pyocd.yaml config files
echo "➡️ Generating configuration files (pyocd.yaml)..."
cat << EOF > "$BASE_DIR/pyocd.yaml"
# Auto-generated pyocd configuration file for local workspace execution
pack:
  - ./toolchain/py32f0/py32f0-template/Misc/Puya.PY32F0xx_DFP.1.1.7.pack

target_override: py32f002ax5

debug:
  connect_mode: under-reset
EOF

# Copy file into toolchain/pyocd/ to satisfy rules.mk expectations
cp "$BASE_DIR/pyocd.yaml" "$PYOCD_DIR/pyocd.yaml"
echo "   [OK] Local pyocd.yaml deployment complete."

echo "================================================================="
echo " 🎉 Workspace Environment Restored Successfully! "
echo "================================================================="
echo "To verify the compiler, run:"
echo "  ./toolchain/gcc-arm/$GCC_FOLDER/bin/arm-none-eabi-gcc --version"
echo ""
echo "To flash the hardware, use:"
echo "  make flash"
echo "================================================================="
