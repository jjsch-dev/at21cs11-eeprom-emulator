##### Project #####

PROJECT			?= swi_eeprom

# The path for generated files
BUILD_DIR		= Build

# MCU types: 
#   PY32F002Ax5 
#   PY32F002Bx5
#   PY32F003x4, PY32F003x6, PY32F003x8,
#   PY32F030x6, PY32F030x8, 
#   PY32F072xB
MCU_TYPE		= PY32F002Ax5

##### Options #####

# Use LL library instead of HAL, y:yes, n:no
USE_LL_LIB ?= y
# Enable printf float %f support, y:yes, n:no
ENABLE_PRINTF_FLOAT	?= n
# Build with FreeRTOS, y:yes, n:no
USE_FREERTOS	?= n
# Build with CMSIS DSP functions, y:yes, n:no
USE_DSP			?= n
# Build with Waveshare e-paper lib, y:yes, n:no
USE_EPAPER		?= n
# Programmer, jlink or pyocd
FLASH_PROGRM	?= pyocd

##### Toolchains #######

ARM_TOOLCHAIN	?= toolchain/gcc-arm/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin

# path to JLinkExe
JLINKEXE		?= /opt/SEGGER/JLink/JLinkExe

# path to PyOCD
PYOCD_EXE		?= toolchain/pyocd/myenv/bin/pyocd

##### Paths ############

# C and CPP source folders
#CDIRS		:= Examples/PY32F0xx/HAL/GPIO/LED_Toggle 
# Single C and CPP source files
CFILES		:= at21cs11_emulator.c \
			   debug.c \
			   py32f0xx_it.c

# ASM source folders
ADIRS		:= User
# Single ASM source files
AFILES		:= 

# Assuming you're inside 'source/swi_debug'
#LIB_ROOT := /home/jjsch/3d_printer/prusa_original/rmx35/xlcd/eeprom/tools/py32f0-template-main/
LIB_ROOT := ../../tools/py32f0-template-main/

# Include paths
INCLUDES	:= $(LIB_ROOT)Libraries/CMSIS/Core/Include \
			   $(LIB_ROOT)Libraries/CMSIS/Device/PY32F0xx/Include \
			   $(CDIRS) \
			   ./Config
				 
##### Library Paths ############

# Library flags
LIB_FLAGS		= $(MCU_TYPE)
# JLink device (Uppercases)
JLINK_DEVICE	?= $(shell echo $(MCU_TYPE) | tr '[:lower:]' '[:upper:]')
# PyOCD device (Lowercases)
PYOCD_DEVICE	?= $(shell echo $(MCU_TYPE) | tr '[:upper:]' '[:lower:]')
# Link descript file: 
LDSCRIPT		= $(LIB_ROOT)Libraries/LDScripts/$(PYOCD_DEVICE).ld


ifneq (,$(findstring PY32F002B,$(MCU_TYPE)))

# PY32F002B >>>
CFILES		+= $(LIB_ROOT)Libraries/CMSIS/Device/PY32F0xx/Source/system_py32f002b.c

ifeq ($(USE_LL_LIB),y)
CDIRS		+= $(LIB_ROOT)Libraries/PY32F002B_LL_Driver/Src \
			   $(LIB_ROOT)Libraries/PY32F002B_LL_BSP/Src
INCLUDES	+= $(LIB_ROOT)Libraries/PY32F002B_LL_Driver/Inc \
			   $(LIB_ROOT)Libraries/PY32F002B_LL_BSP/Inc
LIB_FLAGS   += USE_FULL_LL_DRIVER
else
CDIRS		+= $(LIB_ROOT)Libraries/PY32F002B_HAL_Driver/Src \
			   $(LIB_ROOT)Libraries/PY32F002B_HAL_BSP/Src
INCLUDES	+= $(LIB_ROOT)Libraries/PY32F002B_HAL_Driver/Inc \
		       $(LIB_ROOT)Libraries/PY32F002B_HAL_BSP/Inc
endif
# Startup file
AFILES	:= $(LIB_ROOT)Libraries/CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f002b.s
# PY32F002B <<<

else ifneq (,$(findstring PY32F07,$(MCU_TYPE)))

#  PY32F07x >>>
CFILES		+= $(LIB_ROOT)Libraries/CMSIS/Device/PY32F0xx/Source/system_py32f07x.c

CDIRS		+= $(LIB_ROOT)Libraries/PY32F07x_HAL_Driver/Src \
			   $(LIB_ROOT)Libraries/PY32F07x_HAL_BSP/Src
INCLUDES	+= $(LIB_ROOT)Libraries/PY32F07x_HAL_Driver/Inc \
			   $(LIB_ROOT)Libraries/PY32F07x_HAL_BSP/Inc
LIB_FLAGS   += USE_HAL_DRIVER
# Startup file
AFILES	:= $(LIB_ROOT)Libraries/CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f072.s
#  PY32F07 <<<

else

# PY32F002A,003,030 >>>
CFILES		+= $(LIB_ROOT)Libraries/CMSIS/Device/PY32F0xx/Source/system_py32f0xx.c

ifeq ($(USE_LL_LIB),y)
CDIRS		+= $(LIB_ROOT)Libraries/PY32F0xx_LL_Driver/Src \
			   $(LIB_ROOT)Libraries/PY32F0xx_LL_BSP/Src
INCLUDES	+= $(LIB_ROOT)Libraries/PY32F0xx_LL_Driver/Inc \
			   $(LIB_ROOT)Libraries/PY32F0xx_LL_BSP/Inc
LIB_FLAGS   += USE_FULL_LL_DRIVER
else
CDIRS		+= $(LIB_ROOT)Libraries/PY32F0xx_HAL_Driver/Src \
			   $(LIB_ROOT)Libraries/PY32F0xx_HAL_BSP/Src
INCLUDES	+= $(LIB_ROOT)Libraries/PY32F0xx_HAL_Driver/Inc \
			   $(LIB_ROOT)Libraries/PY32F0xx_HAL_BSP/Inc
endif
# Startup file
ifneq (,$(findstring PY32F002A,$(LIB_FLAGS)))
AFILES	:= $(LIB_ROOT)Libraries/CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f002a.s
endif
ifneq (,$(findstring PY32F003,$(LIB_FLAGS)))
AFILES	:= $(LIB_ROOT)Libraries/CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f003.s
endif
ifneq (,$(findstring PY32F030,$(LIB_FLAGS)))
AFILES	:= $(LIB_ROOT)Libraries/CMSIS/Device/PY32F0xx/Source/gcc/startup_py32f030.s
endif
# PY32F002A,003,030 <<<

endif

######## Additional Libs ########

ifeq ($(USE_FREERTOS),y)
CDIRS		+= $(LIB_ROOT)Libraries/FreeRTOS \
			   $(LIB_ROOT)Libraries/FreeRTOS/portable/GCC/ARM_CM0

CFILES		+= $(LIB_ROOT)Libraries/FreeRTOS/portable/MemMang/heap_4.c

INCLUDES	+= $(LIB_ROOT)Libraries/FreeRTOS/include \
			   $(LIB_ROOT)Libraries/FreeRTOS/portable/GCC/ARM_CM0
endif

ifeq ($(USE_DSP),y)
CFILES 		+= $(LIB_ROOT)Libraries/CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/BayesFunctions/BayesFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/CommonTables/CommonTables.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/ControllerFunctions/ControllerFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/DistanceFunctions/DistanceFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/FastMathFunctions/FastMathFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/FilteringFunctions/FilteringFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/StatisticsFunctions/StatisticsFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/SupportFunctions/SupportFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/SVMFunctions/SVMFunctions.c \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/Source/TransformFunctions/TransformFunctions.c
INCLUDES	+= $(LIB_ROOT)Libraries/CMSIS/DSP/Include \
			   $(LIB_ROOT)Libraries/CMSIS/DSP/PrivateInclude
endif

ifeq ($(USE_EPAPER),y)
CDIRS		+= $(LIB_ROOT)Libraries/EPaper/Lib \
			   $(LIB_ROOT)Libraries/EPaper/Examples \
			   $(LIB_ROOT)Libraries/EPaper/Fonts \
			   $(LIB_ROOT)Libraries/EPaper/GUI

INCLUDES	+= $(LIB_ROOT)Libraries/EPaper/Lib \
			   $(LIB_ROOT)Libraries/EPaper/Examples \
			   $(LIB_ROOT)Libraries/EPaper/Fonts \
			   $(LIB_ROOT)Libraries/EPaper/GUI
endif

include ./rules.mk

