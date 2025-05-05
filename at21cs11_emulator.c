/**
 * @file    at21cs11_emulator.c
 * @brief   AT21CS11 EEPROM Emulation over SWI (Single-Wire Interface)
 * @details This module emulates the behavior of the AT21CS11 EEPROM device using a single-wire interface.
 *          It supports command decoding, memory read/write operations, manufacturer ID response,
 *          timing-based bit detection, ACK/NACK handling, and reset/start condition detection.
 *
 * @note    The implementation uses LL (Low Layer) drivers for GPIO, Timer, and EXTI peripherals
 *          to ensure minimal overhead and maximum control over hardware registers.
 *
 * @note    Timing thresholds are defined to distinguish between logic '0', logic '1',
 *          reset pulses, and discovery pulses. Transmission and reception are bit-banged
 *          via the SWI pin by toggling it under software control with precise delays.
 *
 * @note    Supported commands:
 *          - OPCODE_EEPROM_ACCESS: Read/Write EEPROM memory contents
 *          - OPCODE_MANUFACTURER_ID: Read-only access to Manufacturer ID
 *          - Other commands are acknowledged as not implemented or NACKed accordingly.
 *
 * @author  [jjsch-dev / sudotek]
 * @date    2025-04-06
 */

#include "py32f0xx.h"                   // Device header that includes LL definitions
#include "py32f0xx_ll_tim.h"
#include "py32f0xx_ll_rcc.h"
#include "py32f0xx_ll_bus.h"
#include "py32f0xx_ll_system.h"
#include "py32f0xx_ll_exti.h"
#include "py32f0xx_ll_cortex.h"
#include "py32f0xx_ll_utils.h"
#include "py32f0xx_ll_pwr.h"
#include "py32f0xx_ll_dma.h"
#include "py32f0xx_ll_gpio.h"
#include "py32f0xx_ll_usart.h"
#include "eeprom_data.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "debug.h"

#define ENABLE_START_CONDITION_DETECT   /**< Enables timeout detection for start condition */

//  SWI Pin and Timing Definitions
#define SWI_PIN                 LL_GPIO_PIN_10      /**< SWI pin on GPIOA */
#define SWI_GPIO_Port           GPIOA               /**< Port for SWI pin */
#define CHIP_ENABLE_PIN         LL_GPIO_PIN_1       /**< Enable pin on GPIOA */
#define CHIP_ENABLE_GPIO_Port   GPIOA               /**< Port for chip enable pin */

// Device Address Decoding Macros
/** Format of an 8-bit device address byte:
 * ┌────┬────┬────┬────┬────┬────┬────┬────┐
 * │Bit7│Bit6│Bit5│Bit4│Bit3│Bit2│Bit1│Bit0│
 * ├────┼────┼────┼────┼────┼────┼────┼────┤
 * │    │  OPCODE      │ A2 │ A1 │ A0 │R/W │
 * └────┴────┴────┴────┴────┴────┴────┴────┘
 * Bits 7..4: opcode
 * Bits 3..1: device address (3 bits)
 * Bit  0   : read/write bit
 */
#define AT21_OPCODE_MASK            0xF0U   /**< Opcode bits [7..4] */
#define AT21_OPCODE_SHIFT           4       /**< Shift to align opcode in byte */
#define AT21_ADDR_MASK              0x0EU   /**< Device address bits [3..1] */
#define AT21_ADDR_SHIFT             1       /**< Shift to align address in byte */
#define AT21_RW_MASK                0x01U   /**< Read/Write bit [0] */

///@defgroup AT21_Commands AT21 Command Opcodes
///@{
#define OPCODE_EEPROM_ACCESS        0x0AU   /**< Read/Write the contents of the main memory array. */
#define OPCODE_SEC_REG_ACCESS       0x0BU   /**< Read/Write the contents of the Security register. */
#define OPCODE_LOCK_SEC_REG         0x02U   /**< Permanently lock the contents of the Security register. */
#define OPCODE_ROM_ZONE_REG_ACCESS  0x07U   /**< Inhibit further modification to a zone of the EEPROM array. */
#define OPCODE_FREEZE_ROM           0x01U   /**< Permanently lock the current state of the ROM Zone registers. */
#define OPCODE_MANUFACTURER_ID 	    0x0CU   /**< Query manufacturer and density of device. */
#define OPCODE_STANDARD_SPEED       0x0DU   /**< Switch to Standard Speed mode operation (AT21CS01 only command, the AT21CS11 will NACK this command). */
#define OPCODE_HIGH_SPEED           0x0EU   /**< Switch to High-Speed mode operation (AT21CS01 power-on default; AT21CS11 will ACK this command). */
///@}

#define AT21_DEVICE_ADDR            0x00U   /**< Define your device address (A2..A0) */

/* Timing thresholds in microseconds (adjust as needed) */
#define THRESHOLD_RESET             100U    /**< If a low pulse >100µs => reset trigger */
#define DISCOVERY_HIGH_MIN          10U     /**< Discovery high pulse min */
#define THRESHOLD_BIT               5U      /**< Low pulse <5µs => logic '1'; otherwise '0' */
#define MIN_LOW_PULSE               5U      /**< For driving a '0' in transmit */
#define TIMEOUT_START_US            220U    /**< Timeout if no edge in 200uS (start/stop condition) */


/**
 * @brief State machine for SWI EEPROM emulation
 */
typedef enum {
    STATE_RESET = 0,        /**< Waiting for discovery high pulse after reset low */
    STATE_RECEIVE_CMD,      /**< Receiving the device address/command byte */
    STATE_RECEIVE_ADDR,     /**< Receiving memory address byte (for write or read address pointer) */
    STATE_RECEIVE_DATA,     /**< Receiving data bytes from host (write operation) */
    STATE_TRANSMIT,         /**< Transmitting data back to host */
    STATE_IDLE              /**< Idle state */
} SWI_State_t;

// Global Variables
volatile SWI_State_t swi_state = STATE_IDLE;             /**< Current state of the SWI state machine */
volatile uint32_t last_edge_time = 0;                    /**< Timestamp of the last edge (Timer1, 1µs resolution) */
volatile uint32_t pulse_duration = 0;                    /**< Duration (µs) of the most recent pulse */
volatile uint8_t bit_count = 0;                          /**< Number of bits received so far */
volatile uint8_t current_byte = 0;                       /**< Accumulated byte from the host */
volatile bool send_logic_0 = false;                      /**< Flag indicating we should drive logic 0 at next falling edge */
volatile bool wait_ack = false;                          /**< Waiting for host ACK on transmit */
volatile uint8_t* response_buffer_ptr = NULL;            /**< Pointer to buffer for transmission */
volatile uint8_t response_buffer_size = 0;               /**< Size of the working response buffer */
volatile uint8_t response_index = 0;                     /**< Current byte index in the response buffer */
volatile uint8_t response_bit_index = 0;                 /**< Current bit index (MSB first) in the current response byte */


/* Function Prototypes */
static void system_clock_config(void);
void mx_gpio_init(void);
static void mx_tim1_init(void);
static void mx_exti_init(void);
static inline uint32_t get_us(void);
static void delay_us(uint32_t us);
static void swi_decode_low(void);
static void next_bit(void);

/**
 * @brief Get current time in microseconds using TIM1.
 * @return Current timer count (in microseconds)
 */
static inline uint32_t get_us(void)
{
    return LL_TIM_GetCounter(TIM1);
}

/**
 * @brief Delay for a given number of microseconds using busy-wait.
 * @param us Delay in microseconds
 */
static void delay_us(uint32_t us)
{
    uint32_t start = get_us();
    while ((get_us() - start) < us);
}

/**
 * @brief Configure system clock to 24 MHz using HSI.
 */
static void system_clock_config(void)
{
    LL_RCC_HSI_Enable();
    LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
    while (LL_RCC_HSI_IsReady() != 1);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    LL_SetSystemCoreClock(24000000);
    LL_Init1msTick(24000000);
}

/**
 * @brief Initialize Timer 1 to provide 1 µs resolution and timeout capability.
 */
static void mx_tim1_init(void)
{
    /* Enable clock for TIM1 */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

    /* Set TIM1 prescaler for 1 µs tick */
    LL_TIM_SetPrescaler(TIM1, (SystemCoreClock / 1000000) - 1); // 1 µs tick
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetAutoReload(TIM1, 0xFFFF);
    LL_TIM_DisableARRPreload(TIM1);

#ifdef ENABLE_START_CONDITION_DETECT
    /* Configure Channel 2 for Timeout (Output Compare Timing mode) */
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_FROZEN);
    LL_TIM_OC_SetCompareCH2(TIM1, TIMEOUT_START_US);
    LL_TIM_EnableIT_CC2(TIM1);
#endif

    /* Enable TIM1 counter */
    LL_TIM_EnableCounter(TIM1);

#ifdef ENABLE_START_CONDITION_DETECT
    /* Configure NVIC for TIM1 CC interrupts */
    NVIC_SetPriority(TIM1_CC_IRQn, 1);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
#endif
}

/**
 * @brief Initialize GPIO pins for SWI and Enable input.
 */
void mx_gpio_init(void)
{
    /* Enable clock for GPIOA */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef gpio_init = {0};

    // PA10 for SWI (open-drain output)
    gpio_init.Pin = SWI_PIN;
    gpio_init.Mode = LL_GPIO_MODE_OUTPUT; 
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SWI_GPIO_Port, &gpio_init);
    LL_GPIO_SetOutputPin(SWI_GPIO_Port, SWI_PIN);

    // PA1 as input for enable (low/disable)
    gpio_init.Pin = CHIP_ENABLE_PIN;
    gpio_init.Mode = LL_GPIO_MODE_INPUT;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH; 
    gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(CHIP_ENABLE_GPIO_Port, &gpio_init);
    LL_GPIO_SetOutputPin(CHIP_ENABLE_GPIO_Port, CHIP_ENABLE_PIN);
}

/**
 * @brief Configure EXTI interrupt for the ENABLE pin (rising edge).
 */
static void mx_exti_init(void)
{
    LL_EXTI_InitTypeDef exti_init_struct;
    exti_init_struct.Line = LL_EXTI_LINE_1;
    exti_init_struct.LineCommand = ENABLE;
    exti_init_struct.Mode = LL_EXTI_MODE_IT;
    exti_init_struct.Trigger = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&exti_init_struct);

    NVIC_SetPriority(EXTI0_1_IRQn, 0);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/**
 * @brief EXTI0_1 Interrupt handler for enable pin rising edge.
 *
 * When enable goes low, the IRQ is activated, then we set the SWI pin to high impedance to
 * release the (one wire port) and wait in a loop for the enable signal.
 */
void EXTI0_1_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag(LL_EXTI_LINE_1)) {
        swi_state = STATE_IDLE;
        bit_count = 0;
        current_byte = 0;
        send_logic_0 = false;
        response_index = 0;
        response_bit_index = 0;	
        LL_GPIO_SetOutputPin(SWI_GPIO_Port, SWI_PIN);

        // Wait in the IRQ for enable signal.
        while (LL_GPIO_IsInputPinSet(CHIP_ENABLE_GPIO_Port, CHIP_ENABLE_PIN)); 

        LL_EXTI_ClearFlag(LL_EXTI_LINE_1);
    }
}

#ifdef ENABLE_START_CONDITION_DETECT
/**
 * @brief TIM1 CC interrupt handler for detecting idle periods exceeding one bit time.
 * 
 * This interrupt triggers if the line stays high for more than BIT_TIME_US,
 * indicating a potential start condition after a reset.
 *
 * When triggered:
 * - If in transmit or receive state, assume host may be finishing a write and starting a read
 * - Transition to STATE_RECEIVE_CMD to prepare for next command
 * - Reset relevant counters and flags
 */
void TIM1_CC_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC2(TIM1)) {
        LL_TIM_ClearFlag_CC2(TIM1);
        // If we were receiving data or transmitting, consider this a start condition
        if ((swi_state == STATE_TRANSMIT) ||
            (swi_state == STATE_RECEIVE_ADDR) ||
            (swi_state == STATE_RECEIVE_DATA)) {
            swi_state = STATE_RECEIVE_CMD;
            bit_count = 0;
            current_byte = 0;
        }
        // Restart timer to measure from now
        LL_TIM_SetCounter(TIM1, 0); 
    }
}
#endif

/**
 * @brief Prepare the next bit to be transmitted.
 *
 * Advances to the next byte/bit in the transmission buffer.
 * Sets send_logic_0 if the next bit is '0' to indicate the response.
 * Handles ACK wait state and ensures buffer rollover occurs at end-of-buffer.
 *
 * @note The strategy of sending logic_0 on the falling edge 
 *       (to shorten the response time) has the disadvantage that 
 *       the bit count advances before receiving the host's ACK/NACK.
 *       This must be carefully managed to avoid synchronization issues.
 */
static void next_bit(void)
{
    // If the end of the EEPROM/Manufacurer ID is reached, then the Address Pointer will “roll over” back to the
    // beginning address of that region.
    if (response_index >= response_buffer_size) {
        response_index = 0;
    }

    if (response_bit_index < 8) {
        uint8_t bit = (response_buffer_ptr[response_index] >> (7 - response_bit_index)) & 0x01;
        if (!bit) {
            send_logic_0 = true;
        }
    } else if (response_bit_index == 9) {
        wait_ack = true;
    }

    response_bit_index++;
}

/**
 * @brief Decode low pulse: check for reset or read data bit.
 */
static void swi_decode_low(void)
{
    // If we detect a low pulse > THRESHOLD_RESET => Enter STATE_RESET
    if (pulse_duration > THRESHOLD_RESET) {
        swi_state = STATE_RESET;
        bit_count = 0;
        current_byte = 0;
        send_logic_0 = true;
        response_index = 0;
        response_bit_index = 0;
        //debug_log("reset: %lu us\n", pulse_duration);
        return;
    }

    // If the low pulse is short => '1', else '0'
    uint8_t bit_val = (pulse_duration < THRESHOLD_BIT) ? 1 : 0;

    // If we're in TRANSMIT, we can output bits in response to host edges or timing.
    if (swi_state == STATE_TRANSMIT) {
        if (wait_ack) {
            wait_ack = false;
            if (!bit_val) {
                response_index++;
                response_bit_index = 0;
                next_bit();
            } else {
                swi_state = STATE_RECEIVE_CMD;
            }
        }
        return;
    }

    // Normal Reception of Bits, decode bits on rising edges.
    current_byte = (current_byte << 1) | bit_val;
    bit_count++;

    if (bit_count >= 8) {
        //debug_toggle_pin();
        //debug_log("byte recv = 0x%02X\n", current_byte);
        if (swi_state == STATE_RECEIVE_CMD) {
            uint8_t opcode  = (current_byte & AT21_OPCODE_MASK) >> AT21_OPCODE_SHIFT;
            uint8_t dev_addr = (current_byte & AT21_ADDR_MASK) >> AT21_ADDR_SHIFT;
            uint8_t rw_bit   = (current_byte & AT21_RW_MASK);

            if (dev_addr == AT21_DEVICE_ADDR) {
                if (opcode == OPCODE_MANUFACTURER_ID) {
                    // If the Read/Write bit is set to a logic ‘0’ to indicate a write, the device
                    // will NACK (logic ‘1’) since the Manufacturer ID data is read-only.
                    if (rw_bit) {
                        // Prepare to transmit manufacturer ID
                        response_buffer_ptr = manuf_id;
                        response_buffer_size = sizeof(manuf_id);
                        response_bit_index = 0;
                        response_index = 0;
                        swi_state = STATE_TRANSMIT;
                        // We'll ACK on the next falling edge => set send_logic_0
                        send_logic_0 = true;
                    }
                } else if (opcode == OPCODE_EEPROM_ACCESS) {
                    if (!rw_bit) {
                        // Next byte => memory address
                       swi_state = STATE_RECEIVE_ADDR;
                    } else {
                        response_bit_index = 0;
                        swi_state = STATE_TRANSMIT;
                    }
                    // Prepare to transmit EEPROM bytes.
                    response_buffer_ptr = eeprom_buffer;
                    response_buffer_size = sizeof(eeprom_buffer);
                    response_bit_index = 0;
                    // Typically we might set send_logic_0 = true if the spec requires an ACK after the address byte.
                    send_logic_0 = true;
                } else {
                    // Unknown command => go idle
                    swi_state = STATE_IDLE;
                }
            }
        } else if (swi_state == STATE_RECEIVE_ADDR) {
            response_index = current_byte;
            response_bit_index = 0;
            // Transition to transmit
            swi_state = STATE_RECEIVE_CMD;
            // Typically we might set send_logic_0 = true if the spec requires an ACK after the address byte.
            send_logic_0 = true;
        } else if (swi_state == STATE_RECEIVE_DATA) {
            // If the end of the EEPROM/Manufacurer ID is reached, then the Address Pointer will “roll over” back to the
            // beginning address of that region.
            if (response_index >= response_buffer_size) {
                response_index = 0;
            }
            response_buffer_ptr[response_index++] = current_byte;
            // Typically we might set send_logic_0 = true if the spec requires an ACK after the address byte.
            send_logic_0 = true;
        }

        bit_count = 0;
        current_byte = 0;
    }
}

/**
 * @brief Main entry point.
 */
int main(void)
{
    system_clock_config();
    mx_gpio_init();
    mx_tim1_init();
    mx_exti_init();
    debug_init();
    debug_log("AT21CS11 emulation start\n");

    // Wait for enable is low.
    while (LL_GPIO_IsInputPinSet(CHIP_ENABLE_GPIO_Port, CHIP_ENABLE_PIN));

    // Capture initial state of the SWI pin to detect future transitions
    swi_state = STATE_IDLE;
    uint32_t last_pin = LL_GPIO_ReadInputPort(SWI_GPIO_Port) & SWI_PIN;

    /**
     * @note Polling is used instead of IRQ-based edge detection for timing-critical sections.
     *       This avoids the latency and stack overhead associated with interrupt handling,
     *       which is crucial in bit-banged protocols like SWI where response timing must be precise.
     */
    for (;;) {
        // Wait pin change
        uint32_t port = LL_GPIO_ReadInputPort(SWI_GPIO_Port);
        if ((port & SWI_PIN) != last_pin) {
            if (!last_pin) {
                pulse_duration = LL_TIM_GetCounter(TIM1);
                LL_TIM_SetCounter(TIM1, 0);
                swi_decode_low();	
            } else {
                if (send_logic_0) {
                    LL_GPIO_ResetOutputPin(SWI_GPIO_Port, SWI_PIN);
                    delay_us(MIN_LOW_PULSE);
                    LL_GPIO_SetOutputPin(SWI_GPIO_Port, SWI_PIN);
                    send_logic_0 = false;
                    last_pin ^= SWI_PIN;  // Simulate an edge transition because we forced the pin low manually
                                          // Without this, the next external edge from the host could be missed
                    pulse_duration = LL_TIM_GetCounter(TIM1);
                    /**
                     * Set the counter to MIN_LOW_PULSE instead of zero because this account for the 
                     * low pulse duration already driven.
                     * The host expects to read the bit level ~1.8µs after the rising edge.
                     * By aligning the timer here, we ensure the host has enough time to sample the bit.
                     */
                    LL_TIM_SetCounter(TIM1, MIN_LOW_PULSE); 
                    
                } else {
                    pulse_duration = LL_TIM_GetCounter(TIM1);
                    LL_TIM_SetCounter(TIM1, 0); 
                }

                if (swi_state == STATE_TRANSMIT) {
                    next_bit();
                } else if (swi_state == STATE_RESET) {
                    // If the preceding low pulse was reset, this is the "high" portion.
                    // Check if it's within discovery range:
                    if (pulse_duration >= DISCOVERY_HIGH_MIN) { 
                        // Return to normal command reception
                        swi_state = STATE_RECEIVE_CMD;
                        bit_count = 0;
                        current_byte = 0;
                    }
                }
            }
            last_pin ^= SWI_PIN;
        }
    }
}
