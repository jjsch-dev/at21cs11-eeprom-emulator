#include "py32f0xx.h"      // Device header that includes LL definitions
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

#define ENABLE_START_CONDITION_DETECT

//  SWI Pin and Timing Definitions
#define SWI_PIN           	LL_GPIO_PIN_10
#define SWI_GPIO_Port     	GPIOA

#define ENABLE_PIN          LL_GPIO_PIN_1
#define ENABLE_GPIO_Port    GPIOA

// Device Address Decoding Macros
/* 
   For an 8-bit device address:
   ┌────┬────┬────┬────┬────┬────┬────┬────┐
   │Bit7│Bit6│Bit5│Bit4│Bit3│Bit2│Bit1│Bit0│
   ├────┼────┼────┼────┼────┼────┼────┼────┤
   │    │  OPCODE      │ A2 │ A1 │ A0 │R/W │
   └────┴────┴────┴────┴────┴────┴────┴────┘
   Bits 7..4: opcode
   Bits 3..1: device address (3 bits)
   Bit  0   : read/write bit
*/

#define AT21_OPCODE_MASK  0xF0  /* Bits 7..4 */
#define AT21_OPCODE_SHIFT 4

#define AT21_ADDR_MASK    0x0E  /* Bits 3..1 */
#define AT21_ADDR_SHIFT   1

#define AT21_RW_MASK      0x01  /* Bit 0      */

#define OPCODE_EEPROM_ACCESS    		0x0A  /* Read/Write the contents of the main memory array. */
#define OPCODE_SEC_REG_ACCESS				0x0B	/* Read/Write the contents of the Security register. */
#define OPCODE_LOCK_SEC_REG 				0X02	/* Permanently lock the contents of the Security register. */
#define OPCODE_ROM_ZONE_REG_ACCESS 	0x07	/* Inhibit further modification to a zone of the EEPROM array. */
#define OPCODE_FREEZE_ROM						0x01	/* Permanently lock the current state of the ROM Zone registers. */
#define OPCODE_MANUFACTURER_ID 			0x0C	/* Query manufacturer and density of device. */
#define OPCODE_STANDARD_SPEED				0x0D	/* Switch to Standard Speed mode operation (AT21CS01 only
																						 command, the AT21CS11 will NACK this command). */
#define OPCODE_HIGH_SPEED						0x0E	/* Switch to High-Speed mode operation (AT21CS01 power‑on default.
																					   AT21CS11 will ACK this command). */

#define AT21_DEVICE_ADDR          	0x00 	/* Define your device address (A2..A0) */

/* Timing thresholds in microseconds (adjust as needed) */
#define THRESHOLD_RESET      100    // If a low pulse >100µs => reset trigger
#define DISCOVERY_HIGH_MIN   10     // Discovery high pulse min
#define DISCOVERY_HIGH_MAX   150    // Discovery high pulse max
#define THRESHOLD_BIT        5      // Low pulse <10µs => logic '1'; otherwise '0'
#define MIN_LOW_PULSE        5      // For driving a '0' in transmit
#define TIMEOUT_START_US     220   	// Timeout if no edge in 200uS (start/stop condition)

//  SWI EEPROM Emulation States
typedef enum {
    STATE_RESET = 0,      // Waiting for discovery high pulse after reset low
    STATE_RECEIVE_CMD,    // Receiving the device address/command byte
    STATE_RECEIVE_ADDR,   // Receiving memory address byte (for write or read address pointer)
    STATE_RECEIVE_DATA,   // Receiving data bytes from host (write operation)
    STATE_TRANSMIT,       // Transmitting data back to host
    STATE_IDLE
} SWI_State_t;

volatile SWI_State_t swi_state = STATE_IDLE;


//  Variables for Reception
volatile uint32_t last_edge_time = 0;       // Timestamp of the last edge (Timer1, 1µs resolution)
volatile uint32_t pulse_duration = 0;       // Duration (µs) of the most recent pulse
volatile uint32_t prev_pulse_duration = 0;  // Optional: store previous pulse for debugging/logic
volatile uint8_t  bit_count = 0;            // Number of bits received for current byte
volatile uint8_t  current_byte = 0;         // Accumulated byte from the host
volatile uint8_t  command = 0;              // Last command byte received

/* A flag indicating that after receiving 8 bits the next rising edge is the ACK slot */
volatile bool send_logic_0 = false;

/* A flag used in transmit mode to indicate that we are waiting for the host's ACK (ninth bit) */
volatile bool wait_ack = false;

//  Variables for Transmission
volatile uint8_t* response_buffer_ptr; 		// Pointer to working response buffer
volatile uint8_t  response_buffer_size; 	// Size of the working response buffer
volatile uint8_t response_index = 0;      // Current byte index in the response buffer
volatile uint8_t response_bit_index = 0;  // Current bit index (MSB first) in the current response byte

//  Function Prototypes
static void SystemClock_Config(void);
void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_EXTI_Init(void);
static inline uint32_t get_us(void);
static void delay_us(uint32_t us);
static void swi_decode_low(void);

/**
 * Helper Function: get_us (Timer1)
 **/
static inline uint32_t get_us(void)
{
	return LL_TIM_GetCounter(TIM1);
}

/**
 * Helper Function: delay_us using Timer1 (busy-wait)
 */
static void delay_us(uint32_t us)
{
	uint32_t start = get_us();
	while((get_us() - start) < us);
}

/**
 * System Clock Configuration: 24 MHz from HSI
 */
static void SystemClock_Config(void)
{
  LL_RCC_HSI_Enable();
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
  while (LL_RCC_HSI_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update global SystemCoreClock(or through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(24000000);
  /* Re-init frequency of SysTick source */
  LL_Init1msTick(24000000);
}

/**
 * TIM1 Initialization using LL:
 * - 1 µs tick with prescaler
 * - Channel 2: Output compare for timeout (TIM_OCMODE_TIMING)
 */
static void MX_TIM1_Init(void)
{
	/* Enable clock for TIM1 */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
	
	/* Set TIM1 prescaler for 1 µs tick */
	LL_TIM_SetPrescaler(TIM1, (SystemCoreClock / 1000000) - 1); // 1µs tick
	LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetAutoReload(TIM1, 0xFFFF);
	LL_TIM_DisableARRPreload(TIM1);

#ifdef ENABLE_START_CONDITION_DETECT
	/* --- Configure Channel 2 for Timeout (Output Compare Timing mode) --- */
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_FROZEN);
	LL_TIM_OC_SetCompareCH2(TIM1, TIMEOUT_START_US);
	// Do not start channel 2 interrupt yet; it will be enabled when scheduling a timeout.
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
 *  GPIO Initialization: SWI-input and output using LL
 */  
void MX_GPIO_Init(void)
{
	/* Enable clock for GPIOA */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	 
	LL_GPIO_InitTypeDef gpio_init = {0};

	// Configure PA10 for TIM1_CH3 (open-drain alternate function)
	gpio_init.Pin = SWI_PIN;
	gpio_init.Mode = LL_GPIO_MODE_OUTPUT; 
	gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	gpio_init.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(SWI_GPIO_Port, &gpio_init);
	LL_GPIO_SetOutputPin(SWI_GPIO_Port, SWI_PIN);
	
	// Configure PA1 as input for enable (low /disable)
	gpio_init.Pin = ENABLE_PIN;
	gpio_init.Mode = LL_GPIO_MODE_INPUT;
	gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH; 
	gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	gpio_init.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(ENABLE_GPIO_Port, &gpio_init);
	LL_GPIO_SetOutputPin(ENABLE_GPIO_Port, ENABLE_PIN);
}

/**
 * @brief  Configure EXTI line for ENABLE interrupts on PA10.
 */
static void MX_EXTI_Init(void)
{
  /* Triggerred by falling edge */
  LL_EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.Line = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**
   * Enable interrupt
   * - EXTI0_1_IRQn for PA/PB/PC[0,1]
   * - EXTI2_3_IRQn for PA/PB/PC[2,3]
   * - EXTI4_15_IRQn for PA/PB/PC[4,15]
  */
  NVIC_SetPriority(EXTI0_1_IRQn, 0);
  NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/**
 * @brief  When enable goes low, the IRQ is activated, then we set the SWI pin to high impedance to 
 * 			   release the (one wire port) and wait in a loop for the enable signal.
 */
void EXTI0_1_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag(LL_EXTI_LINE_1)) {
    swi_state   				= STATE_IDLE;
		bit_count   				= 0;
		current_byte			 	= 0;
		send_logic_0  		 	= false;
		response_index     	= 0;
		response_bit_index 	= 0;	
		LL_GPIO_SetOutputPin(SWI_GPIO_Port, SWI_PIN);
		
		// Wait in the IRQ for enable signal.
		while (LL_GPIO_IsInputPinSet(ENABLE_GPIO_Port, ENABLE_PIN)); 
		
    LL_EXTI_ClearFlag(LL_EXTI_LINE_1);
  }
}
  
#ifdef ENABLE_START_CONDITION_DETECT
/**
 * TIM1 Interrupt Handler (for detect Start/Stop condition with CC2)
 */
void TIM1_CC_IRQHandler(void)
{
	/* --- Handle Timeout on Channel 2 --- */
	if (LL_TIM_IsActiveFlag_CC2(TIM1)) {
		LL_TIM_ClearFlag_CC2(TIM1);
		if ((swi_state == STATE_TRANSMIT) || (swi_state == STATE_RECEIVE_ADDR) || (swi_state == STATE_RECEIVE_DATA)) {
			swi_state   = STATE_RECEIVE_CMD;
			bit_count   = 0;
			current_byte= 0;
		}
		
		LL_TIM_SetCounter(TIM1, 0); // Reset the counter
	}
}
#endif

/**
 * @brief  Advance to the next transmit bit, handle ACK/rollover.
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
	// The strategy of sending logic_0 on the falling edge (to shorten the response time) has 
	// the disadvantage that the bit count is ahead of the ack.
	} else if (response_bit_index == 9) {
		wait_ack = true;
	}

	response_bit_index++;
}

/**
 * @brief  Decode a low-pulse duration: reset or data bit.
 * @note   Uses global 'pulse_duration'.
 *         Handle rising edge logic (e.g., discovery low pulse)
 */
static void swi_decode_low(void)
{
	/* If we detect a low pulse > THRESHOLD_RESET => Enter STATE_RESET */
	if (pulse_duration > THRESHOLD_RESET) {
		swi_state   				= STATE_RESET;
		bit_count   				= 0;
		current_byte			 	= 0;
		send_logic_0  		 	= true;
		response_index     	= 0;
		response_bit_index 	= 0;

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
		debug_toggle_pin();
		//debug_log("byte recv = 0x%02X\n", current_byte);

		// We received a full byte
		if (swi_state == STATE_RECEIVE_CMD) {
			// We interpret current_byte as device address.
			uint8_t opcode  = (current_byte & AT21_OPCODE_MASK) >> AT21_OPCODE_SHIFT;
			uint8_t devAddr = (current_byte & AT21_ADDR_MASK) >> AT21_ADDR_SHIFT;
			uint8_t rwBit   = (current_byte & AT21_RW_MASK);
		
			// Now handle opcode, address, etc.
			if(devAddr == AT21_DEVICE_ADDR) {
				if(opcode == OPCODE_MANUFACTURER_ID) {
					// If the Read/Write bit is set to a logic ‘0’ to indicate a write, the device
					// will NACK (logic ‘1’) since the Manufacturer ID data is read-only.
					if (rwBit) {
						// Prepare to transmit manufacturer ID
						response_buffer_ptr  	= manuf_id;
						response_buffer_size	= sizeof(manuf_id);
						response_bit_index		= 0;
						response_index    		= 0;
						swi_state         		= STATE_TRANSMIT;
						// We'll ACK on the next falling edge => set send_logic_0
						send_logic_0        		= true;
					}
			 	} else if (opcode == OPCODE_EEPROM_ACCESS) {
					if (!rwBit) {
						// Next byte => memory address
						swi_state  = STATE_RECEIVE_ADDR;
					} else {
						response_bit_index = 0;
						swi_state = STATE_TRANSMIT;	
					}
					
					// Prepare to transmit EEPROM bytes.
					response_buffer_ptr	  = eeprom_buffer;
					response_buffer_size	= sizeof(eeprom_buffer);
					response_bit_index		= 0;
					//response_index    		= 0;
					// We'll ACK on the next falling edge => set send_logic_0
					send_logic_0 = true;
				} else {
					// Unknown command => go idle
					swi_state  = STATE_IDLE;
				}
			}
   	} else if (swi_state == STATE_RECEIVE_ADDR) {
			response_index     = current_byte;
			response_bit_index = 0;
			// Transition to transmit
			swi_state  = STATE_RECEIVE_CMD;
			// Typically we might set send_logic_0 = true if the spec 
			// requires an ACK after the address byte. 
			send_logic_0 = true;
   	} else if (swi_state == STATE_RECEIVE_DATA) {
			// If the end of the EEPROM/Manufacurer ID is reached, then the Address Pointer will “roll over” back to the
			// beginning address of that region.
			if (response_index >= response_buffer_size) {
				response_index = 0;
			}

			response_buffer_ptr[response_index++] = current_byte;
			// requires an ACK after the address byte. 
			send_logic_0 = true;	
		}
            
    bit_count   = 0;
    current_byte= 0;
	}
}
		
/**
 * Main
  */
int main(void)
{
	SystemClock_Config();
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_EXTI_Init();

	debug_init();
  debug_log("AT21CS11 emulation start\n");
	
	// Wait for enable is low.
	while (LL_GPIO_IsInputPinSet(ENABLE_GPIO_Port, ENABLE_PIN)); 
	
	/* Start in IDLE or STATE_RECEIVE_CMD as needed */
	swi_state = STATE_IDLE;
	uint32_t last_pin = LL_GPIO_ReadInputPort(SWI_GPIO_Port) & SWI_PIN;

	for (;;) {
		// Wait pin change
		uint32_t port = LL_GPIO_ReadInputPort(SWI_GPIO_Port); 
		
		if ((port & SWI_PIN) != last_pin) {
			if (!last_pin) {
				pulse_duration = LL_TIM_GetCounter(TIM1);
				LL_TIM_SetCounter(TIM1, 0);	
				swi_decode_low();	
			} else {
				
				// Drive a logic-0 on the SWI_OUT line (low for MIN_LOW_PULSE µs).
				if (send_logic_0) {
					LL_GPIO_ResetOutputPin(SWI_GPIO_Port, SWI_PIN);
					delay_us(MIN_LOW_PULSE);
					LL_GPIO_SetOutputPin(SWI_GPIO_Port, SWI_PIN);
					send_logic_0 = false;
					last_pin ^= SWI_PIN;
				
					pulse_duration = LL_TIM_GetCounter(TIM1);
					LL_TIM_SetCounter(TIM1, MIN_LOW_PULSE); // Reset the counter with the MIN_LOW_PULSE time.
				} else {
					pulse_duration = LL_TIM_GetCounter(TIM1);
					LL_TIM_SetCounter(TIM1, 0); 
				}
						
				if (swi_state == STATE_TRANSMIT) {
					next_bit();
				}else if (swi_state == STATE_RESET) {
					// If the preceding low pulse was reset, this is the "high" portion.
					// Check if it's within discovery range:
					if (pulse_duration >= DISCOVERY_HIGH_MIN) { 
						// Return to normal command reception
						swi_state   = STATE_RECEIVE_CMD;
						bit_count   = 0;
						current_byte= 0;
					}
				}
			}	
			
			last_pin ^= SWI_PIN;
		} 
	}	
}

