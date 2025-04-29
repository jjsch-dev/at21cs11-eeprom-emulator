/**
 * @file debug.h
 * @brief Debug support: UART logging and debug-pin toggling.
 */

#ifndef DEBUG_H
#define DEBUG_H

#define UART_TX_PIN       	LL_GPIO_PIN_2
#define UART_TX_GPIO_Port 	GPIOA

#define DBG_PIN             LL_GPIO_PIN_14
#define DBG_GPIO_Port    	GPIOA

#undef  ENABLE_DEBUG_PIN
#undef  ENABLE_UART_DEBUG

/**
 * @brief  Initialize debug peripherals (UART TX pin and/or debug GPIO) if enabled.
 */
void debug_init(void);

/**
 * @brief  Send a formatted string over UART if ENABLE_UART_DEBUG is defined.
 * @param  fmt  printf-style format string.
 * @param  ...  Format arguments.
 */
void debug_log(const char *fmt, ...);

/**
 * @brief  Toggle the debug GPIO pin if ENABLE_DEBUG_PIN is defined.
 */
void debug_toggle_pin(void);

#endif // DEBUG_H

