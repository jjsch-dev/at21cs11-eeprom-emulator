/**
 * @file debug.h
 * @brief Implementation of debug support functions.
 *
 * Provides UART-based logging and GPIO pin toggling for debugging purposes.
 * Functions are conditionally compiled based on ENABLE_UART_DEBUG and ENABLE_DEBUG_PIN.
 */

#ifndef DEBUG_H
#define DEBUG_H

#define UART_TX_PIN         LL_GPIO_PIN_2
#define UART_TX_GPIO_Port   GPIOA

#define DBG_PIN             LL_GPIO_PIN_14
#define DBG_GPIO_Port       GPIOA

#undef  ENABLE_DEBUG_PIN
#undef  ENABLE_UART_DEBUG
    
/**
 * @brief  Initialize debug peripherals (UART TX pin and/or debug GPIO) if enabled.
 */
void debug_init(void);

#ifdef ENABLE_UART_DEBUG
/**
 * @brief  Send a formatted string over UART if ENABLE_UART_DEBUG is defined.
 * @param  fmt  printf-style format string.
 * @param  ...  Format arguments.
 */
void debug_log(const char *fmt, ...);

#else

/**
 * @brief  Dummy implementation of debug_log when ENABLE_UART_DEBUG is not defined.
 * This prevents compiler errors when debug_log is called.
 * @param  fmt  printf-style format string (not used).
 * @param  ...  Format arguments (not used).
 */
#define debug_log(fmt, ...) do {} while (0)

#endif // ENABLE_UART_DEBUG

#ifdef ENABLE_DEBUG_PIN
/**
 * @brief  Toggle the debug GPIO pin if ENABLE_DEBUG_PIN is defined.
 * Implemented as a macro.
 */
#define debug_toggle_pin() LL_GPIO_TogglePin(DBG_GPIO_Port, DBG_PIN)

#else
/**
 * @brief  Dummy implementation of debug_toggle_pin when ENABLE_DEBUG_PIN is not defined.
 */
#define debug_toggle_pin() do {} while(0)

#endif // ENABLE_DEBUG_PIN

#endif // DEBUG_H

