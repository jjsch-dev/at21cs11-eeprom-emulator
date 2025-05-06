/**
 * @file debug.c
 * @brief Implementation of debug support functions.
 *
 * Provides UART-based logging and GPIO pin toggling for debugging purposes.
 * Functions are conditionally compiled based on ENABLE_UART_DEBUG and ENABLE_DEBUG_PIN.
 */

#include "debug.h"
#include <stdarg.h>
#include <stdio.h>

#ifdef ENABLE_UART_DEBUG
  #include "py32f0xx_ll_usart.h"
#endif

#include "py32f0xx_ll_gpio.h"
#include "py32f0xx_ll_bus.h"
#include "py32f0xx_ll_utils.h"

#ifdef ENABLE_UART_DEBUG
/**
 * @brief  Initialize USART1 for TX-only debug output at 115200 bps.
 */
static void uart_debug_init(void)
{
    LL_GPIO_InitTypeDef gpio_init = {0};
	
    // Enable GPIOA clock
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	
    // Configure PA2 for USART1 TX if needed.
    gpio_init.Pin = UART_TX_PIN;
    gpio_init.Mode = LL_GPIO_MODE_ALTERNATE; 
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH; 
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    gpio_init.Alternate = LL_GPIO_AF1_USART1; 
    LL_GPIO_Init(UART_TX_GPIO_Port, &gpio_init);

    // Enable USART1 clock
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

	// Configure USART for TX only, 8N1, 115200 baud
    LL_USART_SetBaudRate(USART1, SystemCoreClock,
                        LL_USART_OVERSAMPLING_16, 115200);
    LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
    LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
    LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
    LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
    // Only TX 
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX);
    LL_USART_Enable(USART1);
}
#endif // ENABLE_UART_DEBUG

// -----------------------------
// Public API Implementation
// -----------------------------

void debug_init(void)
{
#ifdef ENABLE_UART_DEBUG
    uart_debug_init();
#endif

#ifdef ENABLE_DEBUG_PIN
    // Enable GPIOA clock
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    
    // Configure debug pin as push-pull output
    LL_GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin        = DBG_PIN;
    gpio_init.Mode       = LL_GPIO_MODE_OUTPUT;
    gpio_init.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull       = LL_GPIO_PULL_NO;
    LL_GPIO_Init(DBG_GPIO_Port, &gpio_init);
#endif
}

#ifdef ENABLE_UART_DEBUG
void debug_log(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    for (char *p = buf; *p; ++p) {
        LL_USART_TransmitData8(USART1, *p);
        while (!LL_USART_IsActiveFlag_TC(USART1)); // Wait for TX buffer empty
        LL_USART_ClearFlag_TC(USART1);
    }
}
#endif

