# EXAMPLE FOR CONFIGURING UART7 ON STM32F746NGH

This example shows how to configure and use UART. Check the main.c, *My_UART7_Init()* routine. 

Mind the order of initialization of UART - activate the alternative function of pins and only after this activate ports. Be sure about parameters of clock generator for UART. better to use SYSCLK for this.

The system Clock is configured as follow :
 * System Clock source = PLL (HSE)
 * SYSCLK(Hz) = 216000000
 * HCLK(Hz) = 216000000
 * AHB Prescaler = 1
 * APB1 Prescaler = 4
 * APB2 Prescaler = 2
 * HSE Frequency(Hz) = 25000000
 * PLL_M = 25
 * PLL_N = 432
 * PLL_P = 2
 * VDD(V) = 3.3
 * Main regulator output voltage = Scale1 mode
 * Flash Latency(WS) = 7
 
 Interrupt is defined in stm32f7xx_it.h file. Interrupt changes values of global variables from my_global.c, and these values are monitored in main cycle. One should keep interrupt handler as small as possible.
 
 This example is quite generic and it does not show any details of actual project.