/**
AN EXAMPLE WHICH DEMONSTRATES DIFFERENT APPROACHES ON HOW TO USE UART ON STM32F756
*/
/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
#include "my_global.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart6; //unused. uart7 is initialized

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void My_UART7_Init(void);
void LL_SendByteUSART(uint8_t data);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#define MANY_SYMBOLS
int main(void)
{
  uint8_t  rxBuffer[MAX_RX_BUFFER];
	uint16_t rxBufferSize= 0;
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock. */
  SystemClock_Config();
  SystemCoreClockUpdate();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_USART6_UART_Init();
  My_UART7_Init();
  /* USER CODE BEGIN 2 */
        currentLengthBuffer = 0;
		symbols_found.symbol13 = 0;
        LL_SendByteUSART('M'); LL_SendByteUSART('C'); LL_SendByteUSART('U'); LL_SendByteUSART(' ');
        LL_SendByteUSART('R'); LL_SendByteUSART('E'); LL_SendByteUSART('A'); LL_SendByteUSART('D'); LL_SendByteUSART('Y');
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
		// http://easystm32.ru/interfaces/16-uart-in-stm32-part-2
		//p1045 of MCU manual
		#ifdef POLLING_APPROACH
			if (UART7->ISR & USART_ISR_RXNE) {			
		     uint8_t UART_data = UART7->RDR;
			#ifdef MANY_SYMBOLS	
				 if (UART_data!=STOP_SYMBOL) {
						rxBuffer[rxBufferSize] = UART_data;
					  rxBufferSize+=1;
				 } else {
				   rxBuffer[rxBufferSize] = 0x00;
					 LL_SendByteUSART('O');
					 LL_SendByteUSART('K');
					 LL_SendByteUSART('|');
					 char symbs[20];
					 int totalbytes2 = sprintf(symbs, "total length: %d", rxBufferSize);
					 uint8_t symbs_indx = 0;
					 while ( symbs_indx<=totalbytes2 ) {
							LL_SendByteUSART(symbs[symbs_indx]);
						  symbs_indx++;
					 }
					 rxBufferSize = 0;
				 }
			#else	 
				 
				 LL_SendByteUSART('_');
				 LL_SendByteUSART(UART_data);
	    #endif   
			}
		#else
		if (symbols_found.symbol13 == 1) {
			char symbs[20];
			int totalbytes2 = sprintf(symbs, "total length: %d", currentLengthBuffer);
			uint8_t symbs_indx = 0;
					 while ( symbs_indx<=totalbytes2 ) {
							LL_SendByteUSART(symbs[symbs_indx]);
						  symbs_indx++;
					 }
			symbols_found.symbol13 = 0;		 
		}
		#endif
  }
  /* USER CODE END 3 */

}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
	
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
/*
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
*/
    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}




/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitTypeDef GPIO_InitStruct2;
	
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
	
  /*Configure GPIO pin : PI11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
	
	}

/* USER CODE BEGIN 4 */
	//low level initialization of USART6
	//see http://chipspace.ru/stm32-usart-1/
	//http://easystm32.ru/interfaces/15-uart-in-stm32-part-1
static void My_UART7_Init(void) { 
 __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    /**UART7 GPIO Configuration    
    PF7     ------> UART7_RX
    PF6     ------> UART7_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; /*GPIO_MODE_AF_PP;*/
    GPIO_InitStruct.Pull = GPIO_PULLUP;
       //GPIO_InitStruct.Pull = GPIO_NOPULL;
     
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
     
     
     
     RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;     
     PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
     PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_SYSCLK;
     if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
     {     }     
     __HAL_RCC_UART7_CLK_ENABLE();
     
       //now comes the initialization of USART6 params
          //baud rate. calculated as in p1002 MCU_ReferenceManual 
          //UART7->BRR = 0xAFC4;     //9600, over8 = 1; 8 bit data; no parity; 1 stop bit
         UART7->BRR = 0x0EA3; //115200, over8=1; 8 bit data; no parity 1 stop bit
                              //USARTDIV=2*216000000/115200 = 3750d = 0x0EA6; 0x6 >> 1 = 3h ; BRR=0x0EA3
         //UART7->CR3 |= USART_CR3_OVRDIS; //This bit is used to disable the receive overrun. Might be worth setting when receiving is invloved
             //do not set this, data are going to get lost!
               UART7->CR1 = 0x00;
               UART7->CR2 = 0x00;
                    UART7->CR1 |= USART_CR1_RXNEIE;    //enable interrupt on ORE=1 (overrun) or RXNE=1 (rx signal) in the USART_ISR. They should be cleaned after I suppose
                    NVIC_EnableIRQ(UART7_IRQn);
               //USART6->CR1 &= ~USART_CR1_M;     //word length: 8 bit 
               //USART6->CR2 &= ~USART_CR2_STOP;  //1 stop bit
       //p1029
       //0b00000000000000000000000000000000
       //M1(bit28)M0(bit12) -> 0;0 (1 Start bit, 8 data bits, n stop bits) Keep default, no change
     UART7->CR1 |= USART_CR1_OVER8; //oversampling=8. involved in calculation of baud rate.
     UART7->CR1 |= USART_CR1_RE;    //enable receiver
     UART7->CR1 |= USART_CR1_TE;    //enable transmitter
     UART7->CR1 |= USART_CR1_UE;    //enable UART7
		
}


//Send one byte
void LL_SendByteUSART(uint8_t data) {
while(!(UART7->ISR & USART_ISR_TXE)) {}; //Wait for bit in SR Register
		UART7->TDR=data; //write data to UART register
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
