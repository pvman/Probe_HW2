/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "usb_device.h"
#include "hardware.h"
#include "fifo.h"
#include <stdbool.h>
#include "usbd_cdc_if.h"

uint16_t transmitBuffer_to_SPI[2]; //temp buffer
<<<<<<< HEAD
uint8_t transmitBuffer_to_USB[2];
uint16_t receiveBuffer_from_SPI[2]; //temp buffer

=======
uint16_t transmitBuffer_to_USB[2];
uint16_t receiveBuffer_from_SPI[2]; //temp buffer
>>>>>>> e201f230350beea4bf5953d17ee317575410912e

uint8_t	USB_Init_flag;

extern SPI_HandleTypeDef hspi1;

uint16_t SPI_received; //test for identity




/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

int main(void)
{
	uint16_t delay_temp, delay_count;
	
	
	SPI_received = 0;
	
	USB_Init_flag = 0;
	
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;	
	// remap pins to enable USB
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;


  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
<<<<<<< HEAD
	MX_USB_DEVICE_Init();
	
	transmitBuffer_to_SPI[0] = 0x01;
	transmitBuffer_to_SPI[1] = 0x02;

=======
//	MX_USB_DEVICE_Init();
	
	transmitBuffer_to_SPI[0] = 0;
	//transmitBuffer_to_SPI[1] = 0x02;
//	HAL_SPI_TransmitReceive_IT(&hspi1, &transmitBuffer_to_SPI[0], &receiveBuffer_from_SPI[0], 1);
>>>>>>> e201f230350beea4bf5953d17ee317575410912e
	
	FIFO_FLUSH(SPI_to_FIFO_to_USBtx)
	FIFO_FLUSH(USBrx_to_FIFO_to_SPI)
	
	delay_temp=HAL_GetTick();
	delay_count=500;

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (USB_Cable_Detect())
		{
			if (HAL_GetTick()-delay_temp>delay_count)
			{	
			
			
			if (!USB_Init_flag)
			{
				SystemClock_Config();
				MX_USB_DEVICE_Init();
				RF_RX_On();
			}
			}
//			temp = FIFO_IS_EMPTY(SPI_to_FIFO_to_USBtx);
			
<<<<<<< HEAD
			if (HAL_SPI_TransmitReceive_IT(&hspi1, &transmitBuffer_to_SPI[0], &receiveBuffer_from_SPI[0], 1) == HAL_OK)
			{	}
			
			
			if (!FIFO_IS_EMPTY(SPI_to_FIFO_to_USBtx))
			{
				if (FIFO_SPACE(SPI_to_FIFO_to_USBtx) < 1024)
				{
					transmitBuffer_to_USB[0] = FIFO_FRONT(SPI_to_FIFO_to_USBtx);
					FIFO_POP(SPI_to_FIFO_to_USBtx)
	

					CDC_Transmit_FS(transmitBuffer_to_USB, 1);		//send to USB		
				}
			}		
=======
			if (HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)transmitBuffer_to_SPI, (uint8_t*)receiveBuffer_from_SPI, 1) == HAL_OK)
			{
				
				
			}
			
			if (USB_Init_flag)
			{
				if (!FIFO_IS_EMPTY(SPI_to_FIFO_to_USBtx))
				{
					if (FIFO_SPACE(SPI_to_FIFO_to_USBtx) < 1024)
					{
						transmitBuffer_to_USB[0] = FIFO_FRONT(SPI_to_FIFO_to_USBtx);
						FIFO_POP(SPI_to_FIFO_to_USBtx)
						CDC_Transmit_FS((uint8_t*)transmitBuffer_to_USB, 2);		//send to USB		
					}
				}	
			}
			
>>>>>>> e201f230350beea4bf5953d17ee317575410912e
		}
		else
		{
			RF_RX_Off();
				//	if (USB_Init_flag)
					{
						MX_USB_DEVICE_DeInit();
						 delay_temp=HAL_GetTick();
					}
		}
		

		
		

		
		
		
  }


}








void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
if (hspi->Instance == hspi1.Instance)
{
	if	(SPI_received == receiveBuffer_from_SPI[0]) //test for identity
	{
		return; 
	}
	
	
	if (!FIFO_IS_FULL(SPI_to_FIFO_to_USBtx))
	{
		SPI_received = receiveBuffer_from_SPI[0];
		FIFO_PUSH (SPI_to_FIFO_to_USBtx, receiveBuffer_from_SPI[0]);
//		FIFO_PUSH (SPI_to_FIFO_to_USBtx, receiveBuffer_from_SPI[1]);
	}
}
}








/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  RCC_CRSInitTypeDef RCC_CRSInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
 //HAL_RCC_OscConfig(&RCC_OscInitStruct);
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  //HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  //HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_CRS_CLK_ENABLE();

  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;
  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
