
#include "hardware.h"

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

#define PIN_USB_DET		GPIO_PIN_1
#define PORT_USB_DET	GPIOB

#define PIN_RF_RX			GPIO_PIN_2
#define PORT_RF_RX		GPIOA


void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  HAL_SPI_Init(&hspi1);

}


/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
GPIO_InitTypeDef GPIO_InitStruct;


	//Configure input USB Detect:
	GPIO_InitStruct.Pin = PIN_USB_DET;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PORT_USB_DET, &GPIO_InitStruct);
	
	//Configure output RF RX:
	GPIO_InitStruct.Pin = PIN_RF_RX;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(PORT_RF_RX, &GPIO_InitStruct);
	
	  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

}


//USB detect
bool USB_Cable_Detect(void)
{
	return (HAL_GPIO_ReadPin (PORT_USB_DET,PIN_USB_DET)) ? true : false;
}

//RF_RX On
void RF_RX_On (void)
{
	HAL_GPIO_WritePin(PORT_RF_RX, PIN_RF_RX, GPIO_PIN_SET);
}

//RF_RX Off
void RF_RX_Off (void)
{
	HAL_GPIO_WritePin(PORT_RF_RX, PIN_RF_RX, GPIO_PIN_RESET);
}


