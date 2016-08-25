

#include "stm32f0xx_hal.h"
#include <stdbool.h>

//SPI_HandleTypeDef hspi1;


void MX_SPI1_Init(void);
void MX_GPIO_Init(void);
bool USB_Cable_Detect(void);
void RF_RX_On (void);
void RF_RX_Off (void);
