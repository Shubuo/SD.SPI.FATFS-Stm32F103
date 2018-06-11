#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>


void SPI_Release(void);
void SPI_SendByte(uint8_t bt);
int SD_Init(void);
void LED_Blink(uint8_t tempo);
void Error(void);

uint8_t SPIx_WriteRead(uint8_t Byte);
uint8_t SD_Read_Block (uint8_t *buff, uint32_t sector);
uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba);
uint8_t SPI_ReceiveByte(void);
uint8_t SD_cmd (uint8_t cmd, uint32_t arg);
uint8_t SPI_wait_ready(void);
