#include "SD.h"
#include "LCD162A.h"
#include "ff_gen_drv.h"
#include "user_diskio.h"

#define CMD0 		(0x40+0) // GO_IDLE_STATE
#define CMD1 		(0x40+1) // SEND_OP_COND (MMC)
#define ACMD41 	(0xC0+41) // SEND_OP_COND (SDC)
#define CMD8 		(0x40+8) // SEND_IF_COND
#define CMD9 		(0x40+9) // SEND_CSD
#define CMD16 (0x40+16) // SET_BLOCKLEN
#define CMD17 (0x40+17) // READ_SINGLE_BLOCK
#define CMD24 (0x40+24) // WRITE_BLOCK
#define CMD55 (0x40+55) // APP_CMD
#define CMD58 (0x40+58) // READ_OCR

#define CS_SD_GPIO_PORT GPIOA
#define CS_SD_PIN GPIO_PIN_3
#define CS_SD_LOW() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_RESET)
#define CS_SD_HIGH() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_SET)

extern SPI_HandleTypeDef hspi2;
uint8_t i = 0;
uint32_t timer = 0;
uint8_t ocr[4];
char str1[60]={0};
uint8_t tmr;
int Init = 0;

/* Error -----------------------------------------------*/

void Error(void){

  while(1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_Delay(200);
	}

  
}


/* SPIx_WriteRead -----------------------------------------------*/

uint8_t SPIx_WriteRead(uint8_t Byte){

  uint8_t receivedbyte = 0;

  HAL_SPI_TransmitReceive(&hspi2,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000); 

  return receivedbyte;

}


/* SPI_Release -----------------------------------------------*/

void SPI_Release(void)

{

  SPIx_WriteRead(0xFF);

}

//SPI_SendByte -----------------------------------------------

void SPI_SendByte(uint8_t bt)

{

  SPIx_WriteRead(bt);

}

//SPI_ReceiveByte -----------------------------------------------

uint8_t SPI_ReceiveByte(void)

{

  uint8_t bt = SPIx_WriteRead(0xFF);

  return bt;

}



/* SD_cmd -----------------------------------------------*/

uint8_t SD_cmd (uint8_t cmd, uint32_t arg)

{

  uint8_t n, res;
	
	// Select the card

		CS_SD_HIGH();
	
		CS_SD_LOW();
		
	
	// Send a command packet
		SPI_SendByte(cmd); // Start + Command index
		SPI_SendByte((uint8_t)(arg >> 24)); // Argument[31..24]
		SPI_SendByte((uint8_t)(arg >> 16)); // Argument[23..16]
		SPI_SendByte((uint8_t)(arg >> 8)); // Argument[15..8]
		SPI_SendByte((uint8_t)arg); // Argument[7..0]
	
		n = 0x01; // Dummy CRC + Stop
		if (cmd == CMD0) {n = 0x95;} // Valid CRC for CMD0(0)
		if (cmd == CMD8) {n = 0x87;} // Valid CRC for CMD8(0x1AA)
		if (cmd == CMD55) {n = 0x65;}
		if (cmd == CMD1) {n = 0xF9;}
		if (cmd == ACMD41 && arg == 0x40000000) {n = 0x77;}
		if (cmd == ACMD41 && arg == 0x00000000) {n = 0xE5;}
		SPI_SendByte(n);
		
	// Receive a command response
		n = 10; // Wait for a valid response in timeout of 10 attempts

		do {
    res = SPI_ReceiveByte();
		} 
		while ((res & 0x80) && --n);
		return res;	
}
// SD_Write_Block -----------------------------------------------

uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba)

{

  uint8_t result;
  uint16_t cnt;
  result=SD_cmd(CMD24,lba); 

  if (result!=0x00){ 
		LCD_LIMPA();
		ENVIA_STRING_LCD("Erro1 WriteBlock");
		HAL_Delay(1000);
		
	}
	
  SPI_Release();
  SPI_SendByte (0xFE); 

  for (cnt=0;cnt<512;cnt++) 
		SPI_SendByte(buff[cnt]); 

  SPI_Release(); 
  SPI_Release();

  result=SPI_ReceiveByte();
	
  if ((result&0x05)!=0x05){ 
		LCD_LIMPA();
		ENVIA_STRING_LCD("Erro2 WriteBlock");
		HAL_Delay(1000);
	}

  cnt=0;
	
  do { 

    result=SPI_ReceiveByte();
    cnt++;
  } 
	
	while ( (result!=0xFF)&&(cnt<0xFFFF) );

  if (cnt>=0xFFFF) {
		LCD_LIMPA();
		ENVIA_STRING_LCD("Erro3 WriteBlock");
		HAL_Delay(1000);
	}
	
  return 0;

}

/* SD_Read_Block -----------------------------------------------*/

uint8_t SD_Read_Block (uint8_t *buff, uint32_t sector)

{
	
uint8_t result;
uint16_t cnt;
	
	// Reads a block of the size selected by SET_BLOCKLEN
	// if SDHC e SDXC size are fixed 512 bytes
	
	if (SD_cmd(CMD17, sector)!=0x00) 		
	{		
//		LCD_LIMPA();
//		ENVIA_STRING_LCD("Erro1 ReadBlock");
//		HAL_Delay(800); 	
	}
	
	cnt=0;

  do{ 
		
    result = SPI_ReceiveByte();
    cnt++;
		
  }while ((result!=0xFE)&&(cnt<0xFFFF));

  if (cnt>=0xFFFF)
		{
//		LCD_LIMPA();
//		ENVIA_STRING_LCD("Erro2 ReadBlock");
//		HAL_Delay(800); 
		}

  for (cnt=0;cnt<512;cnt++) 
		buff[cnt]=SPI_ReceiveByte(); 

  SPI_Release(); 
  SPI_Release();
	
  return 0;

}
/* LED_Blink -----------------------------------------------*/

void LED_Blink(uint8_t tempo){

	for(uint8_t i = 0;i<10;i++){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(tempo);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_Delay(tempo);
	}
}
	
/* SD_Init -----------------------------------------------*/

int SD_Init(void)

{
	timer = 0;
	Init = 0;

  while(Init == 0){
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_Delay(200);
		
		
	//  The card will enter its native operating mode and go ready to accept native command after 74 clock pulse MO=1 CS=1.
	
			CS_SD_HIGH();	
			for(i=0;i<200;i++) 							//Minimo 74 pulsos de clock	
			SPI_Release(); 									// MOSI nivel alto durante os pulsos de clock
			
	// Send a CMD0 with CS low to reset the card. 
	// If the CS signal is low, the card enters SPI mode and responds R1 with In Idle State bit (0x01)
			
			CS_SD_LOW();
		
			if (SD_cmd(CMD0, 0) == 1) 			// Enter Idle state
			{
				SPI_Release();
				
				if (SD_cmd(CMD8, 0x1AA) == 1) // SDv2 - NAO NECESSARIO PARA INICIALIZAR
				{				
					for (i = 0; i < 4; i++) 
					ocr[i] = SPI_ReceiveByte();
					
	// The lower 12 bits in the return value 0x1AA means that the card is SDC version 2 
	// and it can work at voltage range of 2.7 to 3.6 volts. 
					
						if (ocr[2] == 0x01 && ocr[3] == 0xAA) 
						{
							
	// PROBLEMA
	// Nao pode enviar ACMD41, 0x40000000 , retorna somente 0xFF
							
							tmr = 0x02;					
							SPI_Release();
														
							while(tmr != 0x00){
							SPI_Release();
							tmr = SD_cmd(CMD1, 0x40000000); 

						}
												
						SPI_Release();
						Init = 1;		
						return 0;
																										
				    }	
			}
				
		}	
			timer++;
			if(timer > 5)
				break;
	}
	
	return 1;
}

//-----------------------------------------------

uint8_t SPI_wait_ready(void)

{

  uint8_t res;
  uint16_t cnt;
  cnt=0;

  do { 

    res=SPI_ReceiveByte();
    cnt++;

  } while ( (res!=0xFF)&&(cnt<0xFFFF) );

  if (cnt>=0xFFFF) return 1;

  return res;

}




/* END -----------------------------------------------*/
