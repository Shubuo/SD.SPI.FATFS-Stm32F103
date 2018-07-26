
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */

#define CMD0 (0x40+0) // GO_IDLE_STATE
#define CMD1 (0x40+1) // SEND_OP_COND (MMC)
#define ACMD41 (0xC0+41) // SEND_OP_COND (SDC)
#define CMD8 (0x40+8) // SEND_IF_COND
#define CMD9 (0x40+9) // SEND_CSD
#define CMD16 (0x40+16) // SET_BLOCKLEN
#define CMD17 (0x40+17) // READ_SINGLE_BLOCK
#define CMD24 (0x40+24) // WRITE_BLOCK
#define CMD55 (0x40+55) // APP_CMD
#define CMD58 (0x40+58) // READ_OCR

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "SD.h"
#include "LCD162A.h"
#include "ff_gen_drv.h"
#include "user_diskio.h"



#define CS_SD_GPIO_PORT 	GPIOA
#define CS_SD_PIN 				GPIO_PIN_3
#define CS_SD_LOW() 			HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_RESET)
#define CS_SD_HIGH() 			HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_SET)

#define HX_SCK_GPIO_PORT 	GPIOC
#define HX_SCK_PIN 				GPIO_PIN_14
#define HX_SCK_LOW() 			HAL_GPIO_WritePin(HX_SCK_GPIO_PORT, HX_SCK_PIN, GPIO_PIN_RESET)
#define HX_SCK_HIGH()			HAL_GPIO_WritePin(HX_SCK_GPIO_PORT, HX_SCK_PIN, GPIO_PIN_SET)

#define HX_DT_GPIO_PORT 	GPIOC
#define HX_DT_PIN 				GPIO_PIN_15
#define HX_DT_LOW() 			HAL_GPIO_WritePin(HX_DT_GPIO_PORT, HX_DT_PIN, GPIO_PIN_RESET)
#define HX_DT_HIGH() 			HAL_GPIO_WritePin(HX_DT_GPIO_PORT, HX_DT_PIN, GPIO_PIN_SET)
#define HX_DT_Read() 			HAL_GPIO_ReadPin(HX_DT_GPIO_PORT, HX_DT_PIN) 

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

RTC_DateTypeDef sDate;
RTC_TimeTypeDef sTime;

extern int 	i;
extern int  tmr;
char 				USER_Path[4]; /* logical drive path */
extern char st[60];

uint8_t 		sect[512];	
uint8_t 		Time[3];	
uint8_t			estado = 0;
uint8_t			teste = 0;
uint8_t 		ocr2[5];
uint8_t 		result;

// FIR FILTER:
//11 Hamming Window
//double			FIR_C[] = {0.0145489741947061	,0.0305712499252980	,0.0725451577608892	,0.124486576686153	,0.166541934360414	,0.182612214145080,	0.166541934360414,	0.124486576686153,	0.0725451577608892,	0.0305712499252980	,0.0145489741947061	};
//16 Triangular
double			FIR_C[] = {0.0610597081690606,	0.0616322647111381	,0.0621255648055849	,0.0625384434486763,	0.0628699247468551	,0.0631192246649620,	0.0632857532350082	,0.0633691162187148,	0.0633691162187148	,0.0632857532350082,	0.0631192246649620	,0.0628699247468551,	0.0625384434486763,	0.0621255648055849	,0.0616322647111381	,0.0610597081690606};



// IIR FILTER:
double 			sumA = 0;
double 			sumB = 0;
double 			IIR_B[2] ={0.237375515512861,	0.237375515512861};	
double 			IIR_A[2] ={1,	-0.52524896};	

//double 			IIR_B[3] ={0.000238315522932320	,0.000476631045864640,	0.000238315522932320};
//double 			IIR_A[3] ={1, -1.96503497983460,	0.966104557493302};	

//double 			IIR_B[5] ={1.47345880526560e-08,	5.89383522106242e-08,	8.84075283159363e-08	,5.89383522106242e-08	,1.47345880526560e-08};
//double 			IIR_A[5] ={1	,-3.96908538561744,	5.90870411266836	,-3.91012885563580	,0.970510393104559};	


double 			y[4] ={0};	


char 				taux1[10];
char 				taux[3];
FATFS 			SDFatFs;
FATFS 			*fs;
FIL 				MyFile;
FRESULT     res; 

signed int	horas 		= 12;
signed int	hora 		  = 12;
signed int	minutos 	= 30;
signed int	segundos 	= 50;
signed int	dia 			= 15;
signed int	mes 			= 06;
signed int  ano 			= 18;

unsigned long	sensor[20] = {0};


long auxTara 		= 0;
long auxTara2 	= 0;
float Tara 			= 0;

float aForce		= 0;
float Force 		= 0;
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* FRESULT open_append ----------------------------------------------------------*/

	FRESULT open_append (
    FIL* fp,            /* [OUT] File object to create */
    const char* path    /* [IN]  File name to be opened */
	)
	{
    FRESULT fr;
		FRESULT aux;

    /* Opens an existing file. If not exist, creates a new file. */
		
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        aux = f_lseek(fp, f_size(fp));
        if (aux != FR_OK)
            f_close(fp);
    }
    return fr;
}
	

void Update_RTC(void){
	
	HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
	minutos = sTime.Minutes;	
	horas = sTime.Hours;
	dia = sDate.Date;
	mes = sDate.Month;
	ano =	sDate.Year;
}

void LCD_ShowRTC(void){
	
	sprintf(taux1,"%02d/%02d/%02d",dia,mes,ano);
	LCD_CURSOR(1,8);	
  ENVIA_STRING_LCD(taux1);
	LCD_CURSOR(0,11);	
	sprintf(taux,"%02d:%02d",hora,minutos);
	ENVIA_STRING_LCD(taux);
	LCD_CURSOR(1,20);

}

void SD_Backup(unsigned long dataForce){
	
	//  Inicializacao uSD	
			
	disk_initialize(SDFatFs.drv);
					
				if(SD_Init() != 0){
					
					LCD_LIMPA();
					ENVIA_STRING_LCD("NO SD");
					HAL_Delay(500);
				}
				else{					
					if(f_mount(&SDFatFs,(const char*)USER_Path,0)!= FR_OK){
								LCD_LIMPA();
								ENVIA_STRING_LCD("OPEN ERROR 1");
					}
					else{
								
						if(open_append(&MyFile,"MEDICAO.txt") != FR_OK){						
								LCD_LIMPA();
								ENVIA_STRING_LCD("OPEN ERROR 2");																
					}
							else{														
								Update_RTC();					
								f_printf(&MyFile, "Data: %02u/%02u/%u, %2u:%02u\n",dia,mes,ano,horas,minutos);
								f_printf(&MyFile, "FORCA: %lu \n",dataForce);			
								f_close(&MyFile);
								
								
							
								LCD_LIMPA();
								ENVIA_STRING_LCD("Saved...");
								HAL_Delay(800); 
					}					 
				}
			}							
				FATFS_UnLinkDriver(USER_Path);
	}

unsigned long ReadCount(void){
	
	unsigned long Count;
	unsigned char i;
	
	HX_DT_HIGH(); //ADDO=1;
	HX_SCK_LOW(); //ADSK=0;
	
	Count=0;
  while(HX_DT_Read()); 				//while(ADDO); 
	
	for (i=0;i<24;i++){
		HX_SCK_HIGH(); 						//ADSK=1;
		Count=Count<<1;
		HX_SCK_LOW();							//ADSK=0;
		if(HX_DT_Read()) Count++;		//if(ADDO)
	}
	
	HX_SCK_HIGH();								//ADSK=1;
	Count=Count^0x800000;
	HX_SCK_LOW();								//ADSK=0;
	return(Count);
} 

unsigned long moving_average(int M){
	
	unsigned long sum = 0;

	for(i=M;i>0;i--){
		
		sensor[i] = sensor[i-1];
	}
	
	sensor[0] = ReadCount();
	sum = 0;
	
	for(i=0;i<M;i++){
		
		sum = sum + sensor[i];	
	}
	
	return sum/M;

}


double FIR(int M){
	
	unsigned long sum = 0;

	for(i=M;i>0;i--){
		
		sensor[i] = sensor[i-1];
	}
	
	sensor[0] = ReadCount();
	sum = 0;
	
	for(i=0;i<M;i++){
		
		sum = sum + sensor[i]*FIR_C[i];	
	}
	
	return sum;

}

double IIR(int M){
	
	
	for(i=M;i>0;i--){
		
		sensor[i] = ReadCount();
	}
	
	sensor[0] = ReadCount();
	
	sumA = 0;
	sumB = 0;
	
	for(i=0;i<M;i++){
		
		sumB = sumB + sensor[i]*IIR_B[i];	
		
	}
	
	for(i=0;i<(M+1);i++){
	
		sumA = sumA + y[i]*IIR_A[i+1];
	}
			
	for(i=0;i<M;i++){
	
		y[i+1] = y[i];
	}	
	
	y[0] = sumB - sumA;
	
	return (sumB - sumA);
	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
	
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_INICIALIZA();
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
	while (1){
		
		switch(estado){
			
			case 0:
			{				
				if(teste != 1){
					LCD_LIMPA();
					ENVIA_STRING_LCD("MENU");
					teste = 1;	
				}
				
				Update_RTC();
				LCD_ShowRTC();
								
				if(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin)){
					while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin));					
					estado = 1;
					teste = 0;
				}	       
			break; 
			}
			
			case 1:
			{			

				Update_RTC();
				LCD_ShowRTC();			
				
				if(teste != 1){
					LCD_LIMPA();
					ENVIA_STRING_LCD("TARA");
					teste = 1;
				}
				
				if(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin)){
					while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin));
					LCD_LIMPA();
					ENVIA_STRING_LCD("OBTENDO TARA");
					
					auxTara=0;
					auxTara2=0;
					
					for(i=0;i<20;i++){
						auxTara = ReadCount();
						auxTara2 += auxTara;					
					}
					
					Tara = auxTara2/20;
					
					
					sprintf(taux1,"Tara: %f",Tara);
					LCD_LIMPA();					
					LCD_CURSOR(0,0);
					ENVIA_STRING_LCD(taux1);					
					HAL_Delay(1500);
										
					teste = 0;
				}
								
				if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)){
					while(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin));
					estado = 2;
					teste = 0;
				}
				if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)){
					while(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin));
					estado = 3;
					teste = 0;
				}       
			break; 
			}
			
			case 2:
			{		

				Update_RTC();
				LCD_ShowRTC();			
				
				if(teste != 1){
					LCD_LIMPA();
					ENVIA_STRING_LCD("INICIAR");
					LCD_CURSOR(1,20);
					teste = 1;
					
				}
				
				if(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin)){
					while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin));
					LCD_LIMPA();
					
								
								
					while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin) == 0){
					
						//aForce = moving_average(100);
						//aForce = ReadCount();					
					
											
						//aForce = FIR(16);	
						aForce =	IIR(2);						
						Force = (aForce - Tara)*(0.00238);
						
						
						LCD_CURSOR(0,0);
						//LCD_LIMPA();
						sprintf(taux1,"Forca: %.02f",Force);						
						ENVIA_STRING_LCD(taux1);	
																	
					}while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin));
					
					teste = 0;
					
										

         //  Inicializacao uSD				
					
//				if(SD_Init() != 0){					
//					LCD_LIMPA();
//					ENVIA_STRING_LCD("NO SD");
//					HAL_Delay(500);		
//					teste=0;
//				}
//				else{					
//					SD_Backup((Force));					
//				}						
				}
								
				if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)){
					while(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin));
					estado = 1;
					teste = 0;
				}       
				else if (HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)){
					while(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin));
					estado = 3;
					teste = 0;				
				}
			break; 
			}
			case 3:
			{				
				if(teste != 1){
					LCD_LIMPA();
					ENVIA_STRING_LCD("DATA/HORA");
					teste = 1;
				}
					
				Update_RTC();
				LCD_ShowRTC();				
				
				if(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin)){
					while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin));
					LCD_LIMPA();
					ENVIA_STRING_LCD("SELECIONE:");
					while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin));
								
					do{									
									
						sprintf(taux1,"%02d:%02d %02d/%02d/%02d",horas,minutos,dia,mes,ano);
						LCD_CURSOR(1,2);	
						ENVIA_STRING_LCD(taux1);
						LCD_CURSOR(1,20);						
						
						if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)){
							horas += 1;							
							if(horas > 23){
								horas = 0;
							}
							sTime.Hours = horas;
							HAL_Delay(50);							
																												
						}
						
						if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)){
							horas -= 1;							
							if(horas< 0){
								horas = 23;
							}
							sTime.Hours = horas;
							HAL_Delay(50);
						}						
						
					}while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin) == 0);
					while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin));
					
					HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
					HAL_Delay(50);
					
					do{			
						
						sprintf(taux1,"%02d:%02d %02d/%02d/%02d",horas,minutos,dia,mes,ano);
						LCD_CURSOR(1,2);	
						ENVIA_STRING_LCD(taux1);
						LCD_CURSOR(1,20);						
						
						if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)){
							minutos += 1;
								if(minutos > 59){
									minutos = 0;
								}							
								 
							HAL_Delay(50);
																					
						}
						
						if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)){
							minutos -= 1;
								if(minutos < 0){
									minutos = 59;
								}							
							HAL_Delay(50);
						}						
						
						
					}while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin) == 0);
					while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin));
					
					sTime.Minutes = minutos;
					HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
					
					do{			
						
						sprintf(taux1,"%02d:%02d %02d/%02d/%02d",horas,minutos,dia,mes,ano);
						LCD_CURSOR(1,2);	
						ENVIA_STRING_LCD(taux1);
						LCD_CURSOR(1,20);						
						
						if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)){
							dia += 1;
							if(dia > 31){
								dia = 1;
							}							
							sDate.Date = dia;	 
							HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
							HAL_Delay(50);
																					
						}
						
						if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)){
								dia -= 1;
								if(dia < 1){
									dia = 31;
								}							
							sDate.Date = dia;	 
							HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
							HAL_Delay(50);
						}							
						
					}while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin) == 0);
					while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin));
					
					do{			
						
						sprintf(taux1,"%02d:%02d %02d/%02d/%02d",horas,minutos,dia,mes,ano);
						LCD_CURSOR(1,2);	
						ENVIA_STRING_LCD(taux1);
						LCD_CURSOR(1,20);						
						
						if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)){
							mes += 1;
							if(mes > 12){
								mes = 0;
							}							
							sDate.Month = mes;	 
							HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
							HAL_Delay(50);
																					
						}
						
						if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)){
								mes -= 1;
								if(mes < 0){
									mes = 12;
								}							
							sDate.Month = mes;	 
							HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
							HAL_Delay(50);
						}	
						
					}while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin) == 0);
					while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin));
					// -------------------------------------------------------------------
					do{	
						hora = horas;
						sprintf(taux1,"%02d:%02d %02d/%02d/%02d",hora,minutos,dia,mes,ano);
						LCD_CURSOR(1,2);	
						ENVIA_STRING_LCD(taux1);
						LCD_CURSOR(1,20);						
						
						if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)){
							ano += 1;
							if(ano > 99){
								ano = 0;
							}							
							sDate.Year = ano;	 
							HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
							HAL_Delay(50);
																					
						}
						
						if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)){
								ano -= 1;
								if(ano < 0){
									ano = 99;
								}							
							sDate.Year = ano;	 
							HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
							HAL_Delay(50);
						}	
												
					}while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin) == 0);
					while(HAL_GPIO_ReadPin(BT3_GPIO_Port,BT3_Pin));
					
					teste =0;				
				}
					
			if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)){
				while(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin));
				estado = 2;
				teste = 0;
				
			}
			
			if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)){
				while(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin));
				estado = 1;
				teste = 0;
			}  			
				
			break; 				
					
			}								
				
			}	
		
			
			
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2){
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x18;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|HX_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|DB4_Pin|DB5_Pin|DB6_Pin 
                          |DB7_Pin|E_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BT3_Pin|BT2_Pin|BT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 HX_SCK_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|HX_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : HX_DT_Pin */
  GPIO_InitStruct.Pin = HX_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HX_DT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 DB4_Pin DB5_Pin DB6_Pin 
                           DB7_Pin E_Pin RS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|DB4_Pin|DB5_Pin|DB6_Pin 
                          |DB7_Pin|E_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BT3_Pin BT2_Pin BT1_Pin */
  GPIO_InitStruct.Pin = BT3_Pin|BT2_Pin|BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
