#include "LCD162A.h"
#include "stm32f1xx_hal.h"

char LCD_D7;
char LCD_D6;
char LCD_D5;
char LCD_D4;



void LCD_ENVIA_NIBBLE(char nibble)
	{
	HAL_GPIO_WritePin(E_GPIO_Port,E_Pin,GPIO_PIN_RESET);
		
	LCD_D7 = (nibble >> 7) & 1;
	LCD_D6 = (nibble >> 6) & 1;
	LCD_D5 = (nibble >> 5) & 1;
	LCD_D4 = (nibble >> 4) & 1;
	
	if(LCD_D7==1)
		{
			HAL_GPIO_WritePin(DB7_GPIO_Port,DB7_Pin,GPIO_PIN_SET);
		}
	else
			{
				HAL_GPIO_WritePin(DB7_GPIO_Port,DB7_Pin,GPIO_PIN_RESET);
			}
			
	if(LCD_D6==1)
			{
				HAL_GPIO_WritePin(DB6_GPIO_Port,DB6_Pin,GPIO_PIN_SET);
			}
	else
		{
			HAL_GPIO_WritePin(DB6_GPIO_Port,DB6_Pin,GPIO_PIN_RESET);
		}
		
	if(LCD_D5==1)
		{
			HAL_GPIO_WritePin(DB5_GPIO_Port,DB5_Pin,GPIO_PIN_SET);
		}
	else
		{
			HAL_GPIO_WritePin(DB5_GPIO_Port,DB5_Pin,GPIO_PIN_RESET);
		}
	if(LCD_D4==1)
		{
			HAL_GPIO_WritePin(DB4_GPIO_Port,DB4_Pin,GPIO_PIN_SET);
		}
	else
		{
			HAL_GPIO_WritePin(DB4_GPIO_Port,DB4_Pin,GPIO_PIN_RESET);
		}
		
		
	HAL_GPIO_WritePin(E_GPIO_Port,E_Pin,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(E_GPIO_Port,E_Pin,GPIO_PIN_RESET);
		
	LCD_D7 = (nibble >> 3) & 1;
	LCD_D6 = (nibble >> 2) & 1;
	LCD_D5 = (nibble >> 1) & 1;
	LCD_D4 = (nibble >> 0) & 1;
		
	if(LCD_D7==1)
		{
			HAL_GPIO_WritePin(DB7_GPIO_Port,DB7_Pin,GPIO_PIN_SET);
		}
	else
			{
				HAL_GPIO_WritePin(DB7_GPIO_Port,DB7_Pin,GPIO_PIN_RESET);
			}
			
	if(LCD_D6==1)
			{
				HAL_GPIO_WritePin(DB6_GPIO_Port,DB6_Pin,GPIO_PIN_SET);
			}
	else
		{
			HAL_GPIO_WritePin(DB6_GPIO_Port,DB6_Pin,GPIO_PIN_RESET);
		}
		
	if(LCD_D5==1)
		{
			HAL_GPIO_WritePin(DB5_GPIO_Port,DB5_Pin,GPIO_PIN_SET);
		}
	else
		{
			HAL_GPIO_WritePin(DB5_GPIO_Port,DB5_Pin,GPIO_PIN_RESET);
		}
	if(LCD_D4==1)
		{
			HAL_GPIO_WritePin(DB4_GPIO_Port,DB4_Pin,GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(DB4_GPIO_Port,DB4_Pin,GPIO_PIN_RESET);
		}
	}

void LCD_ENVIA_COMANDO(char CMD)
	{
	HAL_GPIO_WritePin(RS_GPIO_Port,RS_Pin,GPIO_PIN_RESET);
	LCD_ENVIA_NIBBLE(CMD);
//	LCD_BUS=CMD;
	HAL_GPIO_WritePin(E_GPIO_Port,E_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(E_GPIO_Port,E_Pin,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(E_GPIO_Port,E_Pin,GPIO_PIN_RESET);		
	}
	
void LCD_ENVIA_DADO(char DADO)
	{
	HAL_GPIO_WritePin(RS_GPIO_Port,RS_Pin,GPIO_PIN_SET);
	LCD_ENVIA_NIBBLE(DADO);
//	LCD_BUS=DADOZ;
	HAL_GPIO_WritePin(E_GPIO_Port,E_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(E_GPIO_Port,E_Pin,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(E_GPIO_Port,E_Pin,GPIO_PIN_RESET);	
	}

	void LCD_LIMPA(void)
	{
	LCD_ENVIA_COMANDO(0x01);
	HAL_Delay(2);
	}
	
void LCD_INICIALIZA(void)
	{
	HAL_GPIO_WritePin(DB4_GPIO_Port,DB4_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DB5_GPIO_Port,DB5_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DB6_GPIO_Port,DB6_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DB7_GPIO_Port,DB7_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(E_GPIO_Port,E_Pin,GPIO_PIN_RESET);
	HAL_Delay(15);
	LCD_ENVIA_COMANDO(0x03);
	HAL_Delay(5);
	LCD_ENVIA_COMANDO(0x03);
	HAL_Delay(5);
	LCD_ENVIA_COMANDO(0x03);			
	HAL_Delay(5);
	LCD_ENVIA_COMANDO(0x02);			
	HAL_Delay(1);
	LCD_ENVIA_COMANDO(0x28);// condições de utilização
	HAL_Delay(1);
	LCD_ENVIA_COMANDO(0x0C);//liga display sem cursor
	LCD_LIMPA();
	HAL_Delay(1);
	LCD_ENVIA_COMANDO(0x06);//modo operação cursos para a direita
	HAL_Delay(1);
	}
void ENVIA_STRING_LCD (char *str)
	{
	//Efetua a transmissão da string para lcd
		while(*str!='\0')
		{
		LCD_ENVIA_DADO(*str);
		str++;
		}
	}
	
	void LCD_CURSOR(int LINHA, int COLUNA)
	{
	if(LINHA==0)
		{
		LCD_ENVIA_COMANDO(0x80 + COLUNA);
		}
	if(LINHA==1)
		{
		LCD_ENVIA_COMANDO(0xC0 + COLUNA);
		}
	HAL_Delay(1);
	}
	
	