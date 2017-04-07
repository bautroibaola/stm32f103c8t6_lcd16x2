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
#include "stm32f1xx_hal.h"

//Hung code
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define RS_Port GPIOA
#define EN_Port GPIOA
#define D4_Port GPIOA
#define D5_Port GPIOA
#define D6_Port GPIOA
#define D7_Port GPIOA

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);

//Hung code
void LCD_Enable(void)
{
HAL_GPIO_WritePin(EN_Port,E_Pin, GPIO_PIN_SET);
HAL_Delay(1);
HAL_GPIO_WritePin(EN_Port,E_Pin,GPIO_PIN_RESET);	
HAL_Delay(1);	
}

void LCD_Send4Bit(unsigned char Data) //org
//void LCD_Send4Bit(uint8_t Data) //Hung
{
HAL_GPIO_WritePin(D4_Port,D4_Pin,(Data>>0)&0x01);
HAL_GPIO_WritePin(D5_Port,D5_Pin,(Data>>1)&0x01);
HAL_GPIO_WritePin(D6_Port,D6_Pin,(Data>>2)&0x01);
HAL_GPIO_WritePin(D7_Port,D7_Pin,(Data>>3)&0x01);	
}

void LCD_SendCommand(unsigned char command) //org
//void LCD_SendCommand(uint8_t command) //Hung
{
	LCD_Send4Bit(command >>4);
	LCD_Enable();
	LCD_Send4Bit(command);
	LCD_Enable();
}

void LCD_Clear(void)
{
 	LCD_SendCommand(0x01);  
  HAL_Delay(5);	
}

void LCD_Init(void)
{
	LCD_Send4Bit(0x00);
	HAL_Delay(20); //Hung added
  HAL_GPIO_WritePin(RS_Port,RS_Pin,0);

	LCD_Send4Bit(0x03);
	LCD_Enable();
	HAL_Delay(5); //Hung added
	LCD_Enable();
	HAL_Delay(100); //Hung added
	LCD_Enable();
	LCD_Send4Bit(0x02);
	LCD_Enable();
	LCD_SendCommand(0x28);
	LCD_SendCommand(0x0C);
	LCD_SendCommand(0x06);
	LCD_SendCommand(0x01);
	
	//LCD_Clear(); //Hung added
}

	void LCD_Gotoxy(unsigned char x, unsigned char y) //Old
	//void LCD_Gotoxy(uint8_t x, uint8_t y)
	{
		unsigned char address; //org
		//uint8_t address; //Hung added
		if(y==0) {
			address=0x80; //org
			//address = (0x80+x); //Hung added
		}
		else if(y==1){
			address=0xc0;
		} 
		else if(y==2){
			address=0x94;
		}
		else {
			address=0xd4;
		}
		
		address+=x;
		LCD_SendCommand(address);
		HAL_Delay(5); //Hung added
	}

void LCD_PutChar(unsigned char Data) //org
//void LCD_PutChar(uint8_t Data) //Hung
{
  HAL_GPIO_WritePin(RS_Port,RS_Pin,1);
 	LCD_SendCommand(Data);
  HAL_GPIO_WritePin(RS_Port,RS_Pin,0);
}

void LCD_Puts(char *s) //org
//void LCD_Puts(uint8_t *s) //Hung
{
   	while (*s){
      	LCD_PutChar(*s);
     	s++;
   	}
}
//End Hung code

int main(void)
{
	
	
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

	//Hung
	LCD_Init();
	LCD_Gotoxy(0,0);
	LCD_Puts("Hello Hung");
	LCD_Gotoxy(0,1);
	LCD_Puts("Tessssst OK");
	
  while (1)
  {
		//LCD_Gotoxy(0,0);
		//LCD_Puts("TEst");
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(1000);

  }

}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS_Pin|E_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin E_Pin D4_Pin D5_Pin 
                           D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = RS_Pin|E_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
