/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int triangle_index;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	SystemClock_Config();
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// Initalize all GPIO leds (output-moder 01)
	GPIOC->MODER |=  ((1<<12)|(1<<14)|(1<<16)|(1<<18));
	GPIOC->MODER &=~ ((1<<13)|(1<<15)|(1<<17)|(1<<19));
	// initalize ADC
	RCC->APB2ENR |= (1<<9);
	// set resolution to 8 bits, bits 4,3: 10
	// set continous conversion mode, bit 13 enable
	// disable hardware trigger, bits 10 and 11
	ADC1->CFGR1 |=  ((1<<4)|(1<<13));
	ADC1->CFGR1 &=~ ((1<<3)|(1<<10)|(1<<11));
	// select/enable the input pin channel, using PC0
	GPIOC->MODER |= ((1<<0)|(1<<1));
	// self calibration, enable, and start ADC
	
	if ((ADC1->CR & ADC_CR_ADEN) != 0){ // Ensure that ADEN = 0
			ADC1->CR |= ADC_CR_ADDIS; // Clear ADEN by setting ADDIS
		}
	while ((ADC1->CR & ADC_CR_ADEN) != 0){
		}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; // Clear DMAEN
	ADC1->CR |= ADC_CR_ADCAL; // Launch the calibration by setting ADCAL
	while ((ADC1->CR & ADC_CR_ADCAL) != 0){ // Wait until ADCAL=0 
		}
	ADC1->CHSELR |= (1<<10);
	ADC1->CR |= ADC_CR_ADEN; // Enable the ADC
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) // Ensure that ADRDY = 0
		{
	ADC1->ISR |= (1<<0); // Clear ADRDY
		}
	// enable dac
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;	
	// pick a dac output, PA4, DACount_1
	GPIOA->MODER |= ((1<<8)|(1<<9));
	 
	// software trigger
	DAC1->CR |= ((1<<3)|(1<<4)|(1<<5));
	// enable 
	DAC1->CR |= (1<<2);
	// actual enable
	DAC1->CR |= (1<<0);
		
	// Triangle Wave: 8-bit, 32 samples/cycle
	const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
	190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
	
	triangle_index = 0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* // Check off 1
		ADC1->CR = ADC_CR_ADSTART;
		while (!(ADC1->ISR & ADC_ISR_EOC)){
		}
    if (ADC1->DR == 0){
			GPIOC->ODR &=~ ((1<<6)|(1<<7)|(1<<8)|(1<<9));
		}
		else if (ADC1->DR > 0 && ADC1->DR < 75){
			GPIOC->ODR |= ((1<<6));
			GPIOC->ODR &=~ ((1<<7)|(1<<8)|(1<<9));
		}
		else if (ADC1->DR > 75 && ADC1->DR < 150){
			GPIOC->ODR |= ((1<<6)|(1<<7));
			GPIOC->ODR &=~ ((1<<8)|(1<<9));
		}
		else if (ADC1->DR > 150 && ADC1->DR < 225){
			GPIOC->ODR |= ((1<<6)|(1<<7)|(1<<8));
			GPIOC->ODR &=~ ((1<<9));
		}
		else if (ADC1->DR > 225){
			GPIOC->ODR |= ((1<<6)|(1<<7)|(1<<8)|(1<<9));
		}
		*/
		// check off 2
		DAC1->DHR8R1 &=~ (0xFF);
		DAC1->DHR8R1 |= triangle_table[triangle_index];
		triangle_index = triangle_index+1;
		DAC1->SWTRIGR |= (1<<0);
		HAL_Delay(1);
		if (triangle_index == 32){
			triangle_index = 0;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
