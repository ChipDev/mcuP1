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

#include "waveform_lut.h"
#include "keypad.h"
#include "dac.h"
#include "tim.h"

void SystemClock_Config(void);

void Interrupt_Config(){
	//Enable NVIC IRQ and enable IRQs
	 NVIC -> ISER[0] = (1 << TIM2_IRQn);
	 __enable_irq();
}


#ifdef DEBUG_ISR// Enable MCO, select MSI (4 MHz source)

	 RCC->CFGR = ((RCC->CFGR & ~(RCC_CFGR_MCOSEL)) | (RCC_CFGR_MCOSEL_0));
	 // Configure MCO output on PA8, output on PA1
	 RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
	 GPIOA->MODER   &= ~(GPIO_MODER_MODE8);		// alternate function mode
	 GPIOA->MODER   |=  (2 << GPIO_MODER_MODE8_Pos);
	 GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT8);		// Push-pull output
	 GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD8);		// no resistor
	 GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8);		// high speed
	 GPIOA->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);		// select MCO function

	 //push pull A1
	 GPIOA->OTYPER &= ~(GPIO_OTYPER_ODR_1);
	 //OUTPUT 01
	 GPIOA->MODER &= ~(GPIO_MODER_MODE1_Msk);
	 GPIOA->MODER |= (GPIO_MODER_MODE1_0);

	 GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED1_Msk);
#endif
#ifdef DEBUG_SQUARE

	#define WAVEFORM_OUT_GPIO GPIOA
	#define WAVEFORM_OUT_PIN (1 << 9)
	 //push pull A9
	 GPIOA->OTYPER &= ~(GPIO_OTYPER_ODR_9);
	 //OUTPUT 01
	 GPIOA->MODER &= ~(GPIO_MODER_MODE9_Msk);
	 GPIOA->MODER |= (GPIO_MODER_MODE9_0);

	 GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED9_Msk);
#endif

//SPEC: Within 60% of MIN delay
#define TIM_CYCLES_DELAY 160
//At minimum hz and 160 TIM_CYCLES_DELAY (highest resolution)
//100hz -> 0.01s per sample


//default, globals
double current_duty_cycle = 0.5;
uint8_t wavetype = SINE_WAVE;
uint16_t dac_to_write = 0;
//1 - 5 for 100 - 500hz respectively
uint8_t frequency_scaling_factor = 1;

uint16_t array_index = 0;

//defaults, main loop sets these for fast ISR.
uint32_t square_high_tim_delay = 0.5 * LOW_FREQ_TIM_CYCLES;
uint32_t square_low_tim_delay = 0.5 * LOW_FREQ_TIM_CYCLES;

const int16_t *wave_lut = sine_lut;

void TIM2_IRQHandler(void){

	#ifdef DEBUG_ISR
		//A1 on for ISR
		GPIOA->ODR |= (1 << 1);
	#endif
	//timer went off
	if(TIM2->SR & TIM_SR_CC1IF){
		if(wavetype == SQUARE_WAVE){

			if(dac_to_write) {
				//DAC was HIGH, will be LOW
				TIM2->CCR1+=square_low_tim_delay;
			}else{
				TIM2->CCR1+=square_high_tim_delay;
			}

			#ifdef DEBUG_SQUARE
				//PA2
				WAVEFORM_OUT_GPIO->ODR ^= WAVEFORM_OUT_PIN;
			#endif

			//flip dac value
			if(!dac_to_write) { dac_to_write = 3769; }
			else { dac_to_write = 0; }
			DAC_write(dac_to_write);


		}else{
			TIM2->CCR1 += TIM_CYCLES_DELAY;
			//wavetype is array-based (Sine, Saw, Tri)
			array_index += frequency_scaling_factor;
			if(array_index >= MAX_SAMPLES_PER_WAVE) array_index -= MAX_SAMPLES_PER_WAVE;
			DAC_write(wave_lut[array_index]);

		}
	}

	//Clear flags
	TIM2->SR &= ~(TIM_SR_CC1IF | TIM_SR_UIF);
	#ifdef DEBUG_ISR
		GPIOA->ODR &= ~(1 << 1);
	#endif
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  Interrupt_Config();
  setup_keypad_pins();
  DAC_init();
  TIM2_Config();


  while (1)
  {
	  uint32_t keypad = lut[get_keypad_press()]; //converts to pressed value
	  uint8_t pressed = 0;
	  if(keypad >= 1 && keypad <= 5) {
		  pressed = 1;
		  frequency_scaling_factor = keypad;
		  square_high_tim_delay = current_duty_cycle * LOW_FREQ_TIM_CYCLES / frequency_scaling_factor;
		  square_low_tim_delay = (1 - current_duty_cycle) * LOW_FREQ_TIM_CYCLES / frequency_scaling_factor;
	  }else if(keypad == 6) {
		  pressed = 1;
		  wavetype = SINE_WAVE;
		  wave_lut = sine_lut;
	  }else if(keypad == 7) {
		  pressed = 1;
		  wavetype = TRI_WAVE;
		  wave_lut = tri_lut;
	  }else if(keypad == 8) {
		  pressed = 1;
		  wavetype = SAW_WAVE;
		  wave_lut = saw_lut;
	  }else if(keypad == 9) {
		  pressed = 1;
		  wavetype = SQUARE_WAVE;
	  }else if(keypad == 13) {
		  pressed = 1;
		  if(current_duty_cycle >= 0.2) current_duty_cycle -= 0.1;
		  square_high_tim_delay = current_duty_cycle * LOW_FREQ_TIM_CYCLES / frequency_scaling_factor;
		  square_low_tim_delay = (1 - current_duty_cycle) * LOW_FREQ_TIM_CYCLES / frequency_scaling_factor;
	  }else if(keypad == 14) {
		  pressed = 1;
		  if(current_duty_cycle <= 0.8) current_duty_cycle += 0.1;
		  square_high_tim_delay = current_duty_cycle * LOW_FREQ_TIM_CYCLES / frequency_scaling_factor;
		  square_low_tim_delay = (1 - current_duty_cycle) * LOW_FREQ_TIM_CYCLES / frequency_scaling_factor;
	  }else if(keypad == 0){
		  pressed = 1;
		  current_duty_cycle = 0.5;
		  square_high_tim_delay = current_duty_cycle * LOW_FREQ_TIM_CYCLES / frequency_scaling_factor;
		  square_low_tim_delay = (1 - current_duty_cycle) * LOW_FREQ_TIM_CYCLES / frequency_scaling_factor;
	  }
	  //Don't register the same press multiple times
	  while(pressed && get_keypad_press() != KEYPAD_NONE) {}
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
