/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "management.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch) {
HAL_UART_Transmit(&huart1, &ch, 1, 1);
}

uint8_t Craction_in,Break_in,Flont_Lamp_in,Key_in,EX_in,Winker_L_in,Winker_R_in;
uint8_t state;
float slot;
uint8_t cnt_Winker_L,cnt_Winker_R;
float speed;
int32_t speed_count,counter;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM15_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  setbuf(stdout, NULL);
  can_init(&hcan);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_PWM_Init(&htim15);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 1001);
  HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
  TIM2->CNT=2000000;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  printf(" slot=%f",slot);
	  printf(" state=%d",state);
	  printf(" C=%d",Craction_in);
	  printf(" F=%d",Flont_Lamp_in);
	  printf(" B=%d",Break_in);
	  printf(" L=%d",Winker_L_in);
	  printf(" R=%d",Winker_R_in);
	  printf(" K=%d",Key_in);
	  printf(" E=%d",EX_in);
	  printf(" speed=%f",speed);
	  printf(" speed_count=%d",(TIM2->CNT));
	  printf("\r\n");
	  HAL_IWDG_Refresh(&hiwdg);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	  	{
	    		Error_Handler();
	  	}

	switch (RxHeader.StdId){
	//error
	case 0x000:
		NVIC_SystemReset();
		break;

	//state button
	case 0x100:
		state=RxData[0];
		break;
	//slot
	case 0x101:
		slot=uchar4_to_float(RxData);
		break;

	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(counter>10){
		speed_count= (TIM2->CNT)-2000000;
		TIM2->CNT=2000000;
		speed=(float)speed_count/100.0;
		counter=0;
	}
	counter++;

	float_to_uchar4(TxData,speed);
	can_send(0x102,(uint8_t*)TxData);


	Winker_L_in=  (state & 0b01000000)>>6;
	Winker_R_in=  (state & 0b00100000)>>5;
	Craction_in=  (state & 0b00010000)>>4;
	Break_in=     (state & 0b00001000)>>3;
	Flont_Lamp_in=(state & 0b00000100)>>2;
	Key_in=       (state & 0b00000010)>>1;
	EX_in=        (state & 0b00000001);

	if(Flont_Lamp_in==0){
		if(Break_in==1){
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
			  HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
		}
		else{
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 700);
			  HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
	}
	else{
		if(Break_in==1){
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
			HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
		}
		else{
			__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 1001);
			HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	}


	if(Winker_L_in==0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,1);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,0);
	}

	if(Winker_R_in==0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,1);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,0);
	}



	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	TxData[0]=0x03;
	TxData[1]=0xFF;
	can_send(0x000,(uint8_t*)TxData);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	HAL_Delay(100);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
