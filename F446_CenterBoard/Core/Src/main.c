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
#include "dac.h"
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

uint8_t Craction_in,Break_in,Flont_Lamp_in,Key_in,EX_in,Winker_L_in,Winker_R_in;
uint8_t state,cnt,connect,EN,EN_rear,EN_front,mode;
float slot,speed,slot_data;
float rol,pit,pit_acc,rol_acc,pit_temp,rol_temp;

float map(float x, float in_min, float in_max, float out_min, float out_max) {
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  //MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  EN_rear=0;
  EN_front=0;
  setbuf(stdout, NULL);
  can_init(&hcan1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  HAL_UART_Init(&huart2);
  HAL_UART_Receive_DMA(&huart2,(uint8_t *)Rxbuf_from_ESP32,Rxbufsize_from_ESP32);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
  mpu9250_init(&hspi2);

  while(1){
      	if(EN_front==1 && EN_rear==1){break;}
      	if(HAL_GetTick()>1000){
      		Error_Handler();
      	}
      }

  connect=0;
  DFPlayer_init(&huart3);
  HAL_Delay(500);
  DFPlayer_setvolume(0x40);
  HAL_Delay(500);
  DFPlayer_playmp3(2);
  HAL_Delay(3000);

  MX_IWDG_Init();
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  printf(" slot=%f",slot);
	  printf(" slot_out=%f",slot_data);

	  printf(" state=%d",state);
	  printf(" speed=%f",speed);
	  //printf(" C=%d",Craction_in);
	  //printf(" F=%d",Flont_Lamp_in);
	  //printf(" B=%d",Break_in);
	  //printf(" L=%d",Winker_L_in);
	  //printf(" R=%d",Winker_R_in);
	  //printf(" K=%d",Key_in);
	  //printf(" E=%d",EX_in);
	  /*printf(" 0=%d",Rxbuf_from_ESP32[0]);
	  printf(" 1=%d",Rxbuf_from_ESP32[1]);
	  printf(" 2=%d",Rxbuf_from_ESP32[2]);
	  printf(" 3=%d",Rxbuf_from_ESP32[3]);
	  printf(" 4=%d",Rxbuf_from_ESP32[4]);*/
	  printf(" 0=%d",data_from_esp32[0]);
	  printf(" 1=%d",data_from_esp32[1]);
	  printf(" 2=%d",data_from_esp32[2]);
	  //printf(" 3=%d",data_from_esp32[3]);
	  //printf(" 3=%d",data_from_esp32[3]);
	  //printf(" Yaw=%f",gyro_data[0]);
	  //printf(" Rol=%f",gyro_data[1]);
	  //printf(" Pit=%f",gyro_data[2]);
	  //printf(" X=%f",accel_data[0]);
	  //printf(" Y=%f",accel_data[1]);
	  //printf(" Z=%f",accel_data[2]);
	  //printf(" mX=%f",mag_data[0]);
	  //printf(" mY=%f",mag_data[1]);
	  //printf(" mZ=%f",mag_data[2]);
	  //printf(" mpu.temp=%f",temperature);
	 /* printf(" Rol_acc=%7.4f",rol_acc);
	  printf(" Pit_acc=%7.4f",pit_acc);
	  printf(" Rol=%7.4f",rol);
	  printf(" Pit=%7.4f",pit);*/

	  printf("\r\n");

	  HAL_IWDG_Refresh(&hiwdg);

	  //HAL_Delay(10);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	  	{
	    		Error_Handler();
	  	}
	switch (RxHeader.StdId){
	//error
	case 0x000:
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
		break;

	//state button
	case 0x100:
		state=RxData[0];
		Winker_L_in=  (state & 0b01000000)>>6;
		Winker_R_in=  (state & 0b00100000)>>5;
		Craction_in=  (state & 0b00010000)>>4;
		Break_in=     (state & 0b00001000)>>3;
		Flont_Lamp_in=(state & 0b00000100)>>2;
		Key_in=       (state & 0b00000010)>>1;
		EX_in=        (state & 0b00000001);

		EN_front=1;
		break;
	//slot
	case 0x101:
		slot=uchar4_to_float(RxData);
		break;
	//speed
	case 0x102:
		speed=uchar4_to_float(RxData);
		EN_rear=1;
		break;

	}
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//mode=data_from_esp32[0];
	mode=4;

	if(mode==1){
		slot_data=map(slot,0.650,3.265,0.9,3.5);//norm}
	}
	else if(mode==2){
		slot_data=map(slot,0.650,3.265,0.9,2.5);//eco
	}
	else if(mode==3){
		slot_data=powf(slot,3)*0.3713+powf(slot,2)*(-1.8438)+slot*(3.1176)-0.4461;//snow
	}
	else if(mode==4){
		slot_data=powf(slot,4)*(0.3067)+powf(slot,3)*(-2.4464)+powf(slot,2)*(6.3486)+slot*(-4.8485)+1.9619;//sport
	}
	else if(mode==5){
		slot_data=powf(slot,3)*0.1732+powf(slot,2)*(-1.6682)+slot*(5.2727)-1.9883;//sport plus
	}
	else if(mode==10){
		slot_data=powf(slot,5)*(0.0532)+powf(slot,4)*(-0.839)+powf(slot,3)*(4.9461)+powf(slot,2)*(-13.884)+slot*(18.813)-6.4772;//extreme
	}
	else{slot_data=map(slot,0.650,3.265,0.9,2.5);//eco
	}


	if(slot_data>3.5){slot_data=3.5;}
	if(slot_data<0.9){slot_data=0.9;}

	float setvalue1=(4095.0/9.9)*slot_data;
	float setvalue2=(4095.0/9.9)*slot_data;

	//if(EN==1){
		if(mode==4){//sport
			if(speed==0.0) {
				setvalue1=(4095.0/9.9)*slot_data;
				setvalue2=(4095.0/9.9)*slot_data*0.6;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
			}
			else if(speed==0.01) {
				setvalue1=(4095.0/9.9)*slot_data;
				setvalue2=(4095.0/9.9)*slot_data*0.7;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
			}
			else if(speed==0.02) {
				setvalue1=(4095.0/9.9)*slot_data;
				setvalue2=(4095.0/9.9)*slot_data*0.8;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
			}
			else if(speed==0.03) {
				setvalue1=(4095.0/9.9)*slot_data;
				setvalue2=(4095.0/9.9)*slot_data*0.9;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
			}
			else {
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
			}
		}

		else if(mode==5){//sport plus
			if(speed==0.0) {
				setvalue1=(4095.0/9.9)*slot_data;
				setvalue2=(4095.0/9.9)*slot_data*0.7;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
			}
			else if(speed==0.01) {
				setvalue1=(4095.0/9.9)*slot_data;
				setvalue2=(4095.0/9.9)*slot_data*0.8;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
			}
			else if(speed==0.02) {
				setvalue1=(4095.0/9.9)*slot_data;
				setvalue2=(4095.0/9.9)*slot_data*0.9;
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
			}
			else {
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
			}
		}
		else if(mode==10){//extreme
					if(speed==0.0) {
						setvalue1=(4095.0/9.9)*slot_data*0.8;
						setvalue2=(4095.0/9.9)*slot_data;
						HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
						HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
					}
					else if(speed==0.01) {
						setvalue1=(4095.0/9.9)*slot_data*0.9;
						setvalue2=(4095.0/9.9)*slot_data;
						HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
						HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
					}
					else {
						HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
						HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
					}
				}

		else{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (int)setvalue1);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (int)setvalue2);
		}
	//}
	//else{
	//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	//}

	uint8_t senddata[5];
	senddata[0]=254;
	senddata[1]=state;
	senddata[2]=0;
	senddata[3]=cnt;
	senddata[4]=253;

	HAL_UART_Transmit(&huart2,(uint8_t*)senddata ,5, 0x0f);

	if(cnt>250){
		cnt=0;
		if(EN_front==0){
			Error_Handler();
		}
		else{
			EN_front=0;
		}
		if(EN_rear==0){
			Error_Handler();
		}
		else{
			EN_rear=0;
		}
	}
	else{
		cnt++;
	}

	/*mpu9250_read_gyro();
	mpu9250_read_acc();

	pit_acc=myAtan2(accel_data[0], sqrt(accel_data[1] * accel_data[1]+accel_data[2] * accel_data[2]))*180.0/ M_PI;
	rol_acc=myAtan2(accel_data[1], accel_data[2]) *180.0/ M_PI;
	pit=0.95*(gyro_data[0]*0.002+pit_temp)+0.05*pit_acc;
	rol=0.95*(gyro_data[1]*0.002+rol_temp)+0.05*rol_acc;
	pit_temp=pit;
	rol_temp=rol;*/

	if(data_from_esp32[1]>0){
		if(connect==0){
			if(data_from_esp32[1]==1){
				DFPlayer_playmp3(6);
				EN=1;
			}
			else{
				EN=0;
				DFPlayer_playmp3(1);
			}
		}
		connect=1;
	}
	else{
		if(connect==1){
			DFPlayer_playmp3(3);
		}
		connect=0;
		EN=0;
	}


}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uint8_t j = 0;

	while (Rxbuf_from_ESP32[j] != 254 &&  j<sizeof(Rxbuf_from_ESP32)) {
		j++;
	}
	if(j>=sizeof(Rxbuf_from_ESP32)){
		for(uint8_t k=0;k<(sizeof(data_from_esp32));k++){
			data_from_esp32[k]=0;
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,1);
	}
	else{
		for (uint8_t k = 0; k < sizeof(data_from_esp32); k++) {
			if ((j + k) >= sizeof(data_from_esp32)) {
				data_from_esp32[k] = Rxbuf_from_ESP32[k - (sizeof(data_from_esp32) - j)];
			}
			else {
				data_from_esp32[k] = Rxbuf_from_ESP32[j + k + 1];
			}
		}
	}
	if(data_from_esp32[sizeof(data_from_esp32)-1]==253){
		for(uint8_t k=0;k<sizeof(data_from_esp32);k++){
			Rxbuf_from_ESP32_temp[k]=data_from_esp32[k];
		}
	}
	else{
		for(uint8_t k=0;k<sizeof(data_from_esp32);k++){
			data_from_esp32[k]=Rxbuf_from_ESP32_temp[k];
		}
	}


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
	TxData[0]=0x02;
	TxData[1]=0xFF;
	can_send(0x000,(uint8_t*)TxData);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
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
