/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int8_t data = 0 ;
char datatext[32] ="";
char datatext1[32] ="";
char datatext2[32] ="";




PID_TypeDef TPIDPitch;
PID_TypeDef TPIDRoll;
PID_TypeDef TPIDYaw;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
MPU6050_t MPU6050_2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

double yawbefore, yawafter, fark;

double throttle = 280;

double anglePitch, PIDOutPitch, angleSetpointPitch;
double angleRoll, PIDOutRoll, angleSetpointRoll;
double angleYaw, PIDOutYaw, angleSetpointYaw;
double test1, test2, test3;

double motorBackRight,motorBackLeft,motorFrontRight,motorFrontLeft;
double M_B_R, M_B_L, M_F_R, M_F_L;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  while ( MPU6050_Init(&hi2c1)== 1);


  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  angleSetpointPitch = 0;
  angleSetpointRoll = 0;
  angleSetpointYaw = 0;

  // PID PİTCH
  PID(&TPIDPitch,&anglePitch,&PIDOutPitch,&angleSetpointPitch,10,0.7,5,_PID_P_ON_E,_PID_CD_DIRECT);

  PID_SetMode(&TPIDPitch, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TPIDPitch, 10);
  PID_SetOutputLimits(&TPIDPitch, 0, 100);

  // PID ROLL
  PID(&TPIDRoll,&angleRoll,&PIDOutRoll,&angleSetpointRoll,5,0.0001,30,_PID_P_ON_E,_PID_CD_DIRECT);

  PID_SetMode(&TPIDRoll, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TPIDRoll, 10);
  PID_SetOutputLimits(&TPIDRoll, 0, 20);

  // PID YAW
  PID(&TPIDYaw,&angleYaw,&PIDOutYaw,&angleSetpointYaw,1,0,0,_PID_P_ON_E,_PID_CD_DIRECT);

  PID_SetMode(&TPIDRoll, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TPIDRoll, 50);
  PID_SetOutputLimits(&TPIDRoll, 0, 0);


  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,500);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,500);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,500);
  HAL_Delay(5000);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //HAL_ADC_Start(&hadc3);
	  //HAL_ADC_PollForConversion(&hadc3, 100);
	  //potvalue = HAL_ADC_GetValue(&hadc3);
	  //HAL_ADC_Stop(&hadc3);

	  MPU6050_Read_All(&hi2c1, &MPU6050);
	  //MPU6050_Read_All(&hi2c2, &MPU6050_2);

	  anglePitch = MPU6050.KalmanAngleX;
	  angleRoll= MPU6050.KalmanAngleY;
	  angleYaw = MPU6050.KalmanAngleZ;
	  yawafter = MPU6050.Gz;

	  //test1 = MPU6050.KalmanAngleX;
	  //test2 = anglePitch - test1;


	  //if(fark<1 && fark){angleYaw = yawbefore;}


	  PID_Compute(&TPIDPitch);
	  PID_Compute(&TPIDRoll);
	  PID_Compute(&TPIDYaw);



	  motorBackRight = throttle - PIDOutPitch - PIDOutRoll - PIDOutYaw; //// (+ pıdoutmax +pıdoutmin+pıdout1max)1 -yaw 220-280  260-300
	  motorBackLeft = throttle - PIDOutPitch + PIDOutRoll + PIDOutYaw;   //// (+pıdoutmax+pıdoutmin)2 +yaw 250-310  260-300
	  motorFrontRight = throttle + PIDOutPitch - PIDOutRoll + PIDOutYaw; //// (+pıdout1max)3 +yaw 250-310 260-300
	  motorFrontLeft = throttle + PIDOutPitch + PIDOutRoll - PIDOutYaw;  ////  4 -yaw  280-340 260-300
/*
	  if (motorBackRight < 250){motorBackRight = 250};

	  if (motorBackRight > 400){motorBackRight = 400};

	  if (motorBackLeft < 250){motorBackLeft = 250};
	  if (motorBackLeft > 400){motorBackLeft = 400};

	  if (motorFrontLeft < 250){motorFrontLeft = 250};
	  if (motorFrontLeft > 400){motorFrontLeft = 400};

	  if (motorFrontRight < 250){motorFrontRight = 250};
	  if (motorFrontRight > 400){motorFrontRight = 400};
*/

	  M_B_R = map(motorBackRight, 160, 280, 550, 700);
	  M_B_L = map(motorBackLeft, 180, 300, 550, 700);
	  M_F_R = map(motorFrontRight, 260, 380, 550, 700);
	  M_F_L = map(motorFrontLeft, 280, 400, 550, 700);



	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,M_B_R);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,M_B_L);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,M_F_R);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,M_F_L);
	  HAL_Delay(10);


	  //int a = (int)angleYaw;

	  //int b = (int)angleYaw;

	  //int c = (int)motorBackRight;

	  //sprintf(datatext,"\f %d \n\r",c); //
	  //pwm1 = map((uint32_t)potvalue, 0, 4095, 250, 500);

	  //sprintf(datatext,"\f %d \n\r",potvalue);
	  //sprintf(datatext2,"\f %d \n\r",data);

	  //HAL_UART_Transmit(&huart2,(uint8_t*)datatext,strlen(datatext),10);
	  //HAL_UART_Transmit(&huart2,(uint8_t*)datatext1,strlen(datatext1),100);
	  //HAL_UART_Transmit(&huart2,(uint8_t*)datatext2,strlen(datatext2),100);



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC34|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
