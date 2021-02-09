/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mems.h"
#include "vector.h"
#include "usbd_cdc_if.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t button=0;
int ButtonClicks=0;
int DRDYCntr=0;
int INT1Cntr=0;
char str1[100];

HAL_StatusTypeDef status;
uint8_t res[6];

int16_t AccData[3];
int16_t AccDataF[3];
int16_t MagData[3];
int16_t GyroData[3];
int optimer = 0;

Vector3f MagR, AccR;

int kPreFAcc = 10;
int kPreFAccM = 9;
Vector3f MagShift,AccShift;
Matrix3f MagCalibr, AccCalibr;
Vector3f Mag, Acc;

Vector3f vX, vY, vZ;
Matrix3f M0, M1, M2, M3;
uint16_t Code;
Vector3f Euler;

uint8_t bufArd[30];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Mag_Ini();
  Accel_Ini();
  Gyro_Ini();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

     MagShift[0]= -20.336049f; //-24.387127f;
     MagShift[1]= -93.346292f; //-93.431390f;
     MagShift[2]= 20.061157f; //19.604326f;

     MagCalibr[0][0]=0.985646f; //0.993762f;
     MagCalibr[0][1]=0.009129f; //0.009329f;
     MagCalibr[0][2]=0.012333f; //0.013027f;

     MagCalibr[1][0]=0.009129f; //0.009329f;
     MagCalibr[1][1]=1.010575f; //1.007857;
     MagCalibr[1][2]=-0.000815f; //-0.000663f;

     MagCalibr[2][0]=0.012333f; //0.013027f;
     MagCalibr[2][1]=-0.000815f; //-0.000663f;
     MagCalibr[2][2]=1.135197f; //1.132632f;


     AccShift[0]=-101.668956f;//-106.112709f;//-124.290925f;
     AccShift[1]=211.548766f;//184.615032f;//320.388665f;
     AccShift[2]=-390.748024f;//-384.216527f;//-425.187116f;

     AccCalibr[0][0]=1.020583f;//1.002710f;//1.000517f;
     AccCalibr[0][1]=-0.000664f;//0.001763f;//-0.001543f;
     AccCalibr[0][2]=0.000368f;//-0.000784f;//0.000143f;

     AccCalibr[1][0]=-0.000664f;//0.001763f;//-0.001543f;
     AccCalibr[1][1]=0.987645f;//0.970568f;//0.974505f;
     AccCalibr[1][2]=-0.002446f;//-0.000693f;//0.001479f;

     AccCalibr[2][0]=0.000368f;//-0.000784f;//0.000143f;
     AccCalibr[2][1]=-0.002446f;//-0.000693f;//0.001479f;
     AccCalibr[2][2]=0.994811f;//0.980695f;//0.978986f;


//  Gyro_GetXYZ(GyroData);


  while (1)
  {

	    Accel_GetXYZ(AccData);
	    AccDataF[0]=(AccDataF[0]/kPreFAcc)*kPreFAccM + (AccData[0]/kPreFAcc);
	    AccDataF[1]=(AccDataF[1]/kPreFAcc)*kPreFAccM + (AccData[1]/kPreFAcc);
	    AccDataF[2]=(AccDataF[2]/kPreFAcc)*kPreFAccM + (AccData[2]/kPreFAcc);
	    Mag_GetXYZ(MagData);

	  MagR[0]=(float)MagData[0]-MagShift[0];
	  MagR[1]=(float)MagData[1]-MagShift[1];
	  MagR[2]=(float)MagData[2]-MagShift[2];
	  V3fTransform(MagR, MagCalibr, Mag);

  	  AccR[0]=(float)AccDataF[0]-AccShift[0];
	  AccR[1]=(float)AccDataF[1]-AccShift[1];
	  AccR[2]=(float)AccDataF[2]-AccShift[2];
	  V3fTransform(AccR, AccCalibr, Acc);

	  V3fReject(Mag, Acc, vX);
	  V3fNormalizeSelf(vX);
	  V3fNormalize(Acc, vZ);

	  V3fCross(vZ,vX,vY);
	  M3fSetRow(M1,vX,0);
	  M3fSetRow(M1,vY,1);
	  M3fSetRow(M1,vZ,2);

//	  Code++;
//	  if (Code>999)
//		  Code=0;

	  uint8_t ttt= M3fInvert(M1, M3);

	  M3fMultiply(M3, M0, M2);
	  M3fGetEuler(M2, Euler);
	  V3fMult(Euler, 10);
//	  V3fMult(Euler, 1000);

//	  sprintf(str1, "%08d %08d %08d %08d %08d %08d %08d %08d %08d\r\n", (int16_t)(AccDataF[0]), (int16_t)AccDataF[1], (int16_t)AccDataF[2], GyroData[0], GyroData[1], GyroData[2], MagData[0], MagData[1], MagData[2]);
//	  if (Code%300 == 0)
//	  	CDC_Transmit_FS((unsigned char*)str1, strlen(str1));


	  optimer++;
	  if (optimer>10000) {
	  	optimer = 0;
	  }
	  if (optimer%3==0) {
	  	Code++;
	  	if (Code>999)
	  		Code=0;

		bufArd[0]=(uint8_t)(0xAA);
		bufArd[1]=(uint8_t)(0xAA);
		bufArd[2]=(uint8_t)Code;
		bufArd[3]=(uint8_t)(Code>>8);

		bufArd[4]=(uint8_t)0x00;
		bufArd[5]=(uint8_t)0x00;
		bufArd[6]=(uint8_t)0x00;
		bufArd[7]=(uint8_t)0x00;

		bufArd[8]=(uint8_t)0x00;
		bufArd[9]=(uint8_t)0x00;
		bufArd[10]=(uint8_t)0x00;
		bufArd[11]=(uint8_t)0x00;

		bufArd[12]=(uint8_t)0x00;
		bufArd[13]=(uint8_t)0x00;
		bufArd[14]=(uint8_t)0x00;
		bufArd[15]=(uint8_t)0x00;

		bufArd[16]=(uint8_t) 0x00;
		bufArd[17]=(uint8_t) 0x00;
		bufArd[18]=(uint8_t) 0x00;
		bufArd[19]=(uint8_t) 0x00;

		bufArd[20]=(uint8_t) 0x00;
		bufArd[21]=(uint8_t) 0x00;
		bufArd[22]=(uint8_t) 0x00;
		bufArd[23]=(uint8_t) 0x00;

		bufArd[24]=(uint8_t) 0x00;
		bufArd[25]=(uint8_t) 0x00;
		bufArd[26]=(uint8_t) 0x00;
		bufArd[27]=(uint8_t) 0x00;

		bufArd[28]=(uint8_t)(0x55);
		bufArd[29]=(uint8_t)(0x55);

		floatToByteArray(Euler[0],bufArd,4);
		floatToByteArray(Euler[1],bufArd,8);
		floatToByteArray(Euler[2],bufArd,12);

		CDC_Transmit_FS(bufArd,30);
	  }
//		HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE9 PE10 PE11 
                           PE12 PE13 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==GPIO_PIN_0)
	{
		button=!button;
		M3fDupe(M1, M0);
	}
	else if (GPIO_Pin==GPIO_PIN_1)
	{
		Gyro_GetXYZ(GyroData);
	}
	else if (GPIO_Pin==GPIO_PIN_2)
	{
		Mag_GetXYZ(MagData);
	}
	else if (GPIO_Pin==GPIO_PIN_4)
	{
		Accel_GetXYZ(AccData);
		AccDataF[0]=(AccDataF[0]/kPreFAcc)*kPreFAccM + (AccData[0]/kPreFAcc);
		AccDataF[1]=(AccDataF[1]/kPreFAcc)*kPreFAccM + (AccData[1]/kPreFAcc);
		AccDataF[2]=(AccDataF[2]/kPreFAcc)*kPreFAccM + (AccData[2]/kPreFAcc);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
