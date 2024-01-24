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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GYRO_WHO_AM_I 0b11010011

#define GYRO_ADDR_OUT_X_L 0xA8
#define GYRO_ADDR_OUT_X_H 0xA9
#define GYRO_ADDR_OUT_Y_L 0xAA
#define GYRO_ADDR_OUT_Y_H 0xAB
#define GYRO_ADDR_OUT_Z_L 0xAC
#define GYRO_ADDR_OUT_Z_H 0xAD

#define ACC_I2C_ADDR 0x32
#define ACC_WHO_AM_I 0b00110011
#define ACC_ADDR_WHO_AM_I 0x0F
#define ACC_ADDR_OUT_X_L 0x28
#define ACC_ADDR_CRTL1 0x20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t gyro_adr[6] = {
		GYRO_ADDR_OUT_X_L, GYRO_ADDR_OUT_X_H,
		GYRO_ADDR_OUT_Y_L, GYRO_ADDR_OUT_Y_H,
		GYRO_ADDR_OUT_Z_L, GYRO_ADDR_OUT_Z_H
};

int16_t gyro_xyz[3] ={0,0,0};
int16_t acc_xyz[3] = {0,0,0};

float a_x, a_y, a_z = 0;
float g_x, g_y, g_z = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
extern void initialise_monitor_handles(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool I3G4250D_who_check(void)
{
	uint8_t check = 0;
	uint8_t who = 0x8F;
	HAL_GPIO_WritePin(naszCS_GPIO_Port, naszCS_Pin, 0);
	HAL_SPI_Transmit(&hspi1,&who,1,100);
	HAL_SPI_Receive(&hspi1, &check, 1, 100);
	HAL_GPIO_WritePin(naszCS_GPIO_Port, naszCS_Pin, 1);
	return check == GYRO_WHO_AM_I;
}

void I3G4250D_init(void)
{
	HAL_GPIO_WritePin(naszCS_GPIO_Port, naszCS_Pin, 1);
	HAL_Delay(1);

	uint8_t gyro_ctr1 = 0b00100000;
	uint8_t data = 0b00001111;
	if(I3G4250D_who_check())
	{
		//put gyro in normal mode
		HAL_GPIO_WritePin(naszCS_GPIO_Port, naszCS_Pin, 0);
		HAL_SPI_Transmit(&hspi1,&gyro_ctr1,1,100);
		HAL_SPI_Transmit(&hspi1,&data,1,100);
		HAL_GPIO_WritePin(naszCS_GPIO_Port, naszCS_Pin, 1);
	}
	else
	{
		printf("ERROR: gyro not found\n");
		HAL_Delay(1000);
	}
}

void I3G4250D_Read_Gyro(void)
{
	uint8_t odebrane[6] = {0,0,0,0,0,0};

	for (int i=0; i<6 ; i++)
	{
	  HAL_GPIO_WritePin(naszCS_GPIO_Port, naszCS_Pin, 0);
	  HAL_SPI_Transmit(&hspi1,&gyro_adr[i],1,100);
	  HAL_SPI_Receive(&hspi1, &odebrane[i], 1, 100);
	  HAL_GPIO_WritePin(naszCS_GPIO_Port, naszCS_Pin, 1);
	}

	gyro_xyz[0]= (int16_t)((odebrane[1] << 8) | odebrane[0]);
	gyro_xyz[1]= (int16_t)((odebrane[3] << 8) | odebrane[2]);
	gyro_xyz[2]= (int16_t)((odebrane[5] << 8) | odebrane[4]);

	g_x = gyro_xyz[0] * 1/155;
	g_y = gyro_xyz[1] * 1/155;
	g_z = gyro_xyz[2] * 1/155;
}

void LSM303AGR_init(void)
{
	uint8_t check = 0;
	HAL_I2C_Mem_Read(&hi2c1, ACC_I2C_ADDR, ACC_ADDR_WHO_AM_I, 1, &check, 1, 1000);

	if(check == ACC_WHO_AM_I)
	{
		uint8_t data = 0x57;
		HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDR, ACC_ADDR_CRTL1, 1, &data, 1, 1000);
	}
	else
	{
		printf("ERROR: acc not found\n");
		HAL_Delay(1000);
	}
	HAL_Delay(100);
}

void LSM303AGR_Read_Acc(void)
{
	uint8_t odebrane[6] = {0,0,0,0,0,0};

	for(int i=0; i<6; i++)
	{
		HAL_I2C_Mem_Read(&hi2c1, ACC_I2C_ADDR, ACC_ADDR_OUT_X_L+i, 1, &odebrane[i], 1, 1000);
	}
	acc_xyz[0]= (int16_t)((odebrane[1] << 8) | odebrane[0]);
	acc_xyz[1]= (int16_t)((odebrane[3] << 8) | odebrane[2]);
	acc_xyz[2]= (int16_t)((odebrane[5] << 8) | odebrane[4]);

	//pomnozyć przez 4 mg
	a_x = acc_xyz[0] * 1/1600;
	a_y = acc_xyz[1] * 1/1600;
	a_z = acc_xyz[2] * 1/1600;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char *front_slash = {0b00101111}; // '/'
  char *back_slash = {0b10100100};  // '\'
  char *box = {0b11111111};

  char *gyro_X[3];
  char *gyro_Y[3];
  char *gyro_Z[3];
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();

  initialise_monitor_handles();

  I3G4250D_init();
  LSM303AGR_init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  I3G4250D_Read_Gyro();
	  LSM303AGR_Read_Acc();

	  printf("%.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\n"
			  ,g_x,  g_y,   g_z,   a_x,   a_y,   a_z);

	  sprintf(gyro_X, "%.0f", g_x);
	  sprintf(gyro_Y, "%.0f", g_y);
	  sprintf(gyro_Z, "%.0f", g_z);

	  // Normalnie
	  if ((a_z>=9) && (a_x<=3 && a_x>=-3) && (a_y<=3 && a_y>=-3))
	  	  {
		  lcd_str_XY(0,1, &box);
		  lcd_str_XY(1,1, &box);
		  lcd_str_XY(2,1, &box);
	  	  }
	  // Do góry nogami
	  else if (a_z<=-9 && (a_x<=3 && a_x>=-3) && (a_y<=3 && a_y>=-3))
	  	  {
		  lcd_str_XY(0,0, &box);
		  lcd_str_XY(1,0, &box);
		  lcd_str_XY(2,0, &box);
	  	  }
	  // Do przodu
	  else if (a_x>=3 && a_z<=9)
		  {
		  lcd_str_XY(0,0, &box);
		  lcd_str_XY(1,0, &box);
		  lcd_str_XY(2,0, &box);
		  lcd_str_XY(0,1, &box);
		  lcd_str_XY(1,1, &box);
		  lcd_str_XY(2,1, &box);
		  }
	  // Do tyłu
	  else if (a_x<=-3 && a_z<=9)
		  {
		  lcd_str_XY(0,1, "_");
		  lcd_str_XY(1,1, "_");
		  lcd_str_XY(2,1, "_");
		  }
	  // Przechył w lewo
	  else if ((a_y>=-9 && a_y<=-3) && (a_z>=3 && a_z<=9))
	  	  {
		  lcd_str_XY(0,0, &box);
		  lcd_str_XY(1,0, &back_slash);
		  lcd_str_XY(0,1, &box);
		  lcd_str_XY(1,1, &box);
		  lcd_str_XY(2,1, &box);
	  	  }
	  // Pion w lewo
	  else if ((a_y<=-10) && (a_z<=3))
	  	  {
		  lcd_str_XY(0,0, &box);
		  lcd_str_XY(0,1, &box);
	  	  }
	  //Przechył w prawo
	  else if ((a_y<=9 && a_y>=3) && (a_z>=3 && a_z<=9))
	  	  {
		  lcd_str_XY(2,0, &box);
		  lcd_str_XY(1,0, &front_slash);
		  lcd_str_XY(0,1, &box);
		  lcd_str_XY(1,1, &box);
		  lcd_str_XY(2,1, &box);
	  	  }
	  //Pion w prawo
	  else if ((a_y>=10) && (a_z<=3))
	  	  {
		  lcd_str_XY(2,0, &box);
		  lcd_str_XY(2,1, &box);
	  	  }

	  lcd_str_XY(4, 0, "X:");
	  lcd_str_XY(4, 1, &gyro_X);
	  lcd_str_XY(8, 0, "Y:");
	  lcd_str_XY(8, 1, &gyro_Y);
	  lcd_str_XY(12, 0, "Z:");
	  lcd_str_XY(12, 1, &gyro_Z);

	  HAL_Delay(250);
	  lcd_clear();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, naszCS_Pin|LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin
                          |LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|LCD_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : naszCS_Pin LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin
                           LCD_D7_Pin */
  GPIO_InitStruct.Pin = naszCS_Pin|LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin
                          |LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
