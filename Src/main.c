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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//LCD
#include "i2c-lcd.h"
#include "ssd1306.h"
//Virtual Comport
#include "usbd_cdc_if.h"
#include "string.h"
//IMU
#include "MPU_6050.h"
//NANOEDGE AI
#include "NanoEdgeAI.h"
#include "knowledge.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float input_user_buffer[DATA_INPUT_USER * AXIS_NUMBER]; // Buffer of input values
float output_class_buffer[CLASS_NUMBER]; // Buffer of class probabilities
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
SensorData_t SensorData;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
void fill_buffer(float sample_buffer[])
{
	/* USER BEGIN */
	 for (size_t  i = 0; i < DATA_INPUT_USER; i++){
		MPU_6050_Accelerometer_Read(&SensorData);
		input_user_buffer[AXIS_NUMBER*i]=SensorData.ACCEL_XOUT;
		input_user_buffer[(AXIS_NUMBER*i)+1]=SensorData.ACCEL_YOUT;
		input_user_buffer[(AXIS_NUMBER*i)+2]=SensorData.ACCEL_ZOUT;
		HAL_Delay(50);
	}
	/* USER END */
}
const char *id2class[CLASS_NUMBER + 1] = { // Buffer for mapping class id to class name
	"unknown",
	"Normal_State",
	"Stop_State",
	"Inside_Touch",
	"Missaligment",
	"Outside_Touch",
};
uint8_t Mode = 0;
volatile uint8_t button_pressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
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

  /* USER CODE BEGIN 1 */
  enum neai_state error_code = neai_classification_init(knowledge);
  if (error_code != NEAI_OK) {
		/* This happens if the knowledge does not correspond to the library or if the library works into a not supported board. */
	}
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  ssd1306_Init();
  HAL_Delay(200);
  ssd1306_Fill(Black);
  ssd1306_SetCursor(41,0);
  ssd1306_WriteString("STATUS:", Font_7x10, White);
  ssd1306_UpdateScreen();
  MPU_6050_Init();
  uint16_t id_class = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(Mode == 0 ){
		fill_buffer(input_user_buffer);
		neai_classification(input_user_buffer, output_class_buffer, &id_class);
		Lcd_update(id_class);
		Uart_Transfer(id_class);
	}
	else{
	Data_Collect_Mode(Mode);
	}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x40B285C2;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x40B285C2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Lcd_update(uint16_t Status){
		ssd1306_Fill(Black);
		ssd1306_SetCursor(41,0);
		ssd1306_WriteString("STATUS:", Font_7x10, White);
		switch (Status) {
	case 1:
		 ssd1306_SetCursor(30,12);
		 ssd1306_WriteString("Normal", Font_11x18, White);
		 break;
	case 2:
		ssd1306_SetCursor(25,12);
		ssd1306_WriteString("Stopped", Font_11x18, White);
		break;
	case 3:
		ssd1306_SetCursor(30,12);
		ssd1306_WriteString("InSide", Font_11x18, White);
		break;
	case 4:
		ssd1306_SetCursor(25,12);
		ssd1306_WriteString("Balance", Font_11x18, White);
		break;
	case 5:
		ssd1306_SetCursor(25,12);
		ssd1306_WriteString("OutSide", Font_11x18, White);
		break;
	default:
		ssd1306_SetCursor(30,12);
		ssd1306_WriteString("ERROR", Font_11x18, White);
	}
	ssd1306_UpdateScreen();
}
void Uart_Transfer(uint16_t Status){
	switch (Status) {
		case 1:
			CDC_Transmit_FS("Normal \n", strlen("Normal \n"));
			break;
		case 2:
			CDC_Transmit_FS("Stopped \n", strlen("Stopped \n"));
			break;
		case 3:
			CDC_Transmit_FS("InSide Touch \n", strlen("InSide Touch \n"));
			break;
		case 4:
			CDC_Transmit_FS("Balance Issue \n", strlen("Balance Issue \n"));
			break;
		case 5:
			CDC_Transmit_FS("OutSide Touch \n", strlen("OutSide Touch \n"));
			break;
		default:
			CDC_Transmit_FS("ERROR \n", strlen("ERROR \n"));
			break;
		}
}
void Data_Collect_Mode(){
		ssd1306_Fill(Black);
		ssd1306_SetCursor(41,0);
		ssd1306_WriteString("STATUS:", Font_7x10, White);
		ssd1306_SetCursor(9,12);
		ssd1306_WriteString("Collecting", Font_11x18, White);
		ssd1306_UpdateScreen();
		char data[10];
		 for (size_t  i = 0; i < DATA_INPUT_USER; i++){

			MPU_6050_Accelerometer_Read(&SensorData);
			input_user_buffer[AXIS_NUMBER*i]=SensorData.ACCEL_XOUT;
			input_user_buffer[(AXIS_NUMBER*i)+1]=SensorData.ACCEL_YOUT;
			input_user_buffer[(AXIS_NUMBER*i)+2]=SensorData.ACCEL_ZOUT;
			HAL_Delay(50);
		}
		//HAL_UART_Transmit(&huart1, " \n", strlen(" \n"), 100);
		for(size_t  i = 0; i < DATA_INPUT_USER*AXIS_NUMBER; i++){
			sprintf(data,"%.4f\t",input_user_buffer[i]);
			CDC_Transmit_FS(data, strlen(data));
			HAL_Delay(10);
		}
		CDC_Transmit_FS(" \n", strlen(" \n"));
		HAL_Delay(200);

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	  if(GPIO_Pin == GPIO_PIN_13) {
		  if (Mode == 1){
			  Mode = 0;
			  CDC_Transmit_FS("Detection Activeted \n", strlen("Detection Activeted \n"));
		  }
		  else if (Mode == 0){
		  	  Mode = 1;
		  	  CDC_Transmit_FS("Collection Activeted \n", strlen("Collection Activeted \n"));
		  }
	  } else {
      __NOP();
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
