/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6500.h"

#include "dshot.h"
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
uint16_t my_motor_value_off[4] = {0, 0, 0, 0};
uint16_t my_motor_value_on[4] = {200, 200, 200, 200};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* spi code stack overflow

#define SPI_TIMEOUT 100

HAL_StatusTypeDef ReadRegister(uint8_t addr, uint8_t *byte)
{
    HAL_StatusTypeDef hal_status;
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = addr | 0x80;  // read operation
    tx_data[1] = 0;            // dummy byte for response

    hal_status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, SPI_TIMEOUT);

    if (hal_status == HAL_OK)
    {
        *byte = rx_data[1];    // response is in the second byte
    }
    return hal_status;
}

*/
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



  Mpu6500 mpu6500;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  dshot_init(DSHOT600);

  uint16_t start_cmd[4] = {0, 0, 0, 0};
  uint16_t idle_cmd[4] = {48, 48, 48, 48};
  uint16_t run_cmd[4] = {100, 100, 100, 100};
  uint16_t beep_cmd[4] = {1, 2, 3, 4};



  while (1)
  {
	  char msg[20];
	  int len = sprintf(msg, "starting\n");
	  CDC_Transmit_FS((uint8_t*)msg, len);

	  Mpu6500_Init(&mpu6500, &hi2c1);
	 // char msg[20];
	  if (Mpu6500_ConfigSrd(&mpu6500, 19)) {
	       // Initialization successful - proceed

		  int len = sprintf(msg, "Did work\n");
		  CDC_Transmit_FS((uint8_t*)msg, len);
	  }
	  else {
	       // Initialization failed - handle error

			  int len = sprintf(msg, "Did not work\n");
			  CDC_Transmit_FS((uint8_t*)msg, len);
			  // Or your preferred error handling
	   }
	  char msg2[100]; // Buffer for string formatting - make sure it's large enough
	  int len2;

	  if (Mpu6500_Read(&mpu6500)) {
	      len2 = sprintf(msg2, "did work! %d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
	    		  mpu6500.new_imu_data,
				  mpu6500.accel[0],
				  mpu6500.accel[1],
				  mpu6500.accel[2],
				  mpu6500.gyro[0],
				  mpu6500.gyro[1],
				  mpu6500.gyro[2],
				  mpu6500.temp);
	      CDC_Transmit_FS((uint8_t*)msg2, len2);
	  }
	  else{
		  len2 = sprintf(msg2, "did not work :(\n");
		  	      CDC_Transmit_FS((uint8_t*)msg2, len2);
	  }

      //HAL_Delay(500);

	  /*
	  ReadRegister(63,&byte);

	  char msg[20];
	  int len = sprintf(msg, "Hello from STM32!!: %d\r\n", byte);
	  CDC_Transmit_FS((uint8_t*)msg, len);
      HAL_Delay(1000);
*/

/*

	  char msg[20];
	  uint8_t who_am_i = 0;
	  HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x75,I2C_MEMADD_SIZE_8BIT,&who_am_i,1,100);
c
*/

	  //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);

	  /*** writing to dshot motor working!!
	   *   for (int i = 0; i < 250; i++)  // 500ms / 2ms = 250 iterations
  {
      dshot_write(start_cmd);
      HAL_Delay(2);
  }
	   *
	  for (int i = 0; i < 250; i++)  // 500ms / 2ms = 250 iterations
	  {
	      dshot_write(idle_cmd);
	      HAL_Delay(2);
	  }
	  dshot_write(beep_cmd);
	 HAL_Delay(2);
	  for (int i = 0; i < 250; i++)  // 500ms / 2ms = 250 iterations
	  {
	      dshot_write(run_cmd);
	      HAL_Delay(2);
	  }
*/

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
