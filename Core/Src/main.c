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
#include "MahonyAHRS.h"
#include "dshot.h"

#define MAP_AND_CLAMP(input, imin, imax, omin, omax) \
    ( ((input) < (imin) ? (imin) : ( (input) > (imax) ? (imax) : (input) )) - (imin) ) * ((omax) - (omin)) / ((imax) - (imin)) + (omin)


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

const uint16_t max_motor_value = 100; //+ x* max pid for actual
const uint16_t max_pid = 100;
// angles
const float pid_area_pitch = 20.0f;
const float pid_area_roll = 20.0f;
const float pid_area_yaw = 50.0f;

uint16_t motor_value[4] = {0,0,0,0};
const uint16_t motor_offset = 48;

const int input_min = 22000;
const int input_max = 41000;
const int output_min = 0;
const int output_max = 10000;
const int update_freq = 500;
const float delta_t = 1.0f/update_freq;
//in deg
const float max_delta_yaw = 360.0f*.5f * delta_t;
const float max_rx_angle = 20.0f;

float roll;
float pitch;
float yaw;

int roll_cor;
int pitch_cor;
int yaw_cor;
unsigned long rx_inactive = 0;

float target_yaw = 0.0f;
int throttle;

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

int32_t frequency, duty_cycle;
uint32_t capture_value;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	rx_inactive = 0;
  /*if(htim ->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
    capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    if(capture_value){
      frequency = SystemCoreClock / (capture_value);
      duty_cycle = 10000 * HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) / capture_value - 750;
    }
  }*/
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

  HAL_Delay(1500);

  twoKp = 2.0f * 0.5f;  // Adjust these values based on your application
  twoKi = 2.0f * 0.0f;
  q0 = 1.0f; q1 = q2 = q3 = 0.0f;  // Initial quaternion (no rotation)

  dshot_init(DSHOT600);


	int rx_throttle = 0;

	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);


	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);


	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);


	HAL_TIM_IC_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim9, TIM_CHANNEL_2);

	int wait;
    char msg[1000];
    int len;
	Mpu6500_Init(&mpu6500, &hi2c1);

	if (Mpu6500_ConfigSrd(&mpu6500, 1)) {
		 // Initialization successful - proceed

		  len = sprintf(msg, "imu init worked\n");
		  CDC_Transmit_FS((uint8_t*)msg, len);
	}
	else {
		 // Initialization failed - handle error

			  len = sprintf(msg, "imu init Did not work\n");
			  CDC_Transmit_FS((uint8_t*)msg, len);
			  // Or your preferred error handling
	 }
	HAL_Delay(1000);

  while (1){
	  ++rx_inactive;
	  wait = 0;
	  len = sprintf(msg,"\n######\n");
	  bool succ = false;
	  while(!succ){
		  succ = Mpu6500_Read(&mpu6500);
		  if (succ) {
			  MahonyAHRSupdateIMU(
				mpu6500.gyro[2],
				mpu6500.gyro[0],
				mpu6500.gyro[1],
				mpu6500.accel[2],
				mpu6500.accel[0],
				mpu6500.accel[1]);

		    // Compute Euler angles (roll, pitch, yaw) in radians
		    roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1*q1 + q2*q2));
		    pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
		    yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2*q2 + q3*q3));

		    // Convert to degrees
		    roll *= (180.0f / M_PI);
		    pitch *= (180.0f / M_PI);
		    yaw *= (180.0f / M_PI);

	     /* len = sprintf(msg2, "did work! %d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
	    		  mpu6500.new_imu_data,
				  mpu6500.accel[0],
				  mpu6500.accel[1],
				  mpu6500.accel[2],
				  mpu6500.gyro[0],
				  mpu6500.gyro[1],
				  mpu6500.gyro[2],
				  mpu6500.temp);*/

		  len = sprintf(msg+len,"waited: %3d inactive_since: %4d angles %3.2f\t%3.2f\t%3.2f\t\n quat: %3.2f\t%3.2f\t%3.2f\t%3.2f\n",wait,rx_inactive,roll,pitch, yaw,q0,q1,q2,q3);



		  }
		  else{
					 wait++;// CDC_Transmit_FS("n\n", 2);
		  }
	  }

	    roll += MAP_AND_CLAMP(HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2),input_min,input_max,-max_rx_angle,max_rx_angle);
	    pitch += MAP_AND_CLAMP(HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1),input_min,input_max,-max_rx_angle,max_rx_angle);
	    target_yaw += MAP_AND_CLAMP(HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_2),input_min,input_max,-max_delta_yaw,max_delta_yaw);
	    yaw += target_yaw;

	    if (yaw < 0){yaw += ((int)(-yaw) + 180)/360*360;}
	    else{yaw -= ((int)(yaw) + 180)/360*360;}

	    throttle = MAP_AND_CLAMP(HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2),input_min,input_max,0,max_motor_value);

	    roll_cor = (int)(MAP_AND_CLAMP(roll,-pid_area_roll,pid_area_roll,-max_pid,max_pid));
	    pitch_cor = (int)(MAP_AND_CLAMP(pitch,-pid_area_pitch,pid_area_pitch,-max_pid,max_pid));
	    yaw_cor = (int)(MAP_AND_CLAMP(pitch,-pid_area_yaw,pid_area_yaw,-max_pid,max_pid));


		len += sprintf(msg+len,"roll: %3.2f\t pitch:%3.2f\t yaw:%3.2f\ttarget_yaw:%3.2f throttle: %d\n",roll,pitch, yaw,target_yaw,throttle);


	   /* int len = sprintf(msg, "Roll: %5d Pitch: %5d Yaw: %5d Throttle: %5d\n",
	                      rx_roll, rx_pitch, rx_yaw, rx_throttle);
	    CDC_Transmit_FS((uint8_t*)msg, len);*/

	    motor_value[0] = throttle + roll_cor;// + yaw_cor;
	    motor_value[1] = throttle + pitch_cor;// - yaw_cor;
	    motor_value[2] = throttle - roll_cor;// + yaw_cor;
	    motor_value[3] = throttle - pitch_cor;// - yaw_cor;

	    if(motor_value[0]>50000){motor_value[0] = 0;}
	    if(motor_value[1]>50000){motor_value[1] = 0;}
	    if(motor_value[2]>50000){motor_value[2] = 0;}
	    if(motor_value[3]>50000){motor_value[3] = 0;}

	    if(rx_inactive>200 || throttle == 0){
	    	motor_value[0] = motor_value[1] = motor_value[2] = motor_value[3] = 0;
	    }

		len += sprintf(msg+len,"motors: %d\t%d\t%d\t%d\n",motor_value[0],motor_value[1],motor_value[2],motor_value[3]);


	    dshot_write(motor_value);

	    CDC_Transmit_FS((uint8_t*)msg, len);


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
