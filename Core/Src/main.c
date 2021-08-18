/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "mpu6050.h"
#include "math.h"
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
const float pid_p_gain_roll = 1.0;               //Gain setting for the pitch and roll P-controller (default = 1.3).
const float pid_i_gain_roll = 0.02;              //Gain setting for the pitch and roll I-controller (default = 0.04).
const float pid_d_gain_roll = 10.0;              //Gain setting for the pitch and roll D-controller (default = 18.0).
const int16_t pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int16_t pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 3.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int16_t pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).

int16_t channel_1_start = 0;
int16_t channel_1 = 0;
uint8_t channel_1_captured = 0;
int16_t channel_2_start = 0;
int16_t channel_2 = 0;
uint8_t channel_2_captured = 0;
int16_t channel_3_start = 0;
int16_t channel_3 = 0;
uint8_t channel_3_captured = 0;
int16_t channel_4_start = 0;
int16_t channel_4 = 0;
uint8_t channel_4_captured = 0;
uint8_t start;

uint32_t loop_timer;
int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle;

int32_t acc_total_vector;
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float battery_voltage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t micros(void) {
	uint32_t m0 = HAL_GetTick();
	uint32_t u0 = SysTick->LOAD - SysTick->VAL;
	uint32_t m1 = HAL_GetTick();
	uint32_t u1 = SysTick->LOAD - SysTick->VAL;
	if (m1 > m0) {
		return ( m1 * 1000 + (u1 * 1000) / SysTick->LOAD);
	} else {
		return ( m0 * 1000 + (u0 * 1000) / SysTick->LOAD);
	}
}

int _write(int file, char *ptr, int len) {
	CDC_Transmit_FS((uint8_t*) ptr, len); return len;
}

void calculate_pid(void){
	//Roll calculations
	pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
	if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
	else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

	pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
	if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
	else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

	pid_last_roll_d_error = pid_error_temp;

	//Pitch calculations
	pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
	if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
	else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

	pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
	if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
	else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

	pid_last_pitch_d_error = pid_error_temp;

	//Yaw calculations
	pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
	if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
	else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

	pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
	if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
	else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_error_temp;
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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	HAL_Delay(5000);
	gyro_setup(&hi2c2);
	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

	loop_timer = micros();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		//printf("channel_1 = %d", channel_1);
		//printf(" channel_2 = %d", channel_2);
		//printf(" channel_3 = %d", channel_3);
		//printf(" channel_4 = %d\n", channel_4);
		gyro_read(&hi2c2);

		//65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
		gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
		gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
		gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

		//Gyro angle calculations
		//0.0000611 = 1 / (250Hz / 65.5)
		angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
		angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

		//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
		angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
		angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

		//Accelerometer angle calculations
		acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

		if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
			angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
		}
		if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
			angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
		}

		//Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
		angle_pitch_acc -= 1.01; 														 //Accelerometer calibration value for pitch.
		angle_roll_acc -= 2.11;    														 //Accelerometer calibration value for roll.

		angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
		angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

		pitch_level_adjust = angle_pitch * 15;                                           //Calculate the pitch angle correction.
		roll_level_adjust = angle_roll * 15;                                             //Calculate the roll angle correction.

		//printf("Pitch: %f  ",angle_pitch);
		//printf("Roll : %f\n", angle_roll);

		//For starting the motors: throttle low and yaw left (step 1).
		if (channel_3 < 1050 && channel_4 < 1050)start = 1;
		//When yaw stick is back in the center position start the motors (step 2).
		if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {
			start = 2;
			HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);

			angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
			angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

			//Reset the PID controllers for a bumpless start.
			pid_i_mem_roll = 0;
			pid_last_roll_d_error = 0;
			pid_i_mem_pitch = 0;
			pid_last_pitch_d_error = 0;
			pid_i_mem_yaw = 0;
			pid_last_yaw_d_error = 0;
		}
		//Stopping the motors: throttle low and yaw right.
		if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
			start = 0;
			HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
		}

		//The PID set point in degrees per second is determined by the roll receiver input.
		//In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_roll_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if (channel_1 > 1508)pid_roll_setpoint = channel_1 - 1508;
		else if (channel_1 < 1492)pid_roll_setpoint = channel_1 - 1492;

		pid_roll_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
		pid_roll_setpoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


		//The PID set point in degrees per second is determined by the pitch receiver input.
		//In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_pitch_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if (channel_2 > 1508)pid_pitch_setpoint = channel_2 - 1508;
		else if (channel_2 < 1492)pid_pitch_setpoint = channel_2 - 1492;

		pid_pitch_setpoint -= pitch_level_adjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
		pid_pitch_setpoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

		//The PID set point in degrees per second is determined by the yaw receiver input.
		//In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_yaw_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if (channel_3 > 1050) { //Do not yaw when turning off the motors.
			if (channel_4 > 1508)pid_yaw_setpoint = (channel_4 - 1508) / 3.0;
			else if (channel_4 < 1492)pid_yaw_setpoint = (channel_4 - 1492) / 3.0;
		}

		calculate_pid();                                                                 //PID inputs are known. So we can calculate the pid output.

		//The battery voltage is needed for compensation.
		//A complementary filter is used to reduce noise.
		//1410.1 = 112.81 / 0.08.
		//battery_voltage = battery_voltage * 0.92 + ((float)HAL_ADC_GetValue(&hadc1)/ 1410.1);

		//Turn on the led if battery voltage is to low. In this case under 10.0V
		//if (battery_voltage < 10.0 && error == 0)//TODO:Vishal


		throttle = channel_3;                                                            //We need the throttle signal as a base signal.

		if (start == 2) {                                                                //The motors are started.
			if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.

			//printf("throttle = %d\n", throttle);

			esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
			esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
			esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
			esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

			if (esc_1 < 1100) esc_1 = 1100;                                                //Keep the motors running.
			if (esc_2 < 1100) esc_2 = 1100;                                                //Keep the motors running.
			if (esc_3 < 1100) esc_3 = 1100;                                                //Keep the motors running.
			if (esc_4 < 1100) esc_4 = 1100;                                                //Keep the motors running.

			if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
			if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
			if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
			if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.
		}
		else {
			esc_1 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
			esc_2 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
			esc_3 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
			esc_4 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
		}


		htim4.Instance->CCR1 = esc_1;
		htim4.Instance->CCR2 = esc_2;
		htim4.Instance->CCR3 = esc_3;
		htim4.Instance->CCR4 = esc_4;
		htim4.Instance->CNT = 5000;

		//printf("esc1 = %d", esc_1);
		//printf(" esc2 = %d", esc_2);
		//printf(" esc3 = %d", esc_3);
		//printf(" esc4 = %d\n", esc_4);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (micros() - loop_timer > 4215){
			HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
		}
		while (micros() - loop_timer < 4165);
		//int lt = micros() - loop_timer;
		loop_timer = micros();
		//printf("%d\n", lt);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 74;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 74;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
//{
//    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
//}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim2.Instance == htim->Instance) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			if (channel_1_captured == 0) {
				channel_1_start = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
				channel_1_captured = 1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
			} else if (channel_1_captured == 1) {
				channel_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) - channel_1_start;
				channel_1_captured = 0;
				if (channel_1 < 0)
					channel_1 += 0xFFFF;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			if (channel_2_captured == 0) {
				channel_2_start = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
				channel_2_captured = 1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_FALLING);
			} else if (channel_2_captured == 1) {
				channel_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) - channel_2_start;
				channel_2_captured = 0;
				if (channel_2 < 0)
					channel_2 += 0xFFFF;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,TIM_INPUTCHANNELPOLARITY_RISING);
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			if (channel_3_captured == 0) {
				channel_3_start = (int16_t)HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
				channel_3_captured = 1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_FALLING);
			} else if (channel_3_captured == 1) {
				channel_3 = (int16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3) - channel_3_start;
				channel_3_captured = 0;
				if (channel_3 < 0)
					channel_3 += 0xFFFF;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_RISING);
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			if (channel_4_captured == 0) {
				channel_4_start = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
				channel_4_captured = 1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,TIM_INPUTCHANNELPOLARITY_FALLING);
			} else if (channel_4_captured == 1) {
				channel_4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4) - channel_4_start;
				channel_4_captured = 0;
				if (channel_4 < 0)
					channel_4 += 0xFFFF;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,TIM_INPUTCHANNELPOLARITY_RISING);
			}
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
	//__disable_irq();
	while (1) {
		HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
		HAL_Delay(1000);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
