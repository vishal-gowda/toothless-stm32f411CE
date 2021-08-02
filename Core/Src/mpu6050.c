/*
 * mpu6050.c
 *
 *  Created on: Jul 14, 2021
 *      Author: vishal
 */
#include "main.h"
#include "mpu6050.h"

#define MPU6050_ADDR 0xD0

int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0;
//int16_t manual_acc_pitch_cal_value = 0;
//int16_t manual_acc_roll_cal_value = 0;

int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin,
		GPIO_PinState state, uint32_t timeout) {
	uint32_t Tickstart = HAL_GetTick();
	uint8_t ret = 1;
	/* Wait until flag is set */
	for (; (state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret);) {
		/* Check for the timeout */
		if (timeout != HAL_MAX_DELAY) {
			if ((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout)) {
				ret = 0;
			} else {
			}
		}
		__NOP();
	}
	return ret;
}

static void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *handle,
		uint32_t timeout) {
	GPIO_InitTypeDef GPIO_InitStructure;

	// 1. Clear PE bit.
	CLEAR_BIT(handle->Instance->CR1, I2C_CR1_PE);

	//  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	HAL_I2C_DeInit(handle);

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStructure.Pull = GPIO_NOPULL;

	GPIO_InitStructure.Pin = I2C2_SCL_Pin;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = I2C2_SDA_Pin;
	HAL_GPIO_Init(I2C2_SDA_GPIO_Port, &GPIO_InitStructure);

	// 3. Check SCL and SDA High level in GPIOx_IDR.
	HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET);

	wait_for_gpio_state_timeout(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET,
			timeout);
	wait_for_gpio_state_timeout(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET,
			timeout);

	// 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_RESET);

	// 5. Check SDA Low level in GPIOx_IDR.
	wait_for_gpio_state_timeout(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin,
			GPIO_PIN_RESET, timeout);

	// 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_RESET);

	// 7. Check SCL Low level in GPIOx_IDR.
	wait_for_gpio_state_timeout(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin,
			GPIO_PIN_RESET, timeout);

	// 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET);

	// 9. Check SCL High level in GPIOx_IDR.
	wait_for_gpio_state_timeout(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET,
			timeout);

	// 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET);

	// 11. Check SDA High level in GPIOx_IDR.
	wait_for_gpio_state_timeout(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET,
			timeout);

	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Alternate = MODE_AF;

	GPIO_InitStructure.Pin = I2C2_SCL_Pin;
	HAL_GPIO_Init(I2C2_SCL_GPIO_Port, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = I2C2_SDA_Pin;
	HAL_GPIO_Init(I2C2_SDA_GPIO_Port, &GPIO_InitStructure);

	// 13. Set SWRST bit in I2Cx_CR1 register.
	SET_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
	__NOP();

	/* 14. Clear SWRST bit in I2Cx_CR1 register. */
	CLEAR_BIT(handle->Instance->CR1, I2C_CR1_SWRST);
	__NOP();

	/* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
	SET_BIT(handle->Instance->CR1, I2C_CR1_PE);
	__NOP();

	// Call initialization function.
	HAL_I2C_Init(handle);
}

void gyro_setup(I2C_HandleTypeDef *I2Cx){
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(I2Cx, MPU6050_ADDR, 1, 100);
	if (ret == HAL_BUSY) {
		I2C_ClearBusyFlagErratum(I2Cx, 100);
	}
	uint8_t Data;
	Data = 0x00; // power management
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x6B, 1, &Data, 1,100);
	Data = 0x08; // 500dps full scale
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x1B, 1, &Data, 1,100);
	Data = 0x10; //+/- 8g full scale range
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x1C, 1, &Data, 1,100);
	Data = 0x03; // Digital Low Pass Filter to ~43Hz
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x1A, 1, &Data, 1,100);
	HAL_Delay(3000);
	for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {
		if (cal_int % 25 == 0) HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
		gyro_read(I2Cx);
		gyro_roll_cal += gyro_roll;
		gyro_pitch_cal += gyro_pitch;
		gyro_yaw_cal += gyro_yaw;
		HAL_Delay(3);
	}
	gyro_roll_cal /= 2000;
	gyro_pitch_cal /= 2000;
	gyro_yaw_cal /= 2000;
	manual_gyro_pitch_cal_value = gyro_pitch_cal;
	manual_gyro_roll_cal_value = gyro_roll_cal;
	manual_gyro_yaw_cal_value = gyro_yaw_cal;
}

void gyro_read(I2C_HandleTypeDef *I2Cx){
	uint8_t Rec_Data[14];
	if(HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, 0x3B, 1, Rec_Data, 14,100) == HAL_OK){
		acc_y = Rec_Data[0] <<8 |Rec_Data[1];
		acc_x = Rec_Data[2] <<8 |Rec_Data[3];
		acc_z = Rec_Data[4] <<8 |Rec_Data[5];
		temperature = Rec_Data[6] <<8 |Rec_Data[7];
		gyro_roll = Rec_Data[8] <<8 |Rec_Data[9];
		gyro_pitch = Rec_Data[10] <<8 |Rec_Data[11];
		gyro_yaw = Rec_Data[12] <<8 |Rec_Data[13];
		gyro_pitch *= -1;
		gyro_yaw *= -1;

		gyro_roll -= manual_gyro_roll_cal_value;
		gyro_pitch -= manual_gyro_pitch_cal_value;
		gyro_yaw -= manual_gyro_yaw_cal_value;
	}
}

