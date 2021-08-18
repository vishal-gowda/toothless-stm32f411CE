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

int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

void gyro_setup(I2C_HandleTypeDef *I2Cx){
	uint8_t Data;
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, 0x75, 1, &Data, 1, 100);
	if ( ret == HAL_OK && Data == 0x68)
	{
		Data = 0x00; // power management
		ret = HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x6B, 1, &Data, 1, 100);
		Data = 0x08; // 500dps full scale
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x1B, 1, &Data, 1, 100);
		Data = 0x10; //+/- 8g full scale range
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x1C, 1, &Data, 1, 100);
		Data = 0x03; // Digital Low Pass Filter to ~43Hz
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x1A, 1, &Data, 1, 100);
		HAL_Delay(500);
		for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {
			if (cal_int % 25 == 0) HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
			gyro_read(I2Cx);
			gyro_roll_cal += gyro_roll;
			gyro_pitch_cal += gyro_pitch;
			gyro_yaw_cal += gyro_yaw;
			HAL_Delay(4);
		}
		gyro_roll_cal /= 2000;
		gyro_pitch_cal /= 2000;
		gyro_yaw_cal /= 2000;
		manual_gyro_pitch_cal_value = gyro_pitch_cal;
		manual_gyro_roll_cal_value = gyro_roll_cal;
		manual_gyro_yaw_cal_value = gyro_yaw_cal;
	}
	else{
		Error_Handler();
	}
}

void gyro_read(I2C_HandleTypeDef *I2Cx){
	uint8_t Rec_Data[14];
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, 0x3B, 1, Rec_Data, 14,100);
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

