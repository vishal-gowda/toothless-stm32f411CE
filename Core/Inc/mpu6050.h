/*
 * mpu6050.h
 *
 *  Created on: Jul 14, 2021
 *      Author: vishal
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

int16_t temperature;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;


void gyro_setup(I2C_HandleTypeDef *I2Cx);
void gyro_read(I2C_HandleTypeDef *I2Cx);

#endif /* INC_MPU6050_H_ */
