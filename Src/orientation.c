//
// Created by Marton on 2018. 02. 24..
//
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdint-gcc.h>
#include <stm32f407xx.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hal_i2c.h"

/* Variables -----------------------------------------------------------------*/

/************************ Gyro, accelo, magneto **********************************/
#define    MPU9250_ADDRESS            0x68
#define    MAGNETO_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define    MAGNETO                   1

int device = 1;

// GPS
#define GPS_ADDRESS 0x42

// I2C ADDRESS
#define HMC5983_ADDRESS 0x1E

// I2C COMMANDS
#define HMC5983_WRITE	0x3C
#define HMC5983_READ 	0x3D

/********************* Gyro, accelo, magneto END *********************************/
/* Function prototypes -------------------------------------------------------*/


/* Hook prototypes */


void Task_Orientation(void* param)
{
	(void)param; /* Suppress unused parameter warning */

    // Config: gnd -> gnd
    // 5V -> VCC
    // PB10 -> SCL, PB11 -> SDA

	vTaskDelay(1000);

	I2C_WriteRegister(I2C1, MPU9250_ADDRESS<<1, 0x19, 0x09, 1);
	//I2C_WriteRegister(I2C1, MPU9250_ADDRESS<<1, 0x19, 0x07, 1);
	I2C_WriteRegister(I2C1, MPU9250_ADDRESS<<1, 0x1A, 0x00, 1);
	I2C_WriteRegister(I2C1, MPU9250_ADDRESS<<1, 0x1B, GYRO_FULL_SCALE_2000_DPS, 1);
	I2C_WriteRegister(I2C1, MPU9250_ADDRESS<<1, 0x1C, ACC_FULL_SCALE_16_G, 1);
	I2C_WriteRegister(I2C1, MPU9250_ADDRESS<<1, 0x6B, 0x00, 1);
	I2C_WriteRegister(I2C1, MPU9250_ADDRESS<<1, 0x37, 0x02, 1);
	I2C_WriteRegister(I2C1, MAGNETO_ADDRESS<<1, 0x0A, 0x06, 1);
	while(1)
	{
		uint8_t Buf[14];
		I2C_ReadRegister(I2C1, MPU9250_ADDRESS<<1, 0x3B, Buf, 14);

		// Accelerometer
		int16_t ax=-(Buf[0]<<8 | Buf[1]);
		int16_t ay=-(Buf[2]<<8 | Buf[3]);
		int16_t az=Buf[4]<<8 | Buf[5];

		// Gyroscope
		int16_t gx=-(Buf[8]<<8 | Buf[9]);
		int16_t gy=-(Buf[10]<<8 | Buf[11]);
		int16_t gz=Buf[12]<<8 | Buf[13];

		// _____________________
		// :::  Magnetometer :::


		// Read register Status 1 and wait for the DRDY: Data Ready
		uint8_t ST1;
		do
		{
			I2C_ReadRegister(I2C1, MAGNETO_ADDRESS<<1, 0x02, ST1, 1);
		}
		while (!(ST1&0x01));

		// Read magnetometer data
		uint8_t Mag[7];

		//Read magnet sensor data.
		I2C_ReadRegister(I2C1, MAGNETO_ADDRESS<<1, 0x03, Mag, 7);


		// Create 16 bits values from 8 bits data

		// Magnetometer
		int16_t my=-(Mag[1]<<8 | Mag[0]);
		int16_t mx=-(Mag[3]<<8 | Mag[2]);
		int16_t mz=-(Mag[5]<<8 | Mag[4]);
	}
}
