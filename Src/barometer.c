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

/***************************** BAROMETER *****************************************/

#define MS5611_ADDRESS                (0x77)

#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,
    MS5611_HIGH_RES         = 0x06,
    MS5611_STANDARD         = 0x04,
    MS5611_LOW_POWER        = 0x02,
    MS5611_ULTRA_LOW_POWER  = 0x00
} ms5611_osr_t;

uint16_t fc[6];
uint8_t ct;
uint8_t uosr;
int32_t TEMP2;
int64_t OFF2, SENS2;

/***************************** BAROMETER END *****************************************/

/* Function prototypes -------------------------------------------------------*/


/* Hook prototypes */

/***************************** BAROMETER *****************************************/


// FROM: https://github.com/jarzebski/Arduino-MS5611
// Set oversampling value
void setOversampling(ms5611_osr_t osr)
{
    switch (osr)
    {
        case MS5611_ULTRA_LOW_POWER:
            ct = 1;
            break;
        case MS5611_LOW_POWER:
            ct = 2;
            break;
        case MS5611_STANDARD:
            ct = 3;
            break;
        case MS5611_HIGH_RES:
            ct = 5;
            break;
        case MS5611_ULTRA_HIGH_RES:
            ct = 10;
            break;
    }

    uosr = osr;
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t readRegister16(uint8_t reg)
{
    uint16_t value;
    uint8_t va[2];

    if (I2C_ReadRegister(I2C1, MS5611_ADDRESS<<1, reg, va, 2) != HAL_OK) {
        return 0xFF;
    }

    value = va[0] << 8 | va[1];

    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t readRegister24(uint8_t reg)
{
    uint32_t value;
    uint8_t va[3];
    if (I2C_ReadRegister(I2C1, MS5611_ADDRESS<<1, reg, va, 2) != HAL_OK) {
        return HAL_ERROR;
    }

    value = ((int32_t)va[0] << 16) | ((int32_t)va[1] << 8) | va[2];

    return value;
}

// Get oversampling value
ms5611_osr_t getOversampling(void)
{
    return (ms5611_osr_t)uosr;
}

void reset(void)
{
    TM_I2C_WriteNoRegister(I2C1, MS5611_ADDRESS<<1, MS5611_CMD_RESET);
}

void readPROM(void)
{
    uint8_t offset;
    for (offset = 0; offset < 6; offset++)
    {
        fc[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
    }
}

uint32_t readRawTemperature(void)
{
    TM_I2C_WriteNoRegister(I2C1, MS5611_ADDRESS<<1, MS5611_CMD_CONV_D2 + uosr);

    vTaskDelay(ct);

    return readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t readRawPressure(void)
{
    TM_I2C_WriteNoRegister(I2C1, MS5611_ADDRESS<<1, MS5611_CMD_CONV_D1 + uosr);

    vTaskDelay(ct);

    return readRegister24(MS5611_CMD_ADC_READ);
}

int32_t readPressure(int compensation)
{
    uint32_t D1 = readRawPressure();

    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

    if (compensation == 1)
    {
        int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

        OFF2 = 0;
        SENS2 = 0;

        if (TEMP < 2000)
        {
            OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
            SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
        }

        if (TEMP < -1500)
        {
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
        }

        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

double readTemperature(int compensation)
{
    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (compensation == 1)
    {
        if (TEMP < 2000)
        {
            TEMP2 = (dT * dT) / (2 << 30);
        }
    }

    TEMP = TEMP - TEMP2;

    return ((double)TEMP/100);
}

// Calculate altitude from Pressure & Sea level pressure
double getAltitude(double pressure, double seaLevelPressure)
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)seaLevelPressure, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double getSeaLevel(double pressure, double altitude)
{
    return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
}
/***************************** BAROMETER END *****************************************/

void Task_Barometer(void* param)
{
    (void)param; /* Suppress unused parameter warning */

    // Config: gnd -> gnd, csb
    // 5V -> VCC, PS
    // PB6 -> SCL, PB7 -> SDA

    /*IMU_register_write_no_value(I2C1, _addr, MS561101BA_RESET);

    vTaskDelay(1000);

    int i_C;
    for (i_C=0;i_C<MS561101BA_PROM_REG_COUNT;i_C++) {
        IMU_register_write_no_value(I2C1, _addr, MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
        vTaskDelay(100);
        I2C_start(I2C1, _addr<<1, I2C_Direction_Receiver);
        _C[i_C] = I2C_read_ack(I2C1) << 8 | I2C_read_nack(I2C1);
    }

    int preset;
    for(preset=0; preset<MOVAVG_SIZE; preset++) {
      movavg_buff[preset] = getPressure(MS561101BA_OSR_4096);
    }*/
    reset();

    setOversampling(MS5611_ULTRA_HIGH_RES);

    vTaskDelay(100);

    readPROM();

    while(1)
    {
        // Standard sea level pressure 101325 : https://en.wikipedia.org/wiki/Atmospheric_pressure#Mean_sea_level_pressure
        // Devide pressure by 100 to get it in hPa

        // Read true temperature & Pressure (without compensation)
        double realTemperature = readTemperature(0);
        long realPressure = readPressure(0);
        double realAltitude = getAltitude(realPressure, 101325);

        // Read true temperature & Pressure (with compensation)
        double realTemperature2 = readTemperature(1);
        long realPressure2 = readPressure(1);
        double realAltitude2 = getAltitude(realPressure2, 101325);

        /*
        // Set you real altitude
        // My location: Poland, Bytom, 8 floor
        double myRealAltitude = 335;
        // Calculate sealevel pressure
        double seaLevelPressure = getSeaLevel(realPressure, myRealAltitude);*/
    }
}
