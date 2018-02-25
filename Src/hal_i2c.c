
#include "stm32f4xx_hal.h"

/* Handle values for I2C */
#ifdef I2C1
static I2C_HandleTypeDef I2C1Handle = {I2C1};
#endif
#ifdef I2C2
static I2C_HandleTypeDef I2C2Handle = {I2C2};
#endif
#ifdef I2C3
static I2C_HandleTypeDef I2C3Handle = {I2C3};
#endif
#ifdef I2C4
static I2C_HandleTypeDef I2C4Handle = {I2C4};
#endif

I2C_HandleTypeDef* TM_I2C_GetHandle(I2C_TypeDef* I2Cx) {
#ifdef I2C1
    if (I2Cx == I2C1) {
        return &I2C1Handle;
    }
#endif
#ifdef I2C2
    if (I2Cx == I2C2) {
        return &I2C2Handle;
    }
#endif
#ifdef I2C3
    if (I2Cx == I2C3) {
        return &I2C3Handle;
    }
#endif
#ifdef I2C4
    if (I2Cx == I2C4) {
		return &I2C4Handle;
	}
#endif

    /* Return invalid */
    return 0;
}

/** Regiszter olvasĂˇsa I2C-n keresztĂĽl. */
HAL_StatusTypeDef I2C_ReadRegister(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count)
{
    I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);

//    /* Send register address */
//    if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, &register_address, 1, 1000) != HAL_OK) {
//        /* Check error */
//        if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
//
//        }
//
//        /* Return error */
//        return HAL_ERROR;
//    }
//
//    /* Receive multiple byte */
//    if (HAL_I2C_Master_Receive(Handle, device_address, data, count, 1000) != HAL_OK) {
//        /* Check error */
//        if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
//
//        }
//
//        /* Return error */
//        return HAL_ERROR;
//    }

    if (HAL_I2C_Mem_Read(Handle, device_address, register_address, 1, data, count, 5000) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/** Regiszter Ă­rĂˇsa I2C-n keresztĂĽl. */
HAL_StatusTypeDef I2C_WriteRegister(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count)
{
    I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);

    /* Try to transmit via I2C */
//    if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, &data, 1, 1000) != HAL_OK) {
//        /* Check error */
//        if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {
//
//        }
//
//        /* Return error */
//        return HAL_ERROR;
//    }

    if (HAL_I2C_Mem_Write(Handle, device_address, register_address, 1, data, count, 5000) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Return OK */
    return HAL_OK;
}

HAL_StatusTypeDef TM_I2C_WriteNoRegister(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t data) {
    I2C_HandleTypeDef* Handle = TM_I2C_GetHandle(I2Cx);

    /* Try to transmit via I2C */
    if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, &data, 1, 1000) != HAL_OK) {
        /* Check error */
        if (HAL_I2C_GetError(Handle) != HAL_I2C_ERROR_AF) {

        }

        /* Return error */
        return HAL_ERROR;
    }

    /* Return OK */
    return HAL_OK;
}
