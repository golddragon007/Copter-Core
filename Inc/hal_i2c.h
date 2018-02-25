/*
 * hal_i2c.h
 *
 *  Created on: 2018. febr. 25.
 *      Author: Marton
 */

#ifndef HAL_I2C_H_
#define HAL_I2C_H_

HAL_StatusTypeDef I2C_ReadRegister(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count);
HAL_StatusTypeDef I2C_WriteRegister(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count);
HAL_StatusTypeDef TM_I2C_WriteNoRegister(I2C_TypeDef* I2Cx, uint8_t device_address, uint8_t data);

#endif /* HAL_I2C_H_ */
