/*
 * LPS22HH_Hardware.c
 *
 *  Created on: Mar 24, 2022
 *      Author: evanl
 */
/**************************************//**************************************//**************************************
 * Includes
 **************************************//**************************************//**************************************/
#include "LPS22HH_Hardware.h"
#include <stdio.h>
#include "stm32u5xx_hal.h"
#include "gpio.h"
#include "i2c.h"
#include "log.h"

/**************************************//**************************************//**************************************
 * Defines
 **************************************//**************************************//**************************************/
#define LPS22HH_DEVICE_ADDRESS (0xBAU)

/**************************************//**************************************//**************************************
 * Private Function Prototypes
 **************************************//**************************************//**************************************/
static void LPS22HH_Init();
static void LPS22HH_DeInit();
static LPS22HH_Status_t LPS22HH_WriteReg(uint8_t reg, uint8_t *pdata, uint8_t length);
static LPS22HH_Status_t LPS22HH_ReadReg(uint8_t reg, uint8_t *pdata, uint8_t length);
static uint8_t LPS22HH_ioctl(LPS22HH_Cmd_t command);

/**************************************//**************************************//**************************************
 * Private Function Definitions
 **************************************//**************************************//**************************************/

/*Initializes low level IO*/
static void LPS22HH_Init(){
	HAL_Delay(5); //Device takes 4.5 ms to boot.
	LPS22HH_GPIO_Init();
	MX_I2C2_Init();
}

/*DeInitializes low level IO.*/
static void LPS22HH_DeInit(){
	//Do Not De-Init I2C Peripheral as other devices may be using it.
	HAL_GPIO_DeInit(LPS22HH_IRQ_GPIO_Port, LPS22HH_IRQ_Pin);
}

/*Sends data to register over I2C2 Bus*/
static LPS22HH_Status_t LPS22HH_WriteReg(uint8_t reg, uint8_t *pdata, uint8_t length){
	if(HAL_I2C_Mem_Write(&hi2c2, LPS22HH_DEVICE_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, pdata , length, 5000) != HAL_OK){
		_log(log_i2c,"Write to LPS22HH Reg address %x failed.",reg);
		return LPS22HH_Error;
	}
	return LPS22HH_Ok;
}

/*Reads data from register over I2C2 Bus*/
static LPS22HH_Status_t LPS22HH_ReadReg(uint8_t reg, uint8_t *pdata, uint8_t length){
	if(HAL_I2C_Mem_Read(&hi2c2, LPS22HH_DEVICE_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, pdata , length, 5000) != HAL_OK){
		_log(log_i2c,"Read from LPS22HH Reg address %x failed.", reg);
		return LPS22HH_Error;
	}
	return LPS22HH_Ok;
}

/*Performs any other needed functions for the driver.*/
static uint8_t LPS22HH_ioctl(LPS22HH_Cmd_t command){
	uint8_t PinStatus;
	switch(command){

	case LPS22HH_IRQEnable:
		NVIC_EnableIRQ(LPS22HH_IRQ_EXTI_IRQn);
		return LPS22HH_Ok;
		break;

	case LPS22HH_IRQDisable:
		NVIC_DisableIRQ(LPS22HH_IRQ_EXTI_IRQn);
		return LPS22HH_Ok;
		break;

	case LPS22HH_ReadIntPin:
		PinStatus = HAL_GPIO_ReadPin(LPS22HH_IRQ_GPIO_Port, LPS22HH_IRQ_Pin);
		if(PinStatus == GPIO_PIN_SET){
			return 1;
		} else {
			return 0;
		}
	default:
		break;

	}
	return 0;
}


/**************************************//**************************************//**************************************
 * Public Variable Defitinion
 **************************************//**************************************//**************************************/
LPS22HH_IO_Drv_t LPS22HH_Hardware_Drv = {
		.Init = LPS22HH_Init,
		.DeInit = LPS22HH_DeInit,
		.WriteReg = LPS22HH_WriteReg,
		.ReadReg = LPS22HH_ReadReg,
		.ioctl = LPS22HH_ioctl
};

