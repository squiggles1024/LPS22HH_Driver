/*
 * LPS22HH_Hardware.h
 *
 *  Created on: Mar 24, 2022
 *      Author: evanl
 */

#ifndef INC_LPS22HH_HARDWARE_H_
#define INC_LPS22HH_HARDWARE_H_
/*******************************************************************************
 * Includes
 *******************************************************************************/
#include <stdint.h>

/*******************************************************************************
 * Enumerations
 *******************************************************************************/
typedef enum{
	LPS22HH_IRQEnable = 0,
	LPS22HH_IRQDisable,
	LPS22HH_ReadIntPin
}LPS22HH_Cmd_t;


typedef enum{
	LPS22HH_Ok = 0,
	LPS22HH_Error = 1
}LPS22HH_Status_t;

/*******************************************************************************
 * Driver Structs
 *******************************************************************************/
typedef struct{
	void (*Init)(void);
	void (*DeInit)(void);
	LPS22HH_Status_t (*WriteReg)(uint8_t, uint8_t*, uint8_t); //reg, data buf, length
	LPS22HH_Status_t (*ReadReg)(uint8_t, uint8_t*, uint8_t);  //reg, data buf, length
	uint8_t (*ioctl)(LPS22HH_Cmd_t);
}LPS22HH_IO_Drv_t;

/*******************************************************************************
 * Public Variables
 *******************************************************************************/
extern LPS22HH_IO_Drv_t LPS22HH_Hardware_Drv;

#endif /* INC_LPS22HH_HARDWARE_H_ */
