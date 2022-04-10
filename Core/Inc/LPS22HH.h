/*
 * LPS22HH.h
 *
 *  Created on: Mar 24, 2022
 *      Author: evanl
 */

#ifndef INC_LPS22HH_H_
#define INC_LPS22HH_H_
/**************************************//**************************************//**************************************
 * Includes
 **************************************//**************************************//**************************************/
#include "LPS22HH_Hardware.h"
#include <stdint.h>

/**************************************//**************************************//**************************************
 * Typedefs / Enumerations
 **************************************//**************************************//**************************************/

typedef enum{
	LPS22HH_DataReady,
	LPS22HH_DataNotReady
}LPS22HH_DataReadyFlag_t;

typedef enum{
	LPS22HH_PowerDownOneShot, /*One-shot mode is executed in the power down state*/
	LPS22HH_1Hz,
	LPS22HH_10Hz,
	LPS22HH_25Hz,
	LPS22HH_50Hz,
	LPS22HH_75Hz,
	LPS22HH_100Hz,
	LPS22HH_200Hz
}LPS22HH_OutputDataRate_t;


typedef enum{
	LPS22HH_IRQActiveHigh,
	LPS22HH_IRQActiveLow
}LPS22HH_IRQPolarity_t;

typedef enum{
	LPS22HH_LowCurrentMode,
	LPS22HH_LowNoiseMode
}LPS22HH_LowNoiseCurrentMode_t;

typedef enum{
	LPS22HH_LPFDisabled= 0,
	LPS22HH_Div9 = 2,
	LPS22HH_Div20 = 3
}LPS22HH_LPFMode_t;

/*
 * When the two "Auto____" are enabled, a pressure sample will be taken immediately.
 * This sample, "RefP", will be used as the negative end of a differential pressure signal.
 * In AutoZero mode: The pressure output in PRESS_OUT registers will be a PRESS_OUT(t)- RefP(t0) (Differential Pressure)
 * In AutoRefP mode: The pressure output in the PRESS_OUT registers will be PRESS_OUT(t) (Single ended pressure)
 * AN5290 Reports "PRESS_OUT(t)" as press_out_mux1(t). Both modes use differential pressure to generate an interrupt.
 */
typedef enum{
	LPS22HH_HighPressure = 1 << 0,
	LPS22HH_LowPressure = 1 << 1,
	LPS22HH_LatchIRQMode = 1 << 2, /*Latch means: IRQ line will remain active as soon as IRQ threshold is exceeded, even if it goes back below threshold before IRQ is serviced.*/
	LPS22HH_AutoZero = 1 << 5,
	LPS22HH_AutoRefP = 1 << 7
}LPS22HH_IRQMode_t; /* Bitwise OR These together */


typedef enum{
	LPS22HH_DataReadySignalEvent,
	LPS22HH_PressureHighEvent,
	LPS22HH_PressureLowEvent,
	LPS22HH_PressureHighOrLowEvent,
	LPS22HH_IRQPinDisabled
}LPS22HH_DRDYPinControl_t;


/**************************************//**************************************//**************************************
 * Driver Structs
 **************************************//**************************************//**************************************/
typedef struct{
	float Pressure;
	LPS22HH_DataReadyFlag_t DataReadyFlag;
	LPS22HH_IO_Drv_t LPS22HH_IO;
}LPS22HH_Handle_t;


typedef struct{
	uint16_t PresThreshold;
	int16_t ReferencePressure;
	LPS22HH_OutputDataRate_t OutputDataRate;
	LPS22HH_IRQPolarity_t IRQPolarity;
	LPS22HH_LowNoiseCurrentMode_t LowNoiseLowCurrent;
	LPS22HH_LPFMode_t LowPassFilterMode;
	LPS22HH_IRQMode_t IRQMode;
	LPS22HH_DRDYPinControl_t DataReadyPinCtrl;
}LPS22HH_Init_Struct_t;

/**************************************//**************************************//**************************************
 * Defines
 **************************************//**************************************//**************************************/
#define LPS22HH_DEVICE_ID                          (0xB3)

#define LPS22HH_REG_INTERRUPT_CFG                  (0x0B)
#define LPS22HH_REG_THS_P_L                        (0x0C)
#define LPS22HH_REG_THS_P_H                        (0x0D)
#define LPS22HH_REG_IF_CTRL                        (0x0E)
#define LPS22HH_REG_WHO_AM_I                       (0x0F)
#define LPS22HH_REG_CTRL_REG1                      (0x10)
#define LPS22HH_REG_CTRL_REG2                      (0x11)
#define LPS22HH_REG_CTRL_REG3                      (0x12)
#define LPS22HH_REG_FIFO_CTRL                      (0x13)
#define LPS22HH_REG_FIFO_WTM                       (0x14)
#define LPS22HH_REG_REF_P_L                        (0x15)
#define LPS22HH_REG_REF_P_H                        (0x16)
#define LPS22HH_REG_RPDS_L                         (0x18)
#define LPS22HH_REG_RPDS_H                         (0x19)
#define LPS22HH_REG_INT_SOURCE                     (0x24)
#define LPS22HH_REG_FIFO_STATUS1                   (0x25)
#define LPS22HH_REG_FIFO_STATUS2                   (0x26)
#define LPS22HH_REG_STATUS                         (0x27)
#define LPS22HH_REG_PRESSURE_OUT_XL                (0x28)
#define LPS22HH_REG_PRESSURE_OUT_L                 (0x29)
#define LPS22HH_REG_PRESSURE_OUT_H                 (0x2A)
#define LPS22HH_REG_TEMP_OUT_L                     (0x2B)
#define LPS22HH_REG_TEMP_OUT_H                     (0x2C)
#define LPS22HH_REG_FIFO_DATA_OUT_PRESS_XL         (0x78)
#define LPS22HH_REG_FIFO_DATA_OUT_PRESS_L          (0x79)
#define LPS22HH_REG_FIFO_DATA_OUT_PRESS_H          (0x7A)
#define LPS22HH_REG_FIFO_DATA_OUT_TEMP_L           (0x7B)
#define LPS22HH_REG_FIFO_DATA_OUT_TEMP_H           (0x7C)

/**************************************//**************************************//**************************************
 * Public Function Prototypes
 **************************************//**************************************//**************************************/
void LPS22HH_Init(LPS22HH_Init_Struct_t Settings, LPS22HH_Handle_t *Dev, LPS22HH_IO_Drv_t LowLevelDrivers);
void LPS22HH_DeInit(LPS22HH_Handle_t *Dev);
void LPS22HH_Reset(LPS22HH_Handle_t *Dev); /*Ctrl Reg 2*/
LPS22HH_DataReadyFlag_t LPS22HH_ReadPressure(LPS22HH_Handle_t *Dev);
void LPS22HH_SetPressureThresholds(LPS22HH_Handle_t *Dev, uint16_t Pressure);
void LPS22HH_AutoZeroSet(LPS22HH_Handle_t *Dev);
void LPS22HH_AutoRefpSet(LPS22HH_Handle_t *Dev);
void LPS22HH_AutoReset(LPS22HH_Handle_t *Dev);
void LPS22HH_StartConversion(LPS22HH_Handle_t *Dev);
void LPS22HH_ReadReg(LPS22HH_Handle_t *Dev, uint8_t reg, uint8_t *pdata, uint8_t length);
void LPS22HH_WriteReg(LPS22HH_Handle_t *Dev, uint8_t reg, uint8_t *pdata, uint8_t length);

#endif /* INC_LPS22HH_H_ */
