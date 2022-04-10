/*
 * LPS22HH.c
 *
 *  Created on: Mar 24, 2022
 *      Author: evanl
 */

/**************************************//**************************************//**************************************
 * Includes
 **************************************//**************************************//**************************************/
#include "LPS22HH.h"
#include "log.h"
#include <stddef.h>

/**************************************//**************************************//**************************************
 * Private Function Prototypes
 **************************************//**************************************//**************************************/
static int32_t ConvertPressure(uint8_t* pdata);



/**************************************//**************************************//**************************************
 * Public Function Definitions
 **************************************//**************************************//**************************************/
/**************************************//**************************************
 *@Brief: Initializes LPS22HH Device Handle
 *@Params: Initialization structure with desired params, Device handle pointer, and a structure of low level drivers (see LPS22HH_Hardware.h)
 *@Return: None
 *@Precondition: None
 *@Postcondition: GPIO, and  Communication Interface will be modified and enabled. Device will be initialized with desired settings.
 **************************************//**************************************/
void LPS22HH_Init(LPS22HH_Init_Struct_t Settings, LPS22HH_Handle_t *Dev, LPS22HH_IO_Drv_t LowLevelDrivers){
	Dev->LPS22HH_IO.Init = LowLevelDrivers.Init;
	Dev->LPS22HH_IO.DeInit = LowLevelDrivers.DeInit;
	Dev->LPS22HH_IO.WriteReg = LowLevelDrivers.WriteReg;
	Dev->LPS22HH_IO.ReadReg = LowLevelDrivers.ReadReg;
	Dev->LPS22HH_IO.ioctl = LowLevelDrivers.ioctl;

	Dev->LPS22HH_IO.Init();
	Dev->LPS22HH_IO.ioctl(LPS22HH_IRQDisable);
	uint8_t settings;
	uint8_t buffer8;
	uint16_t buffer16;

	/*************WHO_AM_I*****************/
	//Verify device ID
	Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_WHO_AM_I, &buffer8,1);
	if(buffer8 != LPS22HH_DEVICE_ID){
		_log(log_lps22hh,"Device ID does not match");
	}
	/**************************************/

	/*************CTRL_REG2****************/
	//Set up Int pin polarity, I2C Auto-Increment, and Low Noise/Low Current mode
	settings = (Settings.IRQPolarity  << 6) | (1 << 4) | (Settings.LowNoiseLowCurrent << 1);
	Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_CTRL_REG2,&settings,1);
	Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_CTRL_REG2,&buffer8,1);
	if(settings != buffer8){
		_log(log_lps22hh,"CTRL2 Reg Readback does not match settings");
	}
	/**************************************/

	/************CTRL_REG_1****************/
	//Setup Output data rate, Lowpass filter mode, and enable BDU
	settings = (Settings.OutputDataRate << 4) | (Settings.LowPassFilterMode << 2) | (1 << 1);
	Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_CTRL_REG1, &settings, 1);
	Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_CTRL_REG1, &buffer8, 1);
	if(settings != buffer8){
		_log(log_lps22hh,"CTRL1 Reg Readback does not match settings");
	}
	/**************************************/

	/************IRQ_CFG*******************/

	settings = 0;
	if(Settings.IRQMode != 0 && Settings.IRQMode != LPS22HH_LatchIRQMode){ 	//Set the Differential Enable bit if any bit other than the Latch bit is set (all other IRQs require this bit)
		settings = 1 << 3;
	}
	settings |= Settings.IRQMode;
	Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_INTERRUPT_CFG,&settings,1);
	Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_INTERRUPT_CFG,&buffer8,1);
	if(settings != buffer8){
		_log(log_lps22hh,"IRQ Config Reg Readback does not match settings");
	}
	/**************************************/

	/***************THS_P******************/
	/*Configure IRQ Threshold Pressure*/
	Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_THS_P_L,(uint8_t*)&Settings.PresThreshold,2);
	Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_THS_P_L,(uint8_t*)&buffer16,2);
	if(Settings.PresThreshold != buffer16){
		_log(log_lps22hh,"Threshold pressure does not match settings");
	}
	/**************************************/

	/**************CTRL_REG3***************/
	if(Settings.DataReadyPinCtrl == LPS22HH_IRQPinDisabled){ //Set to 0 if pin is disabled
		settings = 0;
		Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_CTRL_REG3, &settings,1);
	} else {
		settings = ( 1 << 2 )| (Settings.DataReadyPinCtrl << 0);
		Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_CTRL_REG3, &settings,1);
	}
	Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_CTRL_REG3, &buffer8,1);
	if(settings != buffer8){
		_log(log_lps22hh,"Ctrl Reg3 Readback does not match settings");
	}
	/**************************************/

	/**************REFP*******************/
	Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_RPDS_L,(uint8_t*)&Settings.ReferencePressure, 2);
	Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_RPDS_L,(uint8_t*)&buffer16, 2);
	if(Settings.ReferencePressure != (int16_t)buffer16){
		_log(log_lps22hh,"Reference pressure does not match settings");
	}
	Dev->LPS22HH_IO.ioctl(LPS22HH_IRQEnable);
}

/**************************************//**************************************
 *@Brief: Deinitializes LPS22HH Device handle
 *@Params: LPS22HH Device handle to be deinitialized
 *@Return: None
 *@Precondition: Device handle should be initialized and functioning
 *@Postcondition: Device will be SW Reset, DeInit IO function will be called, and LL Driver struct deinitialized
 **************************************//**************************************/
void LPS22HH_DeInit(LPS22HH_Handle_t *Dev){
	LPS22HH_Reset(Dev);
	Dev->LPS22HH_IO.DeInit();
	Dev->LPS22HH_IO.WriteReg = NULL;
	Dev->LPS22HH_IO.ReadReg = NULL;
	Dev->LPS22HH_IO.Init = NULL;
	Dev->LPS22HH_IO.DeInit = NULL;
	Dev->LPS22HH_IO.ioctl = NULL;
}

/**************************************//**************************************
 *@Brief: Performs SW Reset on LPS22HH Device
 *@Params: LPS22HH Device Handle
 *@Return: None
 *@Precondition: LPS22HH Device Handle should be initialized and functional
 *@Postcondition: LPS22HH Device will be reset to default settings
 **************************************//**************************************/
void LPS22HH_Reset(LPS22HH_Handle_t *Dev){
	uint8_t buffer = (1 << 2);
	Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_CTRL_REG2, &buffer, 1);
}

/**************************************//**************************************
 *@Brief: Reads new pressure data set into LPS22HH into device handle
 *@Params: LPS22HH Device Handle
 *@Return: Dataready flag indication whether new data was retrieved or not.
 *@Precondition: Device is initialized. If in one-shot mode, "StartConversion" function should be called first.
 *@Postcondition: If data is read successfully: LPS22HH StatusReg PA_d bit will be cleared, New data will be
 *............... in the device handle, and the DataReady flag will be reset to 0.
 **************************************//**************************************/
LPS22HH_DataReadyFlag_t LPS22HH_ReadPressure(LPS22HH_Handle_t *Dev){
	uint8_t status;
	uint8_t buffer[3];
	if(Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_STATUS, &status,1) == LPS22HH_Ok){
		if((status & 0x01) == 0x01){ // new data is available
			Dev->DataReadyFlag = LPS22HH_DataReady;
			Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_PRESSURE_OUT_XL, buffer,3);
			Dev->DataReadyFlag = LPS22HH_DataNotReady;
			Dev->Pressure = ConvertPressure(buffer) / 4096.0; //Convert to hPA (1 atm = 1013 hPa)
			return LPS22HH_DataReady;
		} else {
			return LPS22HH_DataNotReady;
		}
	} else {
		_log(log_lps22hh, "Reading Pressure failed. Try to read again.");
			return LPS22HH_DataNotReady;
	}
}

/**************************************//**************************************
 *@Brief: Sets the Pressure threshold register used for IRQs in the LPS22HH device
 *@Params: LPS22HH Device Handle, absolute value of desired pressure threshold
 *@Return: None
 *@Precondition: LPS22HH Device handle should be initialized.
 *@Postcondition: LPS22HH THS_P registers will be modified.
 **************************************//**************************************/
void LPS22HH_SetPressureThresholds(LPS22HH_Handle_t *Dev, uint16_t Pressure){
	Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_THS_P_L, (uint8_t*)&Pressure, 2);
}

/**************************************//**************************************
 *@Brief: Enabled AutoZero mode in the LPS22HH Device
 *@Params: LPS22HH Device Handle.
 *@Return: None
 *@Precondition: LPS22HH Device handle should be initialized.
 *@Postcondition: LPS22HH Device will enter auto-zero mode. See Datasheet for details.
 **************************************//**************************************/
void LPS22HH_AutoZeroSet(LPS22HH_Handle_t *Dev){
	uint8_t buffer;
	if(Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_INTERRUPT_CFG, &buffer, 1) == LPS22HH_Ok){ //Read Reg
		buffer |= (1 << 5); //Set the ONE-SHOT bit (Modify Reg)
		Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_INTERRUPT_CFG, &buffer, 1); //Store Reg
	} else {
		_log(log_lps22hh, "Reading IRQ CFG failed. Enable AutoZero again");
	}
}

/**************************************//**************************************
 *@Brief: Enabled AutoRefP mode in the LPS22HH Device
 *@Params: LPS22HH Device Handle.
 *@Return: None
 *@Precondition: LPS22HH Device handle should be initialized.
 *@Postcondition: LPS22HH Device will enter AutoRefP mode. See Datasheet for details.
 **************************************//**************************************/
void LPS22HH_AutoRefpSet(LPS22HH_Handle_t *Dev){
	uint8_t buffer;
	if(Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_INTERRUPT_CFG, &buffer, 1) == LPS22HH_Ok){ //Read Reg
		buffer |= (1 << 7); //Set the ONE-SHOT bit (Modify Reg)
		Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_INTERRUPT_CFG, &buffer, 1); //Store Reg
	} else {
		_log(log_lps22hh, "Reading IRQ CFG failed. Enable AutoRefp again");
	}
}

/**************************************//**************************************
 *@Brief: Disabled AutoRefp and AutoZero mode
 *@Params: LPS22HH Device Handle
 *@Return: None
 *@Precondition: LPS22HH Device handle should be initialized.
 *@Postcondition: LPS22HH Device will exit AutoRefP and AutoZero mode
 **************************************//**************************************/
void LPS22HH_AutoReset(LPS22HH_Handle_t *Dev){
	uint8_t buffer;
	if(Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_INTERRUPT_CFG, &buffer, 1) == LPS22HH_Ok){ //Read Reg
		buffer |= (1 << 6) | (1 << 4); //Set the ONE-SHOT bit (Modify Reg)
		Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_INTERRUPT_CFG, &buffer, 1); //Store Reg
	} else {
		_log(log_lps22hh, "Reading IRQ CFG failed. Set AutoReset again");
	}
}

/**************************************//**************************************
 *@Brief: Starts a data conversion in one-shot mode.
 *@Params: LPS22HH Device Handle
 *@Return: None
 *@Precondition: LPS22HH Device handle should be initialized and be in one-shot (power down) mode
 *@Postcondition: LPS22HH Device will start capturing a pressure sample.
 **************************************//**************************************/
void LPS22HH_StartConversion(LPS22HH_Handle_t *Dev){
	uint8_t buffer;
	if(Dev->LPS22HH_IO.ReadReg(LPS22HH_REG_CTRL_REG2,&buffer, 1) == LPS22HH_Ok){ //Read Reg
		buffer |= 1; //Set the ONE-SHOT bit (Modify Reg)
		Dev->LPS22HH_IO.WriteReg(LPS22HH_REG_CTRL_REG2,&buffer, 1); //Store Reg
	} else {
		_log(log_lps22hh, "Reading Ctrl Reg 2 failed. Try to StartConversion again");
	}
}

/**************************************//**************************************
 *@Brief: Reads a register from the LPS22HH device
 *@Params: LPS22HH Device Handle, register, data buffer, length
 *@Return: None
 *@Precondition: LPS22HH Device handle should be initialized.
 *@Postcondition: Given LPS22HH Register will be read to and data stored in pdata.
 **************************************//**************************************/
void LPS22HH_ReadReg(LPS22HH_Handle_t *Dev, uint8_t reg, uint8_t *pdata, uint8_t length){
	Dev->LPS22HH_IO.ReadReg(reg, pdata, length);
}

/**************************************//**************************************
 *@Brief: Writes to a register in the LPS22HH device
 *@Params: LPS22HH Device Handle, register, data buffer, length
 *@Return: None
 *@Precondition: LPS22HH Device handle should be initialized.
 *@Postcondition: Given LPS22HH Register will be written to.
 **************************************//**************************************/
void LPS22HH_WriteReg(LPS22HH_Handle_t *Dev, uint8_t reg, uint8_t *pdata, uint8_t length){
	Dev->LPS22HH_IO.WriteReg(reg, pdata, length);
}

/**************************************//**************************************//**************************************
 * Private Function Definitions
 **************************************//**************************************//**************************************/
/**************************************//**************************************
 *@Brief: Converts pressure data registers to a signed 32 bit integer.
 *@Params: pointer to data buffer containing the XL, L, and H registers (in that order)
 *@Return: Pressure data as a signed 32 bit integer. (Pressure in hPa * 4096)
 *@Precondition: None
 *@Postcondition: None
 **************************************//**************************************/
static int32_t ConvertPressure(uint8_t* pdata){
	int8_t sign = (int8_t)pdata[2]; //Read MSB pdata[0] = XL, pdata[1] = L, pdata[2] = H
 	int32_t pressure;
	if (sign < 0){
		pressure = 0xFF000000 | (sign << 16) | pdata[1] << 8 | pdata[0];
	} else {
		pressure = 0x00000000 | (sign << 16) | pdata[1] << 8 | pdata[0];
	}
	return pressure;
}
