# LPS22HH_Driver

Simple LPS22HH Pressure Sensor Driver

Files of Interest:

1. LPS22HH.h: Device specific header file - Shouldn't need modification
2. LPS22HH.c: Device specific source file - Shouldn't need modification
3. LPS22HH_Hardware.h: Hardware specific header file - Should not need modification beyond the exported low level driver
4. LPS22HH_Hardware.c: Hardware specific source file - User must implement this file for their board/project needs

To Use:

1. Include HLPS22HH.h
2. Create an LPS22HH_Struct_t with desired user settings.
3. Create an LPS22HH_IO_Drv_t with necessary low level IO functions (I2C/SPI, GPIO Communication functions).
4. Create a LPS22HH_Handle_t
5. Pass the init struct, IO Driver, and device handle to LPS22HH_Init()
6. Functions listed in LPS22HH.h can now be used by passing the initialized device handle as a function arguement
Above example was implemented on an STM32U5 processor (b-u585i-iot02a discovery board)
