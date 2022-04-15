#ifndef DRIVER_VD6283_REGS_H_
#define DRIVER_VD6283_REGS_H_

#include <stdint.h>

//I2C Device Address 8 bit format
#define VD6283_I2C_ADD					0x40

//Device Identification (Who am I)
#define VD6283_ID						0x70
#define VD6283_REVISION					0xBD

#define VD6283_DEVICE_ID				0x00	//Device identification register
#define VD6283_REVISION_ID				0x01	//Silicon revision identification register
#define VD6283_INT_CTRL					0x02	//Interrupt control register
#define VD6283_ALS_CTRL					0x03	//ALS control register
#define VD6283_ALS_PERIOD				0x04	//ALS period register
#define VD6283_ALS_CH1_DATA_H			0x06	//ALS channel 1 data register [23:16]
#define VD6283_ALS_CH1_DATA_M			0x07	//ALS channel 1 data register [15:8]
#define VD6283_ALS_CH1_DATA_L			0x08	//ALS channel 1 data register [7:0]
#define VD6283_ALS_CH2_DATA_H			0x0A	//ALS channel 2 data register [23:16]
#define VD6283_ALS_CH2_DATA_M			0x0B	//ALS channel 2 data register [15:8]
#define VD6283_ALS_CH2_DATA_L			0x0C	//ALS channel 2 data register [7:0]
#define VD6283_ALS_CH3_DATA_H			0x0E	//ALS channel 3 data register [23:16]
#define VD6283_ALS_CH3_DATA_M			0x0F	//ALS channel 3 data register [15:8]
#define VD6283_ALS_CH3_DATA_L			0x10	//ALS channel 3 data register [7:0]
#define VD6283_ALS_CH4_DATA_H			0x12	//ALS channel 4 data register [23:16]
#define VD6283_ALS_CH4_DATA_M			0x13	//ALS channel 4 data register [15:8]
#define VD6283_ALS_CH4_DATA_L			0x14	//ALS channel 4 data register [7:0]
#define VD6283_ALS_CH5_DATA_H			0x16	//ALS channel 5 data register [23:16]
#define VD6283_ALS_CH5_DATA_M			0x17	//ALS channel 5 data register [15:8]
#define VD6283_ALS_CH5_DATA_L			0x18	//ALS channel 5 data register [7:0]
#define VD6283_ALS_CH6_DATA_H			0x1A	//ALS channel 6 data register [23:16]
#define VD6283_ALS_CH6_DATA_M			0x1B	//ALS channel 6 data register [15:8]
#define VD6283_ALS_CH6_DATA_L			0x1C	//ALS channel 6 data register [7:0]
#define VD6283_ALS_EXPOSURE_M			0x1D	//ALS exposure register [9:8]
#define VD6283_ALS_EXPOSURE_L			0x1E	//ALS exposure register [7:0]
#define VD6283_ALS_GAIN_CH1				0x25	//ALS gain channel 1 register
#define VD6283_ALS_GAIN_CH2				0x26	//ALS gain channel 2 register
#define VD6283_ALS_GAIN_CH3				0x27	//ALS gain channel 3 register
#define VD6283_ALS_GAIN_CH4				0x28	//ALS gain channel 4 register
#define VD6283_ALS_GAIN_CH5				0x29	//ALS gain channel 5 register
#define VD6283_ALS_GAIN_CH6				0x2A	//ALS gain channel 6 register
#define VD6283_CH6_EN					0x2D	//Channel 6 enable register
#define VD6283_ALS_CH_EN				0x2E	//ALS channel enable register
#define VD6283_AC_MODE_CTRL				0x31	//AC mode control register
#define VD6283_PEDESTAL_VALUE			0x32	//Pedestal value register
#define VD6283_SDA_DRV_CFG				0x3C	//SDA driver configuration register
#define VD6283_GPIO1_DRV_CFG			0x41	//GPIO1 driver configuration register

#endif /* DRIVER_VD6283_REGS_H_ */
