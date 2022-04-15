#include "laserScanner.h"

//Laser Scanner Data
RobotLaserScanPacket laserScanPacket;

//Sensor configurations
VL53L5CX_Configuration sensorsConfig[6];

//Inverse cos(x)*cos(y) table: 1/(cos(x)*cos(y))
//https://community.st.com/s/question/0D53W00001NTlTzSAL/for-vl53l5cx-what-does-resultsdatadistancemmzonenum-exactly-mean
static const uint16_t flatToRound[8][8] = {	{ 18482, 17939, 17592, 17422, 17422, 17592, 17939, 18482},
											{ 17939, 17412, 17075, 16911, 16911, 17075, 17412, 17939},
											{ 17592, 17075, 16745, 16583, 16583, 16745, 17075, 17592},
											{ 17422, 16911, 16583, 16424, 16424, 16583, 16911, 17422},
											{ 17422, 16911, 16583, 16424, 16424, 16583, 16911, 17422},
											{ 17592, 17075, 16745, 16583, 16583, 16745, 17075, 17592},
											{ 17939, 17412, 17075, 16911, 16911, 17075, 17412, 17939},
											{ 18482, 17939, 17592, 17422, 17422, 17592, 17939, 18482} };

static const uint16_t laserOffset = 23;


void LaserScannerInit() {
	uint8_t status = 0;

	//Init Laser Scan data struct
	laserScanPacket.angleStart = -12868;	//Start: -pi (-180 deg), straight left
	laserScanPacket.angleStop = 12868;	//End: pi (-180 deg), straight right
	laserScanPacket.angleStep = 402;		//Increment: pi/32 (-5.625 deg) comes from: 45 deg FoV with 8 SPADs
	laserScanPacket.scanTime = 100;		//Scan time: 100ms (10Hz)
	laserScanPacket.rangeMin = 10;		//Minimum range considered valid: 10mm (1 cm)
	laserScanPacket.rangeMax = 4000;		//Maximum range considered valid: 4000mm (4 m)
	laserScanPacket.rangeCountV = 8;
	laserScanPacket.rangeCountH = 64;

	//Init VL53L5 Sensor struct with platform information
	sensorsConfig[2].platform.status = 0x00;
	sensorsConfig[2].platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
	sensorsConfig[2].platform.gpio_lp = GPIO_OUT_LP_FL;
	sensorsConfig[2].platform.gpio_rst = GPIO_OUT_RST_FL;

	sensorsConfig[3].platform.status = 0x00;
	sensorsConfig[3].platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
	sensorsConfig[3].platform.gpio_lp = GPIO_OUT_LP_FC;
	sensorsConfig[3].platform.gpio_rst = GPIO_OUT_RST_FC;

	sensorsConfig[4].platform.status = 0x00;
	sensorsConfig[4].platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
	sensorsConfig[4].platform.gpio_lp = GPIO_OUT_LP_FR;
	sensorsConfig[4].platform.gpio_rst = GPIO_OUT_RST_FR;

	sensorsConfig[5].platform.status = 0x00;
	sensorsConfig[5].platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
	sensorsConfig[5].platform.gpio_lp = GPIO_OUT_LP_BR;
	sensorsConfig[5].platform.gpio_rst = GPIO_OUT_RST_BR;

	sensorsConfig[0].platform.status = 0x00;
	sensorsConfig[0].platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
	sensorsConfig[0].platform.gpio_lp = GPIO_OUT_LP_BC;
	sensorsConfig[0].platform.gpio_rst = GPIO_OUT_RST_BC;

	sensorsConfig[1].platform.status = 0x00;
	sensorsConfig[1].platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
	sensorsConfig[1].platform.gpio_lp = GPIO_OUT_LP_BL;
	sensorsConfig[1].platform.gpio_rst = GPIO_OUT_RST_BL;

	//------------------------------------------------------//
	//Power up LIDAR sensors and configure them with new unique addresses
	GPIOWrite(GPIO_EN_VCC_S, 0x00);		//Disable LDO for LIDAR Sensors
	Delay(100);
	GPIOWrite(GPIO_EN_VCC_S, 0x01);		//Enable LDO for LIDAR Sensors
	Delay(100);

	uint8_t i;
	for(i = 0; i < 6; i++) {
		//Enable current sensor interface, out of low power
		GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x01);
		Delay(100);

		//Reset sensor by toggling PINs
		//Reset_Sensor(&(sensorsConfig[1].platform));

		//Set a new I2C address
		status = vl53l5cx_set_i2c_address(&sensorsConfig[i], (0x20 + (i << 1)));
		if(status != 0x00) {
			//Error occurred while setting new I2C Address
			sensorsConfig[i].platform.status = 0x01;

			//Disable current sensor interface, into low power mode
			Delay(10);	//Small delay after I2C transaction finished and before disabling I2C (LP) of sensor
			GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x00);
			continue;
		}

		//Check if there is a VL53L5CX sensor connected (Optional)
		uint8_t isAlive = 0;
		status = vl53l5cx_is_alive(&sensorsConfig[i], &isAlive);
		if(status != 0x00 || isAlive == 0x00) {
			//Error occurred while checking for sensor on specified I2C address
			sensorsConfig[i].platform.status = 0x02;

			//Disable current sensor interface, into low power mode
			Delay(10);	//Small delay after I2C transaction finished and before disabling I2C (LP) of sensor
			GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x00);
			continue;
		}

		//Disable current sensor interface, into low power mode
		Delay(10);	//Small delay after I2C transaction finished and before disabling I2C (LP) of sensor
		GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x00);
	}

	//------------------------------------------------------//
	//Initialize LIDAR sensors
	for(i = 0; i < 6; i++) {
		if(sensorsConfig[i].platform.status == 0x00) {
			//Enable current sensor interface, out of low power
			GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x01);
			Delay(100);

			//Init VL53L5CX sensor (Mandatory)
			status = vl53l5cx_init(&sensorsConfig[i]);
			if(status != 0x00) {
				//Error occurred while initializing the sensor, writing firmware to the sensor RAM
				sensorsConfig[i].platform.status = 0x03;

				GPIOWrite(GPIO_OUT_LED2, 0x01);

				//Disable current sensor interface, into low power mode
				Delay(10);	//Small delay after I2C transaction finished and before disabling I2C (LP) of sensor
				GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x00);
				continue;
			}

			//------------------------------------------------------//
			//Configure Sensor
			//Set resolution: 8x8
			status = vl53l5cx_set_resolution(&sensorsConfig[i], VL53L5CX_RESOLUTION_8X8);
			if(status != 0x00) {
				//Error occurred while configuring resolution
				sensorsConfig[i].platform.status = 0x04;

				GPIOWrite(GPIO_OUT_LED2, 0x01);

				//Disable current sensor interface, into low power mode
				Delay(10);	//Small delay after I2C transaction finished and before disabling I2C (LP) of sensor
				GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x00);
				continue;
			}

			//Set ranging frequency, in 4x4 max is 60Hz and in 8x8 max is 15Hz: 2Hz
			status = vl53l5cx_set_ranging_frequency_hz(&sensorsConfig[i], 2);
			if(status != 0x00) {
				//Error occurred while configuring range frequency
				sensorsConfig[i].platform.status = 0x05;

				GPIOWrite(GPIO_OUT_LED2, 0x01);

				//Disable current sensor interface, into low power mode
				Delay(10);	//Small delay after I2C transaction finished and before disabling I2C (LP) of sensor
				GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x00);
				continue;
			}

			//Set ranging mode: Autonomous (decreased power consumption, VCSEL on only for integration time)
			status = vl53l5cx_set_ranging_mode(&sensorsConfig[i], VL53L5CX_RANGING_MODE_AUTONOMOUS);
			if(status != 0x00) {
				//Error occurred while configuring range mode
				sensorsConfig[i].platform.status = 0x06;

				GPIOWrite(GPIO_OUT_LED2, 0x01);

				//Disable current sensor interface, into low power mode
				Delay(10);	//Small delay after I2C transaction finished and before disabling I2C (LP) of sensor
				GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x00);
				continue;
			}

			//Set integration time (autonomous mode): 5ms (1/rang_freq > (integ_time + 1) * 4)
			status = vl53l5cx_set_integration_time_ms(&sensorsConfig[i], 5);
			if(status != 0x00) {
				//Error occurred while reading integration time
				sensorsConfig[i].platform.status = 0x08;

				GPIOWrite(GPIO_OUT_LED2, 0x01);

				//Disable current sensor interface, into low power mode
				Delay(10);	//Small delay after I2C transaction finished and before disabling I2C (LP) of sensor
				GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x00);
				continue;
			}

			//Set target order: Strongest
			status = vl53l5cx_set_target_order(&sensorsConfig[i], VL53L5CX_TARGET_ORDER_STRONGEST);
			if(status != 0x00) {
				//Error occurred while configuring target order
				sensorsConfig[i].platform.status = 0x07;

				GPIOWrite(GPIO_OUT_LED2, 0x01);

				//Disable current sensor interface, into low power mode
				Delay(10);	//Small delay after I2C transaction finished and before disabling I2C (LP) of sensor
				GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x00);
				continue;
			}

//			//Get integration time
//			uint32_t integration_time_ms;
//			status = vl53l5cx_get_integration_time_ms(&sensorsConfig[i], &integration_time_ms);
//			if(status != 0x00) {
//				//Error occurred while reading integration time
//				sensorsConfig[i].platform.status = 0x08;
//
//				GPIOWrite(GPIO_OUT_LED2, 0x01);
//
//				//Disable current sensor interface, into low power mode
//				Delay(10);	//Small delay after I2C transaction finished and before disabling I2C (LP) of sensor
//				GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x00);
//				continue;
//			}

			sensorsConfig[i].platform.status = 0x00;
		}
	}

	//------------------------------------------------------//
	//Start ranging loop
	for(i = 0; i < 6; i++) {
		if(sensorsConfig[i].platform.status == 0x00) {
			status = vl53l5cx_start_ranging(&sensorsConfig[i]);
			if(status != 0x00) {
				//Error occurred while starting ranging
				sensorsConfig[i].platform.status = 0x09;

				GPIOWrite(GPIO_OUT_LED2, 0x01);

				//Disable current sensor interface, into low power mode
				Delay(10);	//Small delay after I2C transaction finished and before disabling I2C (LP) of sensor
				GPIOWrite(sensorsConfig[i].platform.gpio_lp, 0x00);
				continue;
			}
		}
	}
}

uint8_t status, toggle;
VL53L5CX_ResultsData results;		//Results data from VL53L5CX
void LaserScannerUpdate() {
	uint8_t i;
	for(i = 2; i < 4; i++) {
		if(sensorsConfig[i].platform.status == 0x00 && sensorsConfig[i].platform.dataReady == 0x01) {
			//Get new ranging data
			status = vl53l5cx_get_ranging_data(&sensorsConfig[i], &results);
			if(status != 0x00) {
				//Error occurred while reading new data struct
//				sensorsConfig[i].platform.status = 0x0A;
				continue;
			}

			//Update laser scan data struct
			uint8_t sensorIndex = 0;			//Sensor zone mapping index
			uint8_t hOffset = 8 + i*8;			//Horizontal index in the laser scan data

			uint8_t h, v;
			for(v = 0; v < 8; v++) {
				for(h = 0; h < 8; h++) {
					//Convert Sensor Zone Mapping to laser scan data
					//TOP LEFT						  TOP RIGHT
					//-----------------------------------------
					//|  7 |  6 |  5 |  4 |  3 |  2 |  1 |  0 |
					//-----------------------------------------
					//| 15 | 14 | 13 | 12 | 11 | 10 |  9 |  8 |
					//-----------------------------------------
					//| 23 | 22 | 21 | 20 | 19 | 18 | 17 | 16 |
					//-----------------------------------------
					//| 31 | 30 | 29 | 28 | 27 | 26 | 25 | 24 |
					//-----------------------------------------
					//| 39 | 38 | 37 | 36 | 35 | 34 | 33 | 32 |
					//-----------------------------------------
					//| 47 | 46 | 45 | 44 | 43 | 42 | 41 | 40 |
					//-----------------------------------------
					//| 55 | 54 | 53 | 52 | 51 | 50 | 49 | 48 |
					//-----------------------------------------
					//| 63 | 62 | 61 | 60 | 59 | 58 | 57 | 56 |
					//-----------------------------------------
					//BOTTOM LEFT				   BOTTOM RIGHT
					sensorIndex = 8*(7 - v) + h;
					if(results.target_status[sensorIndex] == VL53L5CX_TARGET_STATUS_RANGE_VALID) {
						laserScanPacket.ranges[v][hOffset + h] = ((int32_t)results.distance_mm[sensorIndex] * flatToRound[h][v]) >> 14;
						laserScanPacket.ranges[v][hOffset + h] += laserOffset;
					}
					else {
						laserScanPacket.ranges[v][hOffset + h] = laserScanPacket.rangeMax;
					}
				}
			}

			//Toggle GPIO to signal new ranging data received (should toggle at 10 Hz)
			GPIOWrite(GPIO_OUT_LED0, toggle);
			if(toggle == 0x00) {
				toggle = 0x01;
			}
			else {
				toggle = 0x00;
			}

			sensorsConfig[i].platform.dataReady = 0;
		}
	}

	//Stop ranging loop
//	status = vl53l5cx_stop_ranging(&sensorsConfig[1]);
}

void EXTI2Callback() {
	//INT set by Sensor Back Left
	sensorsConfig[1].platform.dataReady = 1;
}

void EXTI3Callback() {
	//INT set by Sensor Front Center
	sensorsConfig[3].platform.dataReady = 1;
}

void EXTI5Callback() {
	//INT set by Sensor Back Center
	sensorsConfig[0].platform.dataReady = 1;
}

void EXTI7Callback() {
	//INT set by Sensor Back Right
	sensorsConfig[5].platform.dataReady = 1;
}

void EXTI8Callback() {
	//INT set by Sensor Front Left
	sensorsConfig[2].platform.dataReady = 1;
}

void EXTI10Callback() {
	//INT set by Sensor Front Right
	sensorsConfig[4].platform.dataReady = 1;
}
