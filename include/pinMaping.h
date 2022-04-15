#ifndef PINMAPING_H_
#define PINMAPING_H_

#ifdef __cplusplus
extern "C" {
#endif

//Input Pin Mapping
#define GPIO_IN_INT_FL						24		//Input INT (Interrupt pin) of LIDAR Sensor (VL53L5CX) front left
#define GPIO_IN_INT_FC						19		//Input INT of LIDAR Sensor front center
#define GPIO_IN_INT_FR						42		//Input INT of LIDAR Sensor front right
#define GPIO_IN_INT_BR						39		//Input INT of LIDAR Sensor back right
#define GPIO_IN_INT_BC						37		//Input INT of LIDAR Sensor back center
#define GPIO_IN_INT_BL						34		//Input INT of LIDAR Sensor back left

//Output Pin Mapping
#define GPIO_OUT_LP_FL						21		//Output LP (Low Power) of LIDAR Sensor (VL53L5CX) front left
#define GPIO_OUT_LP_FC						50		//Output LP of LIDAR Sensor front center
#define GPIO_OUT_LP_FR						15		//Output LP of LIDAR Sensor front right
#define GPIO_OUT_LP_BR						38		//Output LP of LIDAR Sensor back right
#define GPIO_OUT_LP_BC						36		//Output LP of LIDAR Sensor back center
#define GPIO_OUT_LP_BL						33		//Output LP of LIDAR Sensor back left

#define GPIO_OUT_RST_FL						25		//Output RST (I2C Interface Reset) of LIDAR Sensor (VL53L5CX) front left
#define GPIO_OUT_RST_FC						20		//Output RST of LIDAR Sensor front center
#define GPIO_OUT_RST_FR						43		//Output RST of LIDAR Sensor front right
#define GPIO_OUT_RST_BR						40		//Output RST of LIDAR Sensor back right
#define GPIO_OUT_RST_BC						16		//Output RST of LIDAR Sensor back center
#define GPIO_OUT_RST_BL						35		//Output RST of LIDAR Sensor back left

#define GPIO_OUT_IO0						29		//Output GPIO0 of backbone interface
#define GPIO_OUT_IO1						28		//Output GPIO1 of backbone interface
#define GPIO_OUT_IO2						18		//Output GPIO2 of backbone interface
#define GPIO_OUT_IO3						17		//Output GPIO3 of backbone interface

#define GPIO_OUT_LED0						31		//Output LED0
#define GPIO_OUT_LED1						32		//Output LED1
#define GPIO_OUT_LED2						45		//Output LED2

#define GPIO_EN_VCC_S						30		//Output enable VCC to LIDAR Sensors

//ADC Pin Mapping
#define	GPIO_ADC_I_VCC						0		//ADC Input Current MCU
#define	GPIO_ADC_I_VCC_S					1		//ADC Input Current LIDAR Sensor (VL53L5CX)

//ADC Channel Mapping
#define ADC_CH_I_VCC						0		//ADC Channel Current MCU
#define ADC_CH_I_VCC_S						1		//ADC Channel Current LIDAR Sensor (VL53L5CX)

#ifdef __cplusplus
}
#endif

#endif /* PINMAPING_H_ */
