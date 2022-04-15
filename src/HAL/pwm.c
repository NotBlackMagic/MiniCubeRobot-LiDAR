#include "pwm.h"
#include "gpio.h"
#include "rcc.h"

void PWM1Init() {
	//Init PWM using Timer 1
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

	//Configure GPIOs
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);

	//Configure the NVIC to handle TIM interrupt
	NVIC_SetPriority(TIM1_UP_IRQn, 0);
	NVIC_EnableIRQ(TIM1_UP_IRQn);

	//Configure Timer
	LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV1);
	LL_TIM_SetPrescaler(TIM1, 0);
	LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetAutoReload(TIM1, 640);
	LL_TIM_EnableARRPreload(TIM1);

	//Configure Timer output
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_LOW);
	LL_TIM_OC_SetCompareCH1(TIM1, 320);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);

	//Configure Timer Interrupts
	LL_TIM_EnableIT_UPDATE(TIM1);

	//Enable Timer and PWM Output
	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_GenerateEvent_UPDATE(TIM1);
}

void PWM2Init() {
	//Init PWM using Timer 2
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

	//Configure GPIOs
	LL_GPIO_AF_RemapPartial2_TIM2();
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_PUSHPULL);

	//Configure the NVIC to handle TIM interrupt
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);

	//Configure Timer
	LL_TIM_SetClockDivision(TIM2, LL_TIM_CLOCKDIVISION_DIV1);
	LL_TIM_SetPrescaler(TIM2, 0);
	LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetAutoReload(TIM2, 640);
	LL_TIM_EnableARRPreload(TIM2);

	//Configure Timer output
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_LOW);
	LL_TIM_OC_SetCompareCH4(TIM2, 320);
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH4);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);

	//Configure Timer Interrupts
	LL_TIM_EnableIT_UPDATE(TIM2);

	//Enable Timer and PWM Output
	LL_TIM_EnableAllOutputs(TIM2);
	LL_TIM_EnableCounter(TIM2);
	LL_TIM_GenerateEvent_UPDATE(TIM2);
}

void PWM4Init() {
	//Init PWM using Timer 4
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

	//Configure GPIOs
	//Set PWM TIM4 CH2 (PB7)
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
	//Set PWM TIM4 CH4 (PB9)
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);

	//Configure Timer
	LL_TIM_SetClockDivision(TIM4, LL_TIM_CLOCKDIVISION_DIV1);		//Lower clock to 1/1 = 64MHz/1 = 64MHz
	LL_TIM_SetPrescaler(TIM4, 249);									//Set PWM clock to 1kHz; Fpwm = 64MHz / ((Prescaler + 1) * Compare)
	LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetAutoReload(TIM4, 255);
	LL_TIM_EnableARRPreload(TIM4);

	//Configure Timer outputs
	//Set PWM TIM4 CH4
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetCompareCH2(TIM4, 0);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
//	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
	//Set PWM TIM4 CH4
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetCompareCH4(TIM4, 0);
	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);
//	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);

	//Configure the NVIC to handle TIM interrupt
//	NVIC_SetPriority(TIM4_IRQn, 0);
//	NVIC_EnableIRQ(TIM4_IRQn);

	//Configure Timer Interrupts
//	LL_TIM_EnableIT_UPDATE(TIM4);

	//Enable Timer and PWM Output
//	LL_TIM_EnableAllOutputs(TIM4);
//	LL_TIM_EnableCounter(TIM4);
//	LL_TIM_GenerateEvent_UPDATE(TIM4);
}

void PWM4Enable() {
	//Enable Timer and PWM Output
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);

	LL_TIM_EnableCounter(TIM4);
	LL_TIM_GenerateEvent_UPDATE(TIM4);
}

void PWM4Disable() {
	//Disable Timer and PWM Output
	LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH4);

	LL_TIM_DisableCounter(TIM4);
}

void PWM4Set(uint8_t channel, uint16_t value) {
	switch(channel) {
		case 1:
			LL_TIM_OC_SetCompareCH1(TIM4, value);
			break;
		case 2:
			LL_TIM_OC_SetCompareCH2(TIM4, value);
			break;
		case 3:
			LL_TIM_OC_SetCompareCH3(TIM4, value);
			break;
		case 4:
			LL_TIM_OC_SetCompareCH4(TIM4, value);
			break;
		default:
			break;
	}
}
