#include "timer.h"

/**
  * @brief	This function initializes the TIM2, 10kHz clock for ADC conversion
  * @param	None
  * @return	None
  */
void TIM2Init() {
	//Enable bus clocks
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	//Configure the Timer: TIM3 is connected to APB1 Timer clocks which has the APB1 clock = 64MHz
	LL_TIM_SetClockDivision(TIM3, LL_TIM_CLOCKDIVISION_DIV1);
	LL_TIM_SetPrescaler(TIM3, 6399);							//Set Clock to 10Hz (0.1s); Fpwm = 64MHz / ((Prescaler + 1) * Compare)
	LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetAutoReload(TIM3, 1000);
	LL_TIM_EnableARRPreload(TIM3);

	//Configure the NVIC to handle TIM interrupt
	NVIC_SetPriority(TIM3_IRQn, 0);
	NVIC_EnableIRQ(TIM3_IRQn);
	//Configure Timer Interrupts
	LL_TIM_EnableIT_UPDATE(TIM3);

	//This is needed to update the prescaler
	LL_TIM_GenerateEvent_UPDATE(TIM3);

	//Enable Timer
	LL_TIM_EnableCounter(TIM3);
}

/**
  * @brief	This function initializes the TIM3, 1Hz counter
  * @param	None
  * @return	None
  */
void TIM3Init() {
	//Enable bus clocks
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	//Configure the Timer: TIM3 is connected to APB1 Timer clocks which has the APB1 clock = 64MHz
	LL_TIM_SetClockDivision(TIM3, LL_TIM_CLOCKDIVISION_DIV1);
	LL_TIM_SetPrescaler(TIM3, 6399);							//Set Clock to 10Hz (0.1s); Fpwm = 64MHz / ((Prescaler + 1) * Compare)
	LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetAutoReload(TIM3, 1000);
	LL_TIM_EnableARRPreload(TIM3);

	//Configure the NVIC to handle TIM interrupt
	NVIC_SetPriority(TIM3_IRQn, 0);
	NVIC_EnableIRQ(TIM3_IRQn);
	//Configure Timer Interrupts
	LL_TIM_EnableIT_UPDATE(TIM3);

	//This is needed to update the prescaler
	LL_TIM_GenerateEvent_UPDATE(TIM3);

	//Enable Timer
	LL_TIM_EnableCounter(TIM3);
}

__attribute__((weak)) void TIM3UpdateCallback() {}

/**
  * @brief	This function is the Handler for TIM3
  * @param	None
  * @return	None
  */
void TIM3_IRQHandler(void) {
	TIM3UpdateCallback();

	LL_TIM_ClearFlag_UPDATE(TIM3);
}
