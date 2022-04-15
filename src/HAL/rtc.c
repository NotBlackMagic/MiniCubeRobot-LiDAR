#include "rtc.h"

/**
  * @brief	This function initializes the RTC clocks and configurations
  * @param	None
  * @return	None
  */
void RTCInit() {
	//Enable the power clock
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_BKP);

	//Enable write access to configure the RTC clock source (to be done once after reset)
	LL_PWR_EnableBkUpAccess();

	//Configure LSE/LSI as RTC clock source
	LL_RCC_LSI_Enable();	//Enable LSI
	while (LL_RCC_LSI_IsReady() != 0x01);		//Wait till LSI is ready

	//Reset backup domain only if LSI is not yet selected as RTC clock source
	if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI) {
		LL_RCC_ForceBackupDomainReset();
		LL_RCC_ReleaseBackupDomainReset();
		LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
	}

	//Enable RTC Clock
	LL_RCC_EnableRTC();

	//Disable RTC registers write protection
	LL_RTC_DisableWriteProtection(RTC);
	while(LL_RTC_IsActiveFlag_RTOF(RTC) != 0x01);	//Wait till RTC is in INIT state

	//Configure RTC
//	LL_RTC_SetAsynchPrescaler(RTC, ((uint32_t)0x9C3F));	//Configure RTC prescaler, LSI = 40kHz
	LL_RTC_SetAsynchPrescaler(RTC, ((uint32_t)0xA053));	//Configure RTC prescaler, LSI = 41044 Hz

	//Configure Interrupts
//	NVIC_SetPriority(RTC_IRQn, 0);
//	NVIC_EnableIRQ(RTC_IRQn);

	//Configure RTC Calendar
	//HOURS*3600 + MINUTES*60 + SECONDS
	uint32_t timeCounter = 0*3600 + 0*60 + 0;
	LL_RTC_TIME_Set(RTC, timeCounter);

	//Enable RTC registers write protection
	LL_RTC_EnableWriteProtection(RTC);
	while(LL_RTC_IsActiveFlag_RTOF(RTC) != 0x01);	//Wait till RTC is in INIT state
}

void RTCGetTime(uint8_t* hour, uint8_t* min, uint8_t* sec) {
	uint32_t timeCounter = LL_RTC_TIME_Get(RTC);
	*hour = (timeCounter/3600);
	*min  = (timeCounter % 3600) / 60;
	*sec  = (timeCounter % 3600) % 60;
}

void RTCSetTime(uint8_t hour, uint8_t min, uint8_t sec) {
	//Disable RTC registers write protection
	LL_RTC_DisableWriteProtection(RTC);
	while(LL_RTC_IsActiveFlag_RTOF(RTC) != 0x01);	//Wait till RTC is in INIT state

	//HOURS*3600 + MINUTES*60 + SECONDS
	uint32_t timeCounter = hour*3600 + min*60 + sec;
	LL_RTC_TIME_Set(RTC, timeCounter);

	//Enable RTC registers write protection
	LL_RTC_EnableWriteProtection(RTC);
	while(LL_RTC_IsActiveFlag_RTOF(RTC) != 0x01);	//Wait till RTC is in INIT state
}
