#include "myTimer.h"

void enable_timer(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef timerInitStructure;

	/* if MCU frequency 48 MHz, prescaler of value 48 will make 1us counter steps
	timerInitStructure.TIM_Prescaler = 48;

	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	how often you'll get irq
	timerInitStructure.TIM_Period = 0xFFFF; // will get irq each 65535us on TIMx->CNT overflow
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);

	//NVIC_InitTypeDef NVIC_InitStruct;
	//NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	/*for more accuracy irq priority should be higher, but now its default*/
	//NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
	//NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	//TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	//NVIC_Init(&NVIC_InitStruct);
	//TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
}

void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		//curTime +=65535;
	}
}
void MyTimerInit(void) {
	SysTick_Config(SystemCoreClock / 1000);
}

void SysTick_Handler()
{
	curTime++;
}

uint32_t millis()
{
	return curTime;
}

void delay(uint32_t ms)
{
	uint32_t start = millis();
	while (millis() - start < ms);
}
