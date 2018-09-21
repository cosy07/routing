#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"

static volatile uint32_t curTime = 0;

void enable_timer(void);
void TIM2_IRQHandler(void);
void MyTimerInit(void);


void SysTick_Handler();

uint32_t millis();

void delay(uint32_t);
