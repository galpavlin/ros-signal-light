#include "stm32f0xx_hal.h"
#include "beeper.h"

uint8_t beep_mode = 0;
uint16_t beep_duration = 0; // x10 it bigcasa

void set_beeper(uint8_t state)
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,state);
}

void update_beeper(void)
{
	if(beep_duration == 0)
	{
		set_beeper(0);
		return;
	}
	else
		beep_duration--;
	
	switch(beep_mode)
	{
		case 0 : set_beeper(1); break;
		case 1 : (beep_duration%10 >= 5) ? set_beeper(1) :  set_beeper(0); break;
		case 2 : (beep_duration%50 >= 25) ? set_beeper(1) :  set_beeper(0); break;
		case 3 : (beep_duration%100 >= 50) ? set_beeper(1) :  set_beeper(0); break;
	}
}

void beepy(uint8_t mode, uint8_t duration)
{
	beep_mode = mode;
	beep_duration = duration*10;
}

uint8_t get_beepmode(void)
{
	return beep_mode;
}

uint8_t get_beeptime(void)
{
	return beep_duration/10;
}
