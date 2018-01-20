#include "stm32f0xx_hal.h"
#include "lights.h"
#include "beeper.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;

uint16_t pwm[9] = {0};
uint16_t current_pwm[9] = {0};
uint8_t fade[3] = {0};

void hsb2rgb(uint16_t index, uint8_t sat, uint8_t bright, uint8_t *color_r, uint8_t *color_g, uint8_t *color_b );

uint16_t counter[3] = {0};


void set_duty(uint8_t N, uint16_t duty)
{
	switch(N)
	{
		case 0 : htim16.Instance->CCR1 = duty; break;
		case 1 : htim2.Instance->CCR4 = duty; break;
		case 2 : htim14.Instance->CCR1 = duty; break;
		case 3 : htim2.Instance->CCR3 = duty; break;
		case 4 : htim2.Instance->CCR2 = duty; break;
		case 5 : htim2.Instance->CCR1 = duty; break;
		case 6 : htim3.Instance->CCR4 = duty; break;
		case 7 : htim3.Instance->CCR1 = duty; break;
		case 8 : htim3.Instance->CCR2 = duty; break;
	}
}

void set_pwm(void)
{
	uint8_t i,j;
	for(i=0;i<3;i++)
	{
		if(fade[i] == 0)
		{
			for(j=i*3;j<i*3+3;j++)
				set_duty(j, pwm[j]);
		}
		else
		{
			counter[i]++;
			if(counter[i] >= fade[i])
			{
				for(j=i*3;j<i*3+3;j++)
				{
					if(current_pwm[j] < pwm[j])
						set_duty(j, current_pwm[j]++);
					else if(current_pwm[j] > pwm[j])
						set_duty(j, current_pwm[j]--);
				}
				counter[i] = 0;
			}
		}
	}
}

void set_color(uint8_t number, uint8_t color, uint8_t intensity, uint8_t fade_time)
{
	uint8_t i;
	uint8_t col[3];
	uint8_t *k = col;
	hsb2rgb(color*3,255,intensity,&col[0],&col[1],&col[2]);
	for(i=number*3;i<number*3+3;i++)
		pwm[i] = *(k++)*4;
	fade[number] = fade_time;
	//beepy(1,2);
}



/******************************************************************************
 * accepts hue, saturation and brightness values and outputs three 8-bit color
 * values in an array (color[])
 *
 * saturation (sat) and brightness (bright) are 8-bit values.
 *
 * hue (index) is a value between 0 and 767. hue values out of range are
 * rendered as 0.
 *
 *****************************************************************************/
void hsb2rgb(uint16_t index, uint8_t sat, uint8_t bright, uint8_t *color_r, uint8_t *color_g, uint8_t *color_b ) 
{
	uint8_t temp[5], n = (index >> 8) % 3;
		uint8_t x = ((((index & 255) * sat) >> 8) * bright) >> 8;
	uint8_t s = ((256 - sat) * bright) >> 8;

		temp[0] = temp[3] =              s;
	temp[1] = temp[4] =          x + s;
		temp[2] =          bright - x    ;
		*color_r  = temp[n + 2];
		*color_g = temp[n + 1];
		*color_b  = temp[n    ];
}


