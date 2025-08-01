#include "JOYSTICK.h"

const JoyStick_CfgType JoyStick_CfgParam[JOYSTICK_UNITS] =
{
    {
	    GPIOA,
		GPIOA,
		GPIO_PIN_6,
		GPIO_PIN_7,
		ADC1,
		ADC_CHANNEL_1,
		ADC_CHANNEL_2
	},
	{
		    GPIOA,
			GPIOA,
			GPIO_PIN_2,
			GPIO_PIN_7,
			ADC1,
			ADC_CHANNEL_3,
			ADC_CHANNEL_4
		}
};

