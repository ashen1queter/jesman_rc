#define HAL_ADC_MODULE_ENABLED

#include "stm32f4xx_hal.h"

#define JOYSTICK_UNITS  2

typedef struct
{
    GPIO_TypeDef * JoyStick_xGPIO;
    GPIO_TypeDef * JoyStick_yGPIO;
    uint16_t       JoyStick_xPIN;
    uint16_t       JoyStick_yPIN;
    ADC_TypeDef*   ADC_Instance;
    uint32_t       ADCx_CH;
    uint32_t       ADCy_CH;
}JoyStick_CfgType;

void JoyStick_Init(uint16_t JoyStick_Instance);
void JoyStick_Read(uint16_t JoyStick_Instance, uint16_t* JoyStick_XY);
void JoyStick_Calibrate();
