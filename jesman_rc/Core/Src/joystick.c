#include "joystick.h"

#include "joystick_cfg.h"

static ADC_HandleTypeDef hadc[JOYSTICK_UNITS] = {0};
static ADC_ChannelConfTypeDef sConfig = {0};
static uint8_t calibrated = 0;

void JoyStick_Init(uint8_t JoyStick_Instances)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();

	for(uint8_t i = 0; i < JoyStick_Instances; i++){
		GPIO_InitStruct.Pin = JoyStick_CfgParam[i].JoyStick_xPIN;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(JoyStick_CfgParam[i].JoyStick_xGPIO, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = JoyStick_CfgParam[i].JoyStick_yPIN;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(JoyStick_CfgParam[i].JoyStick_yGPIO, &GPIO_InitStruct);

		hadc[i].Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
		hadc[i].Init.Resolution = ADC_RESOLUTION_12B;
		hadc[i].Init.ScanConvMode = ENABLE;
		hadc[i].Init.ContinuousConvMode = ENABLE;
		hadc[i].Init.DiscontinuousConvMode = DISABLE;
		hadc[i].Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		hadc[i].Init.ExternalTrigConv = ADC_SOFTWARE_START;
		hadc[i].Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc[i].Init.NbrOfConversion = 4;
		hadc[i].Init.DMAContinuousRequests = ENABLE;
		hadc[i].Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		if (HAL_ADC_Init(&hadc1) != HAL_OK)
			{
				Error_Handler();
			 }

		sConfig.Channel = JoyStick_CfgParam[i].ADCx_CH;
		sConfig.Rank = i;
		sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}

		sConfig.Channel = JoyStick_CfgParam[i].ADCy_CH;
		sConfig.Rank = i+2;
		sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
	}
}


void JoyStick_Calibrate()
{
	uint32_t AD_RES;
	uint32_t ADCx_MAX[2]= 0;
	uint32_t ADCy_MAX[2]= 0;
	uint32_t ADCx_CEN[2]= 0;
	uint32_t ADCy_CEN[2]= 0;
	uint32_t ADCx_MIN[2]= 0;
	uint32_t ADCy_MIN[2]= 0;

	for(uint8_t i = 0; i < JOYSTICK_UNITS; i++)
	{
	sprintf(CALIB_MSG, "Calibrate joystick %d/r/n", i+1);
	HAL_UART_Transmit(&huart1, CALIB_MSG, sizeof(CALIB_MSG), 100);
	HAL_Delay(100);

	sprintf(CALIB_MSG, "Keep the joystick at the center");
	HAL_UART_Transmit(&huart1, CALIB_MSG, sizeof(CALIB_MSG), 100);
	HAL_Delay(100);
	for(uint8_t i = 0; i < 100; i++)
	{
	sConfig.Channel = JoyStick_CfgParam[i].ADCx_CH;
    HAL_ADC_ConfigChannel(&hadc[i], &sConfig);
	HAL_ADC_Start(&hadc[i]);
	HAL_ADC_PollForConversion(&hadc[i], 1);
	AD_RES = HAL_ADC_GetValue(&hadc[i]);
	ADCx_CEN[i] += AD_RES; //Counting

	sConfig.Channel = JoyStick_CfgParam[i].ADCy_CH;
    HAL_ADC_ConfigChannel(&hadc[i], &sConfig);
	HAL_ADC_Start(&hadc[i]);
	HAL_ADC_PollForConversion(&hadc[i], 1);
	AD_RES = HAL_ADC_GetValue(&hadc[i]);
	ADCy_CEN[i] += AD_RES; //Counting
	}

	ADCx_CEN[i] /= 100; //Averaging
	ADCy_CEN[i] /= 100; //Averaging

	sprintf(CALIB_MSG, "Keep the joystick at the top");
	HAL_UART_Transmit(&huart1, CALIB_MSG, sizeof(CALIB_MSG), 100);
	HAL_Delay(100);
	for(uint8_t i = 0; i < 100; i++)
	{
	sConfig.Channel = JoyStick_CfgParam[i].ADCx_CH;
    HAL_ADC_ConfigChannel(&hadc[i], &sConfig);
	HAL_ADC_Start(&hadc[i]);
	HAL_ADC_PollForConversion(&hadc[i], 1);
	AD_RES = HAL_ADC_GetValue(&hadc[i]);
	ADCx_MAX[i] += AD_RES; //Counting

	sConfig.Channel = JoyStick_CfgParam[i].ADCy_CH;
    HAL_ADC_ConfigChannel(&hadc[i], &sConfig);
	HAL_ADC_Start(&hadc[i]);
	HAL_ADC_PollForConversion(&hadc[i], 1);
	AD_RES = HAL_ADC_GetValue(&hadc[i]);
	ADCy_MAX[i] += AD_RES; //Counting

	ADCx_MAX[i] /= 100; //Averaging
	ADCy_MAX[i] /= 100; //Averaging

	sprintf(CALIB_MSG, "Keep the joystick at the bottom");
	HAL_UART_Transmit(&huart1, CALIB_MSG, sizeof(CALIB_MSG), 100);
	HAL_Delay(100);
	for(uint8_t i = 0; i < 100; i++)
	{
	sConfig.Channel = JoyStick_CfgParam[i].ADCx_CH;
    HAL_ADC_ConfigChannel(&hadc[i], &sConfig);
	HAL_ADC_Start(&hadc[i]);
	HAL_ADC_PollForConversion(&hadc[i], 1);
	AD_RES = HAL_ADC_GetValue(&hadc[i]);
	ADCx_MIN += AD_RES; //Counting

	sConfig.Channel = JoyStick_CfgParam[i].ADCy_CH;
    HAL_ADC_ConfigChannel(&hadc[i], &sConfig);
	HAL_ADC_Start(&hadc[i]);
	HAL_ADC_PollForConversion(&hadc[i], 1);
	AD_RES = HAL_ADC_GetValue(&hadc[i]);
	JoyStick_XY[i][1] = AD_RES;
	ADCy_MIN += AD_RES; //Counting

	ADCx_MIN /= 100; //Averaging
	ADCy_MIN /= 100; //Averaging
	}

	if(ADCx_MAX[i] < ADCx_MIN[i]){
		uint32_t val = ADCx_MAX[i];
		ADCx_MAX[i] = ADCx_MIN[i];
		ADCx_MIN[i] = val;
	  }
	if(ADCy_MAX[i] < ADCy_MIN[i]){
		uint32_t val = ADCy_MAX[i];
	    ADCy_MAX[i] = ADCy_MIN[i];
	    ADCy_MIN[i] = val;
	  }
}


void JoyStick_Read(uint8_t JoyStick_Instances, uint16_t* JoyStick_XY[])
{
	uint32_t AD_RES;

	for(uint8_t i = 0; i < JoyStick_Instances; i++){
		sConfig.Channel = JoyStick_CfgParam[i].ADCx_CH;
	    HAL_ADC_ConfigChannel(&hadc[i], &sConfig);
		HAL_ADC_Start(&hadc[i]);
		HAL_ADC_PollForConversion(&hadc[i], 1);
		AD_RES = HAL_ADC_GetValue(&hadc[i]);
		JoyStick_XY[i][0] = AD_RES;

		sConfig.Channel = JoyStick_CfgParam[i].ADCy_CH;
	    HAL_ADC_ConfigChannel(&hadc[i], &sConfig);
		HAL_ADC_Start(&hadc[i]);
		HAL_ADC_PollForConversion(&hadc[i], 1);
		AD_RES = HAL_ADC_GetValue(&hadc[i]);
		JoyStick_XY[i][1] = AD_RES;
	}
}
