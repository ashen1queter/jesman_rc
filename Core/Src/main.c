/**
 * Sources:
 * https://uwarg-docs.atlassian.net/wiki/spaces/ZP/pages/2238283817/SBUS+Protocol
 * https://github.com/Reefwing-Software/Reefwing-SBUS
 */

/**
 * @todo Optimization techniques:
 * https://stackoverflow.com/questions/1778538/how-many-gcc-optimization-levels-are-there
 * https://www.disk91.com/2020/technology/programming/code-optimization-with-stm32-cube-ide/
 * https://stackoverflow.com/questions/14492436/g-optimization-beyond-o3-ofast
 * https://community.st.com/t5/stm32cubeide-mcus/how-do-i-change-code-optimization/td-p/271208
 * https://community.st.com/t5/stm32cubeide-mcus/how-do-i-change-code-optimization/td-p/271208
 *
 * @todo Baremetal
 *
 * @todo Only 2 things: Roll and Pitch are covered for now, idk how to implement 3 axes (Roll, Pitch and Throttle) as HOTAS does, will read on it
 */
#pragma GCC push_options
#pragma GCC optimize ("Og")

#include "main.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart1;

char MSG[32];
//char MSG2[32];
uint8_t calib_button_state = 0;
uint16_t joystick_arr[2] = {0};
uint16_t joystick_arr_[1][2] = {0};
uint32_t joystick_arr_calib[1][2] = {0};
uint32_t joystick_arr_calib_[1][3][2] = {0};
uint16_t channels[16] = {0};
uint8_t packet[25] = {0};
int addr = 0x08040000;
uint16_t actual_val[3] = {2048, 4095 , 0};
uint16_t errors[2] = {0};

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_USART1_UART_Init(void);


uint16_t map(uint16_t value, uint16_t fromLow, uint16_t fromHigh, uint16_t toLow, uint16_t toHigh){
	return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

/**
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
*/

/**
 * @brief Maps values from 0-4095 to 192-1792(suited range for SBUS)
 */
void mapping_adc(){
	joystick_arr_[0][0] = map(joystick_arr[0], 0, 4095, 192, 1792)+errors[0];
	joystick_arr_[0][1] = map(joystick_arr[1], 4095, 0, 192, 1792)+errors[1]; //When joystick is pulled up-value is 0 and when pulled down-value is 4095, hence its reversed
}

/**
 * @brief Mapping adc values (x and y of joystick) to channels, packed in 11 bits
 */
void channels_(){
	channels[0] = (joystick_arr_[0][0]& 0x07FF); //Roll
	channels[1] = (joystick_arr_[0][1]& 0x07FF); //Pitch
}


/**
void sbus_channel(){
    channels[0]  = (uint16_t) ((_payload[0]    |_payload[1] <<8)                     & 0x07FF);
    channels[1]  = (uint16_t) ((_payload[1]>>3 |_payload[2] <<5)                     & 0x07FF);
    channels[2]  = (uint16_t) ((_payload[2]>>6 |_payload[3] <<2 |_payload[4]<<10)    & 0x07FF);
    channels[3]  = (uint16_t) ((_payload[4]>>1 |_payload[5] <<7)                     & 0x07FF);
    channels[4]  = (uint16_t) ((_payload[5]>>4 |_payload[6] <<4)                     & 0x07FF);
    channels[5]  = (uint16_t) ((_payload[6]>>7 |_payload[7] <<1 |_payload[8]<<9)     & 0x07FF);
    channels[6]  = (uint16_t) ((_payload[8]>>2 |_payload[9] <<6)                     & 0x07FF);
    channels[7]  = (uint16_t) ((_payload[9]>>5 |_payload[10]<<3)                     & 0x07FF);
    channels[8]  = (uint16_t) ((_payload[11]   |_payload[12]<<8)                     & 0x07FF);
    channels[9]  = (uint16_t) ((_payload[12]>>3|_payload[13]<<5)                     & 0x07FF);
    channels[10] = (uint16_t) ((_payload[13]>>6|_payload[14]<<2 |_payload[15]<<10)   & 0x07FF);
    channels[11] = (uint16_t) ((_payload[15]>>1|_payload[16]<<7)                     & 0x07FF);
    channels[12] = (uint16_t) ((_payload[16]>>4|_payload[17]<<4)                     & 0x07FF);
    channels[13] = (uint16_t) ((_payload[17]>>7|_payload[18]<<1 |_payload[19]<<9)    & 0x07FF);
    channels[14] = (uint16_t) ((_payload[19]>>2|_payload[20]<<6)                     & 0x07FF);
    channels[15] = (uint16_t) ((_payload[20]>>5|_payload[21]<<3)                     & 0x07FF);
}
*/

// Refer to this
void sbus_packet() {
    memset(packet, 0, sizeof(packet));
    packet[0] = 0x0F;
    packet[1] = (uint8_t) ((channels[0] & 0x07FF));
    packet[2] = (uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((channels[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((channels[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((channels[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((channels[8] & 0x07FF));
    packet[13] = (uint8_t) ((channels[8] & 0x07FF)>>8 | (channels[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((channels[9] & 0x07FF)>>5 | (channels[10] & 0x07FF)<<6);
    packet[15] = (uint8_t) ((channels[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((channels[11] & 0x07FF)>>7 | (channels[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((channels[12] & 0x07FF)>>4 | (channels[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((channels[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((channels[13] & 0x07FF)>>9 | (channels[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((channels[14] & 0x07FF)>>6 | (channels[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((channels[15] & 0x07FF)>>3);
    packet[23] = 0x00;
    packet[24] = 0x00;
}

/**
 * @brief Write the error offset to flash so that if calibration is choosen not to be done after another power cycle, then the error offset stored in flash can be used
 */
void flash_write(){
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(FLASH_SECTOR_6,FLASH_VOLTAGE_RANGE_3);
	for(uint8_t i = 0; i < 2; i++, addr++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, errors[i]);
		}
	HAL_FLASH_Lock();
}

/**
 * @brief Read the error offset stored flash
 */
void flash_read(void) {
    for (uint8_t i = 0; i < 2; i++, addr++) {
    	errors[i] = *(volatile uint16_t*)(addr);
    }
}

/**
 * @ brief From the values (center, high and low) obtained during calibration
 * I have decided to subtract the actual values (2048, 4095, 0) for each x and y and then take average and get the offset
 */
void error_(void){
	for(uint8_t i = 0; i < 3; i++){
		errors[0] += actual_val[i] - joystick_arr_calib_[0][i][0];
		errors[1] += actual_val[i] - joystick_arr_calib_[0][i][1];
	}

	errors[0] /= 3;
	errors[1] /= 3;
	//errorx2 /= 3;
	//errory2 /= 3;
}

/**
 * @brief Calibration of the joystick
 * Center, low and high values are stored
 */
void joystick_calib(void) {
    char CALIB_MSG[32];
    const char* prompts[] = {
        "Calibrate joysticks\r\n",
        "Keep the joysticks at the center\r\n",
        "Keep the joysticks at the top\r\n",
        "Keep the joysticks at the bottom\r\n"
    };

    for (uint8_t pos = 0; pos < 3; ++pos) {
        snprintf(CALIB_MSG, sizeof(CALIB_MSG), "%s", prompts[pos]);
        HAL_UART_Transmit(&huart1, (uint8_t*)CALIB_MSG, strlen(CALIB_MSG), 100);
        HAL_Delay(100);

        memset(joystick_arr_calib, 0, sizeof(joystick_arr_calib));

        for (uint8_t i = 0; i < 100; i++) {
            joystick_arr_calib[0][0] += joystick_arr_[0][0];
            joystick_arr_calib[0][1] += joystick_arr_[0][1];
            //joystick_arr_calib[1][0] += joystick_arr_[1][0];
            //joystick_arr_calib[1][1] += joystick_arr_[1][1];
        }

        joystick_arr_calib_[0][pos][0] = joystick_arr_calib[0][0] / 100;
        joystick_arr_calib_[0][pos][1] = joystick_arr_calib[0][1] / 100;
        //joystick_arr_calib_[1][pos][0] = joystick_arr_calib[1][0] / 100;
        //joystick_arr_calib_[1][pos][1] = joystick_arr_calib[1][1] / 100;
    }
}

/**
 * @brief Checks the state of a button, if pressed begins calibration of the joystick and calibration can only be done once a power cycle since:
 * calib_button_state determines if the button has been pressed once and doesn't allow the function to execute again as it gets negated in the condition to 0
 */
void check_calib_button(void) {
    if (HAL_GPIO_ReadPin(CALIB_BUTTON_GPIO_Port, CALIB_BUTTON_Pin) == GPIO_PIN_SET && !calib_button_state) {
        joystick_calib();
        flash_write();
        flash_read();
        calib_button_state = 1;
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();

    //ADC through DMA begins here
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)joystick_arr, sizeof(joystick_arr) / sizeof(joystick_arr[0]));
    while (1) {
		//Check if button to begin calibration is pressed or no, initially set to 0 since we want one calibration after each power cycle but not mandatory
    	if(!calib_button_state){
        check_calib_button();
    	}

    	mapping_adc(); //Maps to sbus range
    	channels_(); //ADC values into channels
    	sbus_packet(); //Packed for transmission

        //snprintf(MSG, sizeof(MSG), "[X: %d, Y: %d]\r\n", joystick_arr_[0][0]+errors[0], joystick_arr_[0][1]+errors[1]);
        //snprintf(MSG2, sizeof(MSG2), "X2: %d, Y2: %d\r\n", joystick_arr_[1][0]+errorx2, joystick_arr_[1][1]+errory2);

    	/**
    	 * @todo Not sure but I think we need to trasmit after every 'x' us, will check on this
    	 */
        HAL_UART_Transmit(&huart1, (uint8_t*)packet, sizeof(packet)/sizeof(packet[0]), 100);
        HAL_Delay(100);
        //HAL_UART_Transmit(&huart1, (uint8_t*)MSG2, strlen(MSG2), 100);
        //HAL_Delay(100);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : CALIB_BUTTON_Pin */
  GPIO_InitStruct.Pin = CALIB_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(CALIB_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

#pragma GCC pop_options
