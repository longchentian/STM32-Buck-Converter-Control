/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5 ;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED); 
//	HAL_ADC_Start(&hadc1);
  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL3.PLL3M = 5;
    PeriphClkInitStruct.PLL3.PLL3N = 30;
    PeriphClkInitStruct.PLL3.PLL3P = 2;
    PeriphClkInitStruct.PLL3.PLL3Q = 2;
    PeriphClkInitStruct.PLL3.PLL3R = 2;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC1 clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA6     ------> ADC1_INP3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA6     ------> ADC1_INP3
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

uint16_t get_ad(void)
{
	uint16_t temp;
	HAL_ADC_Start(&hadc1);                    //����ADC����ת��
	HAL_ADC_PollForConversion(&hadc1, 50);    //�ȴ�ADCת�����
	temp = HAL_ADC_GetValue(&hadc1); //��ȡADCת������
	return temp;
}

#define M  7
/***��λֵ�˲���***/ 
unsigned int filter2( void )
{
    unsigned int value_buf[M];
    unsigned int count, i, j, temp;
    for( count = 0; count < M; count++ )
    {
        value_buf[count] = get_ad();
    }
    for( j = 0; j < M - 1; j++ )
    {
        for( i = 0; i < M - j - 1; i++ )
        {
            if( value_buf[i] > value_buf[i + 1] )
            {
                temp = value_buf[i];
                value_buf[i] = value_buf[i + 1];
                value_buf[i + 1] = temp;
            }
        }
    }
    return value_buf[( M - 1 ) / 2];
}


/*
������
    Rֵ�̶���QֵԽ�󣬴���Խ���β���ֵ��Qֵ����󣬴���ֻ�ò���ֵ��
             QֵԽС������Խ����ģ��Ԥ��ֵ��QֵΪ0������ֻ��ģ��Ԥ��ֵ��
Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵

R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��
*/
//����һ
float  KalmanFilter(float  inData)
//float  KalmanFilter(float  inData,float prevData)
{
    static float prevData = 0;                                 //��һ������
    static float p = 10, q = 0.009, r = 0.009, kGain = 0;      // q ������� r ������Ӧ�ٶ�
    p = p + q;
    kGain = p / ( p + r );                                      //���㿨��������
    inData = prevData + ( kGain * ( inData - prevData ) );      //���㱾���˲�����ֵ
    p = ( 1 - kGain ) * p;                                      //���²�������
    prevData = inData;
    return inData;                                             //���ع���ֵ
}
/*
	temp=get_ad();//�����ɼ�
	man=(float) temp;
	man=KalmanFilter(man);//�������˲�
*/
float Kalman2(void)
{
	float inData;
	unsigned int count, temp;
	
	float prevData = 0;                                 //��һ������
	float p = 10, q = 0.009, r = 0.009, kGain = 0;      // q ������� r ������Ӧ�ٶ�
	
    for( count = 0; count < M; count++ )
    {
		temp=get_ad();//�����ɼ�
		inData=(float) temp;
		p = p + q;
		kGain = p / ( p + r );                                      //���㿨��������
		inData = prevData + ( kGain * ( inData - prevData ) );      //���㱾���˲�����ֵ
		p = ( 1 - kGain ) * p;                                      //���²�������
		prevData = inData;	
    }
	return inData;
}

	

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
