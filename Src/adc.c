/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
#include "stdint.h"
//define
#define OFF_VALUE 0
#define FRONT_VALUE 1
#define SIDE_VALUE 2
#define FINISH_CONVERT 3
#define IRSENSOR_OFF 4

#define ADC_CONVERT_DATA_SIZE ((uint32_t)4)

#define TRUE 1
#define FALSE 0

//valuable
sensor_t sen_l;
sensor_t sen_fl;
sensor_t sen_front;
sensor_t sen_fr;
sensor_t sen_r;
int16_t sen_front_reference_f;

uint16_t ADCBuff[ADC_CONVERT_DATA_SIZE];
uint16_t ADCOffData[ADC_CONVERT_DATA_SIZE];
uint16_t ADCOntData[ADC_CONVERT_DATA_SIZE];
int16_t adc_counter;
uint8_t cnt_100ms;
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PC5     ------> ADC1_IN15 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();
  
    /**ADC2 GPIO Configuration    
    PC0     ------> ADC2_IN10
    PC1     ------> ADC2_IN11
    PA2     ------> ADC2_IN2
    PA3     ------> ADC2_IN3 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA2_Stream2;
    hdma_adc2.Init.Channel = DMA_CHANNEL_1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_NORMAL;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PC5     ------> ADC1_IN15 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();
  
    /**ADC2 GPIO Configuration    
    PC0     ------> ADC2_IN10
    PC1     ------> ADC2_IN11
    PA2     ------> ADC2_IN2
    PA3     ------> ADC2_IN3 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void Adc_SetSensorValue(void)
{
  sen_l.reference = 108;//111
  sen_l.threshold = 87;//off80

  sen_front.reference = 230;//220
  sen_front_reference_f = 117;//117
  sen_front.threshold = 102;//109

  sen_r.reference = 151;//162
  sen_r.threshold = 120;//off118//130
}

void update_sensor_data(void)
{

  sen_front.now = (sen_fl.now + sen_fr.now) / 2;

  if (sen_front.now < sen_front.threshold)
  {
    sen_front.is_wall = FALSE;
  }
  else
  {
    sen_front.is_wall = TRUE;
  }

  if (sen_l.now < sen_l.threshold)
  {
    sen_l.is_wall = FALSE;
  }
  else
  {
    sen_l.is_wall = TRUE;
  }

  if (sen_r.now < sen_r.threshold)
  {
    sen_r.is_wall = FALSE;
  }
  else
  {
    sen_r.is_wall = TRUE;
  }
}

void Adc_IrSensorStart(void)
{
  adc_counter = 0;
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADCBuff, ADC_CONVERT_DATA_SIZE);
}

void Adc_IrSensorFinish(void)
{
  adc_counter = IRSENSOR_OFF;
  HAL_GPIO_WritePin(ir_front_GPIO_Port, ir_front_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ir_side_GPIO_Port, ir_side_Pin, GPIO_PIN_RESET);
}

void Adc_CheckConvert(void)
{
  if (adc_counter == FINISH_CONVERT)
  {
    update_sensor_data();
    adc_counter = 0;
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADCBuff, ADC_CONVERT_DATA_SIZE);
  }
}

// DMA の変換式を記�?
void Adc_GetSensor(void)
{
  volatile unsigned char i;
  switch (adc_counter)
  {
  case OFF_VALUE:
    HAL_ADC_Stop_DMA(&hadc2);
    ADCOffData[0] = ADCBuff[0];
    ADCOffData[1] = ADCBuff[1];
    ADCOffData[2] = ADCBuff[2];
    ADCOffData[3] = ADCBuff[3];

    HAL_GPIO_WritePin(ir_front_GPIO_Port, ir_front_Pin, GPIO_PIN_SET);

    for (i = 0; i < 200; i++)
    {
    }

    adc_counter = FRONT_VALUE;

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADCBuff, ADC_CONVERT_DATA_SIZE);
    break;

  case FRONT_VALUE:
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_GPIO_WritePin(ir_front_GPIO_Port, ir_front_Pin, GPIO_PIN_RESET);

    ADCOntData[1] = ADCBuff[1];
    ADCOntData[2] = ADCBuff[2];

    //sen_l.diff = sen_l.now - (ADCOntData[2] - ADCOffData[2]);
    sen_fl.now = ADCOntData[1] - ADCOffData[1];

    //sen_fl.diff = sen_fl.now - (ADCOntData[3] - ADCOffData[3]);
    sen_fr.now = ADCOntData[2] - ADCOffData[2];

    HAL_GPIO_WritePin(ir_side_GPIO_Port, ir_side_Pin, GPIO_PIN_SET);

    for (i = 0; i < 200; i++)
    {
    }

    adc_counter = SIDE_VALUE;

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADCBuff, ADC_CONVERT_DATA_SIZE);
    break;

  case SIDE_VALUE:
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_GPIO_WritePin(ir_side_GPIO_Port, ir_side_Pin, GPIO_PIN_RESET);

    ADCOntData[0] = ADCBuff[0];
    ADCOntData[3] = ADCBuff[3];

    //sen_fr.diff = sen_fr.now - (ADCOntData[0] - ADCOffData[0]);
    sen_l.now = ADCOntData[0] - ADCOffData[0];

    //sen_r.diff = sen_r.now - (ADCOntData[1] - ADCOffData[1]);
    sen_r.now = ADCOntData[3] - ADCOffData[3];

    for (i = 0; i < 200; i++)
    {
    }

    adc_counter = FINISH_CONVERT;
    break;

  default:
    break;
  }
}

float Adc_GetBatt(void)
{
  float batt = 0;
  for (int i = 0; i < 10; i++)
  {
    HAL_ADC_Start(&hadc1); // ad convert start
    while (HAL_ADC_PollForConversion(&hadc1, 50) != HAL_OK)
    {
    } // trans
    batt += HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
  }
  batt = batt / 10.0f / 4095.0f * 3.2f * 3.3f;
  if (batt > 8.0f)
  {
    Gpio_FullColor(BLUE);
  }
  else if (batt > 7.5)
  {
    Gpio_FullColor(YELLOW);
  }
  else if (batt > 7.0)
  {
    Gpio_FullColor(RED);
  }
  else
  {
    while (1)
    {
      Gpio_FullColor(RED);
      HAL_Delay(200);
      Gpio_FullColor(DARK);
      HAL_Delay(200);
    }
  }
  return batt;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
