/**
	******************************************************************************
	* File Name          : stm32f4xx_hal_msp.c
	* Description        : This file provides code for the MSP Initialization
	*                      and de-Initialization codes.
	******************************************************************************
	*
	* COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_hal.h"

extern DMA_HandleTypeDef hdma_usart3_rx;

extern DMA_HandleTypeDef hdma_usart3_tx;

extern DMA_HandleTypeDef hdma_spi1_rx;

extern DMA_HandleTypeDef hdma_spi1_tx;

extern DMA_HandleTypeDef hdma_sdio_rx;

extern DMA_HandleTypeDef hdma_sdio_tx;
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
	* Initializes the Global MSP.
	*/
// void HAL_MspInit(void)
// {
//   /* USER CODE BEGIN MspInit 0 */
// 
//   /* USER CODE END MspInit 0 */
// 
//   HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
// 
//   /* System interrupt init*/
//   /* SysTick_IRQn interrupt configuration */
//   HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
// 
//   /* USER CODE BEGIN MspInit 1 */
// 
//   /* USER CODE END MspInit 1 */
// }

void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc) {

	if (hcrc->Instance == CRC) {
		/* USER CODE BEGIN CRC_MspInit 0 */

		/* USER CODE END CRC_MspInit 0 */
		/* Peripheral clock enable */
		__CRC_CLK_ENABLE();
		/* USER CODE BEGIN CRC_MspInit 1 */

		/* USER CODE END CRC_MspInit 1 */
	}

}

void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc) {

	if (hcrc->Instance == CRC) {
		/* USER CODE BEGIN CRC_MspDeInit 0 */

		/* USER CODE END CRC_MspDeInit 0 */
		/* Peripheral clock disable */
		__CRC_CLK_DISABLE();
	}
	/* USER CODE BEGIN CRC_MspDeInit 1 */

	/* USER CODE END CRC_MspDeInit 1 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hspi->Instance == SPI1) {
		/* USER CODE BEGIN SPI1_MspInit 0 */

		/* USER CODE END SPI1_MspInit 0 */
		/* Peripheral clock enable */
		__SPI1_CLK_ENABLE();

		/**SPI1 GPIO Configuration
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		hdma_spi1_rx.Instance = DMA2_Stream0;
		hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
		hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_spi1_rx.Init.Mode = DMA_NORMAL;
		hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_spi1_rx);

		__HAL_LINKDMA(hspi, hdmarx, hdma_spi1_rx);

		hdma_spi1_tx.Instance = DMA2_Stream5;
		hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
		hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_spi1_tx.Init.Mode = DMA_NORMAL;
		hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_spi1_tx);

		__HAL_LINKDMA(hspi, hdmatx, hdma_spi1_tx);

		/* USER CODE END SPI1_MspInit 1 */
	}

}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {

	if (hspi->Instance == SPI1) {
		/* USER CODE BEGIN SPI1_MspDeInit 0 */

		/* USER CODE END SPI1_MspDeInit 0 */
		/* Peripheral clock disable */
		__SPI1_CLK_DISABLE();

		/**SPI1 GPIO Configuration
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

	}
	/* USER CODE BEGIN SPI1_MspDeInit 1 */

	/* USER CODE END SPI1_MspDeInit 1 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {

	if (htim_base->Instance == TIM6) {
		/* USER CODE BEGIN TIM6_MspInit 0 */

		/* USER CODE END TIM6_MspInit 0 */
		/* Peripheral clock enable */
		__TIM6_CLK_ENABLE();
		/* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		/* USER CODE BEGIN TIM6_MspInit 1 */

		/* USER CODE END TIM6_MspInit 1 */
	}

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {

	if (htim_base->Instance == TIM6) {
		/* USER CODE BEGIN TIM6_MspDeInit 0 */

		/* USER CODE END TIM6_MspDeInit 0 */
		/* Peripheral clock disable */
		__TIM6_CLK_DISABLE();

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);

	}
	/* USER CODE BEGIN TIM6_MspDeInit 1 */

	/* USER CODE END TIM6_MspDeInit 1 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (huart->Instance == USART3) {
		/* USER CODE BEGIN USART3_MspInit 0 */

		/* USER CODE END USART3_MspInit 0 */
		/* Peripheral clock enable */
		__USART3_CLK_ENABLE();

		/**USART3 GPIO Configuration
		PB10     ------> USART3_TX
		PB11     ------> USART3_RX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral DMA init*/

		hdma_usart3_rx.Instance = DMA1_Stream1;
		hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart3_rx.Init.Mode = DMA_NORMAL;
		hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_usart3_rx);

		__HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);

		hdma_usart3_tx.Instance = DMA1_Stream3;
		hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart3_tx.Init.Mode = DMA_NORMAL;
		hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_usart3_tx);

		__HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

		/* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
		/* USER CODE BEGIN USART3_MspInit 1 */

		/* USER CODE END USART3_MspInit 1 */
	}

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {

	if (huart->Instance == USART3) {
		/* USER CODE BEGIN USART3_MspDeInit 0 */

		/* USER CODE END USART3_MspDeInit 0 */
		/* Peripheral clock disable */
		__USART3_CLK_DISABLE();

		/**USART3 GPIO Configuration
		PB10     ------> USART3_TX
		PB11     ------> USART3_RX
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);

		/* Peripheral DMA DeInit*/
		HAL_DMA_DeInit(huart->hdmarx);
		HAL_DMA_DeInit(huart->hdmatx);

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(USART3_IRQn);

	}
	/* USER CODE BEGIN USART3_MspDeInit 1 */

	/* USER CODE END USART3_MspDeInit 1 */

}
void HAL_SD_MspInit(SD_HandleTypeDef* hsd) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hsd->Instance == SDIO) {
		/* USER CODE BEGIN SDIO_MspInit 0 */

		/* USER CODE END SDIO_MspInit 0 */
		/* Peripheral clock enable */
		__SDIO_CLK_ENABLE();

		/**SDIO GPIO Configuration
		PC8     ------> SDIO_D0
		PC9     ------> SDIO_D1
		PC10     ------> SDIO_D2
		PC11     ------> SDIO_D3
		PC12     ------> SDIO_CK
		PD2     ------> SDIO_CMD
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
			| GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* Peripheral DMA init*/

		hdma_sdio_rx.Instance = DMA2_Stream3;
		hdma_sdio_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_sdio_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_sdio_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_sdio_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_sdio_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_sdio_rx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_sdio_rx.Init.Mode = DMA_NORMAL;
		hdma_sdio_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_sdio_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		hdma_sdio_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_sdio_rx.Init.MemBurst = DMA_MBURST_INC4;
		hdma_sdio_rx.Init.PeriphBurst = DMA_PBURST_INC4;
		HAL_DMA_Init(&hdma_sdio_rx);

		__HAL_LINKDMA(hsd, hdmarx, hdma_sdio_rx);

		hdma_sdio_tx.Instance = DMA2_Stream6;
		hdma_sdio_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_sdio_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_sdio_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_sdio_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_sdio_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_sdio_tx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_sdio_tx.Init.Mode = DMA_NORMAL;
		hdma_sdio_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_sdio_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		hdma_sdio_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_sdio_tx.Init.MemBurst = DMA_MBURST_INC4;
		hdma_sdio_tx.Init.PeriphBurst = DMA_PBURST_INC4;
		HAL_DMA_Init(&hdma_sdio_tx);

		__HAL_LINKDMA(hsd, hdmatx, hdma_sdio_tx);

		/* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(SDIO_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SDIO_IRQn);
	}

}

void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd) {

	if (hsd->Instance == SDIO) {
		/* USER CODE BEGIN SDIO_MspDeInit 0 */

		/* USER CODE END SDIO_MspDeInit 0 */
		/* Peripheral clock disable */
		__SDIO_CLK_DISABLE();

		/**SDIO GPIO Configuration
		PC8     ------> SDIO_D0
		PC9     ------> SDIO_D1
		PC10     ------> SDIO_D2
		PC11     ------> SDIO_D3
		PC12     ------> SDIO_CK
		PD2     ------> SDIO_CMD
		*/
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
										| GPIO_PIN_12);

		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

		/* Peripheral DMA DeInit*/
		HAL_DMA_DeInit(hsd->hdmarx);
		HAL_DMA_DeInit(hsd->hdmatx);

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(SDIO_IRQn);

	}
	/* USER CODE BEGIN SDIO_MspDeInit 1 */

	/* USER CODE END SDIO_MspDeInit 1 */

}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (htim_ic->Instance == TIM2) {
		/* USER CODE BEGIN TIM2_MspInit 0 */

		/* USER CODE END TIM2_MspInit 0 */
		/* Peripheral clock enable */
		__TIM2_CLK_ENABLE();

		/**TIM2 GPIO Configuration
		PB3     ------> TIM2_CH2
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
		/* USER CODE BEGIN TIM2_MspInit 1 */

		/* USER CODE END TIM2_MspInit 1 */
	}
	else if (htim_ic->Instance == TIM3) {
		/* USER CODE BEGIN TIM3_MspInit 0 */

		/* USER CODE END TIM3_MspInit 0 */
		/* Peripheral clock enable */
		__TIM3_CLK_ENABLE();

		/**TIM3 GPIO Configuration
		PB4     ------> TIM3_CH1
		PB5     ------> TIM3_CH2
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
		/* USER CODE BEGIN TIM3_MspInit 1 */

		/* USER CODE END TIM3_MspInit 1 */
	}
	else if (htim_ic->Instance == TIM4) {
		/* USER CODE BEGIN TIM4_MspInit 0 */

		/* USER CODE END TIM4_MspInit 0 */
		/* Peripheral clock enable */
		__TIM4_CLK_ENABLE();

		/**TIM4 GPIO Configuration
		PB6     ------> TIM4_CH1
		PB7     ------> TIM4_CH2
		PB8     ------> TIM4_CH3
		PB9     ------> TIM4_CH4
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
		/* USER CODE BEGIN TIM4_MspInit 1 */

		/* USER CODE END TIM4_MspInit 1 */
	}

}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* htim_ic) {

	if (htim_ic->Instance == TIM2) {
		/* USER CODE BEGIN TIM2_MspDeInit 0 */

		/* USER CODE END TIM2_MspDeInit 0 */
		/* Peripheral clock disable */
		__TIM2_CLK_DISABLE();

		/**TIM2 GPIO Configuration
		PB3     ------> TIM2_CH2
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(TIM2_IRQn);

		/* USER CODE BEGIN TIM2_MspDeInit 1 */

		/* USER CODE END TIM2_MspDeInit 1 */
	}
	else if (htim_ic->Instance == TIM3) {
		/* USER CODE BEGIN TIM3_MspDeInit 0 */

		/* USER CODE END TIM3_MspDeInit 0 */
		/* Peripheral clock disable */
		__TIM3_CLK_DISABLE();

		/**TIM3 GPIO Configuration
		PB4     ------> TIM3_CH1
		PB5     ------> TIM3_CH2
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4 | GPIO_PIN_5);

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(TIM3_IRQn);

		/* USER CODE BEGIN TIM3_MspDeInit 1 */

		/* USER CODE END TIM3_MspDeInit 1 */
	}
	else if (htim_ic->Instance == TIM4) {
		/* USER CODE BEGIN TIM4_MspDeInit 0 */

		/* USER CODE END TIM4_MspDeInit 0 */
		/* Peripheral clock disable */
		__TIM4_CLK_DISABLE();

		/**TIM4 GPIO Configuration
		PB6     ------> TIM4_CH1
		PB7     ------> TIM4_CH2
		PB8     ------> TIM4_CH3
		PB9     ------> TIM4_CH4
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(TIM4_IRQn);

		/* USER CODE BEGIN TIM4_MspDeInit 1 */

		/* USER CODE END TIM4_MspDeInit 1 */
	}

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (htim_pwm->Instance == TIM1) {
		/* USER CODE BEGIN TIM1_MspInit 0 */

		/* USER CODE END TIM1_MspInit 0 */
		/* Peripheral clock enable */
		__TIM1_CLK_ENABLE();

		/**TIM1 GPIO Configuration
		PA8     ------> TIM1_CH1
		PA9     ------> TIM1_CH2
		PA10     ------> TIM1_CH3
		PA11     ------> TIM1_CH4
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE BEGIN TIM1_MspInit 1 */

		/* USER CODE END TIM1_MspInit 1 */
	}else if(htim_pwm->Instance==TIM5)
        {
          /* USER CODE BEGIN TIM5_MspInit 0 */
          
          /* USER CODE END TIM5_MspInit 0 */
          /* Peripheral clock enable */
          __TIM5_CLK_ENABLE();
          
          /**TIM5 GPIO Configuration    
          PA0-WKUP     ------> TIM5_CH1
          PA1     ------> TIM5_CH2 
          */
          GPIO_InitStruct.Pin = GPIO_PIN_1;
          GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
          GPIO_InitStruct.Pull = GPIO_NOPULL;
          GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
          GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
          HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
          
          /* USER CODE BEGIN TIM5_MspInit 1 */
          
          /* USER CODE END TIM5_MspInit 1 */
        }
        
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm) {

	if (htim_pwm->Instance == TIM1) {
		/* Peripheral clock disable */
		__TIM1_CLK_DISABLE();

		/**TIM1 GPIO Configuration
		PA8     ------> TIM1_CH1
		PA9     ------> TIM1_CH2
		PA10     ------> TIM1_CH3
		PA11     ------> TIM1_CH4
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11);
	}else   if(htim_pwm->Instance==TIM5)
        {
          /* USER CODE BEGIN TIM5_MspDeInit 0 */
          
          /* USER CODE END TIM5_MspDeInit 0 */
          /* Peripheral clock disable */
          __TIM5_CLK_DISABLE();
          
          /**TIM5 GPIO Configuration    
          PA0-WKUP     ------> TIM5_CH1
          PA1     ------> TIM5_CH2 
          */
          HAL_GPIO_DeInit(GPIOA,GPIO_PIN_1);
          
        }
}