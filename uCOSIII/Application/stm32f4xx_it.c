/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "Android.h"
#include "AHRS.H"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;
extern SD_HandleTypeDef hsd;

void DMA1_Stream1_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

void DMA1_Stream3_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

void TIM6_DAC_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim6);
}
void SDIO_IRQHandler(void) {
	HAL_SD_IRQHandler(&hsd);
}
void DMA2_Stream0_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

void DMA2_Stream3_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_sdio_rx);
}

void DMA2_Stream5_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_spi1_tx);
}
void DMA2_Stream6_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_sdio_tx);
}
void TIM2_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim2);
}

void TIM3_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim3);
}
void TIM4_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim4);
}