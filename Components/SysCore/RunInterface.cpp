/*
 *  RunInterface.cpp
 *
 *  Created on: Apr 3, 2023
 *      Author: Chris (cjchanx)
 */

#include "../../SoarOS/Drivers/Inc/UARTDriver.hpp"
#include "../../SoarDrivers/DMA/DMATransfer.hpp"
#include "main_avionics.hpp"

#include "RunInterface.hpp"

extern "C" {
void run_interface() { run_main(); }

void cpp_USART2_IRQHandler() {
		Driver::usart2.HandleIRQ_UART();
	}

void cpp_USART3_IRQHandler() {
		Driver::usart3.HandleIRQ_UART();
	}

/* HAL DMA Completion / Error Callbacks -> DMAControl FreeRTOS notifications */
#ifdef HAL_I2C_MODULE_ENABLED
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) { DMAControl::NotifyCompleteFromISR(hi2c, false); }
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) { DMAControl::NotifyCompleteFromISR(hi2c, false); }
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)    { DMAControl::NotifyCompleteFromISR(hi2c, false); }
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)    { DMAControl::NotifyCompleteFromISR(hi2c, false); }
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)        { DMAControl::NotifyCompleteFromISR(hi2c, true);  }
#endif

#ifdef HAL_SPI_MODULE_ENABLED
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)       { DMAControl::NotifyCompleteFromISR(hspi, false); }
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)       { DMAControl::NotifyCompleteFromISR(hspi, false); }
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)     { DMAControl::NotifyCompleteFromISR(hspi, false); }
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)        { DMAControl::NotifyCompleteFromISR(hspi, true);  }
#endif

#ifdef HAL_UART_MODULE_ENABLED
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)    { DMAControl::NotifyCompleteFromISR(huart, false); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)    { DMAControl::NotifyCompleteFromISR(huart, false); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)     { DMAControl::NotifyCompleteFromISR(huart, true);  }
#endif
}
