#include "stm32f3xx_hal.h"
#include "usb_device.h"
#include <stdbool.h>
#include "usbd_cdc_if.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel2;


#define USE_USB_CONSOLE 1

//#define CONSOLE_BUFFER 1024
//
//static char buffer[CONSOLE_BUFFER];
//

//static volatile bool serialComDone;
///*
// * The following two functions are used to support the stdio output functions.
// */
//int _write(int file, char *outgoing, int len) {
//  HAL_StatusTypeDef status;
//
//  serialComDone = false;
//  status = HAL_UART_Transmit_DMA(&huart1, (uint8_t *) outgoing, len);
//  if (HAL_OK != status) {
//    serialComDone = true;
//    len = -1;
//  }
//  while (serialComDone == false) {
//    /* Hang around until the DMA complete interrupt fires */
//  }
//
//  return (len);
//}
//
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
//  serialComDone = true;
//}

#if USE_USB_CONSOLE

int _write(int file, char *outgoing, int len) {
	uint8_t res;

	do {
		res = CDC_Transmit_FS(outgoing, len);
	} while (res == USBD_BUSY);

	return len;
}

#else

int _write(int file, char *outgoing, int len) {
	HAL_UART_Transmit(&huart1, outgoing, len, 1000);

//	HAL_UART_Transmit_DMA(&huart1, (uint8_t *) outgoing, len);
//	HAL_DMA_PollForTransfer(&hdma_usart1_tx, HAL_DMA_FULL_TRANSFER , 1000);

	return len;
}

#endif


