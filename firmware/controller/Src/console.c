#include "stm32f3xx_hal.h"
#include "usb_device.h"
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "assert.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel2;


#define USE_USB_CONSOLE 1
#define USE_BUFFER 1


#if USE_BUFFER
#define BUFFER_MAX 64
static uint8_t buffer[BUFFER_MAX];
static volatile int bufferPos = 0;
#endif

#if USE_USB_CONSOLE


void flushConsole() {
#if USE_BUFFER
	uint8_t res;
	assert(bufferPos <= BUFFER_MAX);

	if (bufferPos > 0) {
		do {
			res = CDC_Transmit_FS(buffer, bufferPos);
		} while (res == USBD_BUSY);
	}
	bufferPos = 0;
#endif
}

int _write(int file, char *outgoing, int len) {
	uint8_t res;

#if USE_BUFFER

	int remain = len;
	while (remain > 0) {

		if (remain > BUFFER_MAX - bufferPos) {
			flushConsole();
		}

		int toCopy = remain > BUFFER_MAX - bufferPos ? BUFFER_MAX - bufferPos : remain;
		toCopy = 1;

		memcpy(buffer + bufferPos, outgoing, toCopy);
		bufferPos += toCopy;
		remain -= toCopy;
		outgoing += toCopy;
	}

#else

	do {
		res = CDC_Transmit_FS(outgoing, len);
	} while (res == USBD_BUSY);
#endif

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


