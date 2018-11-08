#include "stm32f3xx_hal.h"
#include "usb_device.h"
#include "app.h"
#include "pixelBlaster.h"
#include "console.h"

#include <stdint.h>
#include <stdbool.h>

#define ARM_MATH_CM4
#include "arm_math.h"

int _write(int file, char *outgoing, int len);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_adc4;

extern DAC_HandleTypeDef hdac1;

extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;
extern OPAMP_HandleTypeDef hopamp4;

extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_tim4_up;

extern UART_HandleTypeDef huart1;

extern const uint16_t sample1[];
extern const uint16_t sample2[];
extern const uint16_t sample3[];
extern const uint16_t sample4[];

#define LED_BUFFER_SIZE (40*24*8/2)
uint8_t ledBuffer[LED_BUFFER_SIZE];
PixelBlasterData pb;


int16_t cbuf[4][ADC_BUF_SIZE];

static volatile bool triggerDone = false;
int dmaCndtr;

volatile uint32_t ms;
uint32_t lastHitMs = 0;

volatile enum {
	WAITING, CAPTURING, IGNORENEXT
} triggerMode;

/*
 * Set up timer 3 to trigger all ADCs at the same time.
 * simultaneously capture all channels
 * It also seems like it will be more controllable than continuous mode.
 * DMA buffers won't need too much upkeep, should mostly run w/o cpu
 * Use adc watchdog 1 on each adc to trigger captureEvent()
 *
 *           trigger   +------+ dma   +---------------+
 *           +-------> | adc1 +------>+  circ buffer  |
 *           |         +------+       +---------------+
 *           |         +------+ dma   +---------------+
 *           +-------> | adc2 +------>+  circ buffer  |
 * +------+  |         +------+       +---------------+
 * | tim3 +--+         +------+ dma   +---------------+
 * +------+  +-------> | adc3 +------>+  circ buffer  |
 *           |         +------+       +---------------+
 *           |         +------+ dma   +---------------+
 *           +-------> | adc4 +------>+  circ buffer  |
 *                     +--+---+       +---------------+
 *                        |
 *                        |adc 1-4 watchdogs
 *                        +------------------->  captureEvent()
 *
 * When event fires, make note of current dma CNDTR indicating where the circular buffer is at the trigger start
 * start countdown on tim3 triggers until we have the trailing data captured
 * then stop tim3 to process the buffer
 *
 * TODO is it possible one adc triggered an event and there's nothing on the others?
 * TODO any issue with being "blind" during processing/sending?
 * TODO ws2812: if we need to interrupt at 800khz, thats about every 90 cycles. too much? maybe clock stretch?
 */

void setAdcWd(uint16_t low, uint16_t high) {
	uint32_t tr1 = (high << 16) | low;
	ADC1->TR1 = tr1;
	ADC2->TR1 = tr1;
	ADC3->TR1 = tr1;
	ADC4->TR1 = tr1;
}

void setOpAmpGainAndDac(int gain) {

	//	VM_SEL
	//	10: Resistor feedback output (PGA mode)
	//	11: follower mode
	uint32_t csr = 0;
	switch(gain) {
		case 1:
			csr = 0b11 << OPAMP1_CSR_VMSEL_Pos; //set to follower
			break;
		case 2:
			csr = OPAMP_PGA_GAIN_2 | 0b10 << OPAMP1_CSR_VMSEL_Pos;
			break;
		case 4:
			csr = OPAMP_PGA_GAIN_4 | 0b10 << OPAMP1_CSR_VMSEL_Pos;
			break;
		case 8:
			csr = OPAMP_PGA_GAIN_8 | 0b10 << OPAMP1_CSR_VMSEL_Pos;
			break;
		case 16:
			csr = OPAMP_PGA_GAIN_16 | 0b10 << OPAMP1_CSR_VMSEL_Pos;
			break;
		default:
			HardFault_Handler();
	}

	HAL_DACEx_DualSetValue(&hdac1, DAC_ALIGN_12B_R, 2048/gain, 2048/gain);

	MODIFY_REG(hopamp1.Instance->CSR, OPAMP_CSR_PGGAIN_Msk | OPAMP_CSR_VMSEL_Msk, csr);
	MODIFY_REG(hopamp2.Instance->CSR, OPAMP_CSR_PGGAIN_Msk | OPAMP_CSR_VMSEL_Msk, csr);
	MODIFY_REG(hopamp3.Instance->CSR, OPAMP_CSR_PGGAIN_Msk | OPAMP_CSR_VMSEL_Msk, csr);
	MODIFY_REG(hopamp4.Instance->CSR, OPAMP_CSR_PGGAIN_Msk | OPAMP_CSR_VMSEL_Msk, csr);

}

void setup() {

	pbInit(&pb, ledBuffer, LED_BUFFER_SIZE);
	memset(ledBuffer, 0, LED_BUFFER_SIZE);

//	HAL_DACEx_DualSetValue(&hdac1, DAC_ALIGN_12B_R, 128, 128);
	//gain 2x
//	HAL_DACEx_DualSetValue(&hdac1, DAC_ALIGN_12B_R, 1024, 1024);
	//gain 4x
	HAL_DACEx_DualSetValue(&hdac1, DAC_ALIGN_12B_R, 512, 512);

//	setOpAmpGainAndDac(4);

//	MODIFY_REG(hopamp1.Instance->CSR, OPAMP_CSR_PGGAIN_Msk, OPAMP_PGA_GAIN_2);
//	MODIFY_REG(hopamp2.Instance->CSR, OPAMP_CSR_PGGAIN_Msk, OPAMP_PGA_GAIN_2);
//	MODIFY_REG(hopamp3.Instance->CSR, OPAMP_CSR_PGGAIN_Msk, OPAMP_PGA_GAIN_2);
//	MODIFY_REG(hopamp4.Instance->CSR, OPAMP_CSR_PGGAIN_Msk, OPAMP_PGA_GAIN_2);

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	HAL_Delay(100);

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);

	//TODO try without calibrating, seems off anyway.
	HAL_OPAMP_SelfCalibrate(&hopamp1);
	HAL_OPAMP_SelfCalibrate(&hopamp2);
	HAL_OPAMP_SelfCalibrate(&hopamp3);
	HAL_OPAMP_SelfCalibrate(&hopamp4);

	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);
	HAL_OPAMP_Start(&hopamp4);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &cbuf[0][0], ADC_BUF_SIZE);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *) &cbuf[1][0], ADC_BUF_SIZE);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t *) &cbuf[2][0], ADC_BUF_SIZE);
	HAL_ADC_Start_DMA(&hadc4, (uint32_t *) &cbuf[3][0], ADC_BUF_SIZE);

//	disarmAdcWatchdogs();
	setAdcWd(2020 - 75, 2020 + 75);


	//let things settle on powerup
	HAL_Delay(100);

//	DBGMCU->APB1FZ = 0xffff;
//	DBGMCU->APB2FZ = 0xffff;

	LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);

	LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_EnableIT_UPDATE(TIM2);

	LL_TIM_EnableCounter(TIM3);
	LL_TIM_EnableCounter(TIM4);

	//tim4 is used to spit out pixel data to gpio
	TIM4->DIER |= TIM_DIER_UDE; //Update DMA request enable
	HAL_TIM_Base_Start(&htim4);

}

void disarmAdcWatchdogs(void) {
	CLEAR_BIT(hadc1.Instance->IER, ADC_IER_AWD1IE);
	CLEAR_BIT(hadc2.Instance->IER, ADC_IER_AWD1IE);
	CLEAR_BIT(hadc3.Instance->IER, ADC_IER_AWD1IE);
	CLEAR_BIT(hadc4.Instance->IER, ADC_IER_AWD1IE);
}

void armAdcWatchdogs(void) {
	CLEAR_BIT(ADC1->ISR, ADC_ISR_AWD1);
	CLEAR_BIT(ADC2->ISR, ADC_ISR_AWD1);
	CLEAR_BIT(ADC3->ISR, ADC_ISR_AWD1);
	CLEAR_BIT(ADC4->ISR, ADC_ISR_AWD1);

	SET_BIT(hadc1.Instance->IER, ADC_IER_AWD1IE);
	SET_BIT(hadc2.Instance->IER, ADC_IER_AWD1IE);
	SET_BIT(hadc3.Instance->IER, ADC_IER_AWD1IE);
	SET_BIT(hadc4.Instance->IER, ADC_IER_AWD1IE);
}


uint32_t hexShort(uint16_t v) {
	static const char digits[513] =
			"000102030405060708090A0B0C0D0E0F"
			"101112131415161718191A1B1C1D1E1F"
			"202122232425262728292A2B2C2D2E2F"
			"303132333435363738393A3B3C3D3E3F"
			"404142434445464748494A4B4C4D4E4F"
			"505152535455565758595A5B5C5D5E5F"
			"606162636465666768696A6B6C6D6E6F"
			"707172737475767778797A7B7C7D7E7F"
			"808182838485868788898A8B8C8D8E8F"
			"909192939495969798999A9B9C9D9E9F"
			"A0A1A2A3A4A5A6A7A8A9AAABACADAEAF"
			"B0B1B2B3B4B5B6B7B8B9BABBBCBDBEBF"
			"C0C1C2C3C4C5C6C7C8C9CACBCCCDCECF"
			"D0D1D2D3D4D5D6D7D8D9DADBDCDDDEDF"
			"E0E1E2E3E4E5E6E7E8E9EAEBECEDEEEF"
			"F0F1F2F3F4F5F6F7F8F9FAFBFCFDFEFF";
	union {
		struct {
			uint16_t a,b;
		} p;
		uint32_t word;
	} res;

	res.p.b = *((uint16_t *)&digits[(v & 0xff) << 1]);
	res.p.a = *((uint16_t *)&digits[((v>>8) & 0xff) << 1]);
	return res.word;
}

uint32_t helloTimer;

void loop() {
//	if (ms - helloTimer > 5000) {
//		helloTimer = ms;
//		printf("hi\n");
//		flushConsole();
//	}



#if 0
	memcpy(cbuf[0], sample1 + 1500, ADC_BUF_SIZE * sizeof(q15_t));
	memcpy(cbuf[1], sample2 + 1500, ADC_BUF_SIZE * sizeof(q15_t));
	memcpy(cbuf[2], sample3 + 1500, ADC_BUF_SIZE * sizeof(q15_t));
	memcpy(cbuf[3], sample4 + 1500, ADC_BUF_SIZE * sizeof(q15_t));

	analyzeDelays(cbuf, 0);
	HAL_Delay(1000);

	return;

#endif

	if (HAL_DMA_GetState(&hdma_tim4_up) == HAL_DMA_STATE_READY && pbCheckDone(&pb)) {
		//TODO reset the timer counter or it can cut clocks, this seems to transfer immediately
		htim4.Instance->CNT = 0;
		HAL_DMA_Start_IT(&hdma_tim4_up, (uint32_t) ledBuffer, ((uint32_t)&GPIOC->ODR) + 1, LED_BUFFER_SIZE);
	}

	if (triggerDone) {
		if (ms - lastHitMs > 500) {

			int res = analyzeDelays(cbuf, dmaCndtr);

			flushConsole();
			printf("Raw data:\nch1\tch2\tch3\tch4\n");

			union {
				uint32_t vars[4];
				char bytes[20];
			} buf;

			for (int i = 0; i < ADC_BUF_SIZE; i++) {
				int index = (ADC_BUF_SIZE + i - dmaCndtr + 1) % ADC_BUF_SIZE;
				buf.vars[0] = hexShort(cbuf[0][index]);
				buf.vars[1] = hexShort(cbuf[1][index]);
				buf.vars[2] = hexShort(cbuf[2][index]);
				buf.vars[3] = hexShort(cbuf[3][index]);
				buf.bytes[16] = '\n';
				_write(0, buf.bytes, 17);
			}
			printf("=========\n");
			flushConsole();
			lastHitMs = ms;
		}

		LL_TIM_EnableCounter(TIM3);
		HAL_Delay(2);

		triggerDone = false;
		//TODO figure out why adc watchdog keeps firing immediately after it gets reenabled
		triggerMode = WAITING;
		armAdcWatchdogs();
	}

}

void startEventCapture() {

	disarmAdcWatchdogs();
	triggerMode = CAPTURING;

	LL_TIM_EnableCounter(TIM2);
}

//called from TIM2_IRQHandler
void stopEventCapture() {
	if (triggerMode == CAPTURING) {
		LL_TIM_DisableCounter(TIM3);
		triggerDone = true;
		dmaCndtr = hdma_adc1.Instance->CNDTR; //these should all be the same
	}
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc) {
	startEventCapture();
}

void HAL_SYSTICK_Callback() {
	ms++;
}

