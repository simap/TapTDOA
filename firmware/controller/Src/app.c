#include "stm32f3xx_hal.h"
#include "usb_device.h"
#include "app.h"
#include "pixelBlaster.h"

#include <stdint.h>
#include <stdbool.h>

#define ARM_MATH_CM4
#include "arm_math.h"

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


#define BP_NUM_TAPS              32
//low pass around 8k
const q15_t fir1[] = {
		11, 29, 63, 123, 220, 362, 551, 783, 1052, 1344, 1641,
		1923, 2169, 2361, 2482, 2524, 2482, 2361, 2169, 1923, 1641, 1344, 1052,
		783, 551, 362, 220, 123, 63, 29, 11, 0 };


#define ADC_BUF_SIZE (1024*3)
int16_t cbuf[4][ADC_BUF_SIZE];
q15_t tmpBuf[ADC_BUF_SIZE];
q15_t tmpBuf2[ADC_BUF_SIZE];
uint32_t peakIndex[4];

#define BLOCK_SIZE            64
q15_t firState[BLOCK_SIZE + BP_NUM_TAPS];

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = ADC_BUF_SIZE / BLOCK_SIZE;

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
	uint32_t csr;
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

int findNextPeak(int dir, q15_t tmpBuf[], int start, int length, int * foundIndex, q15_t * foundValue) {
	for (int i = start; i >= 0 && i < length; i += dir) {

	}
}


void analyzeDelays() {

	volatile uint32_t timer = 0;
	volatile uint32_t timer2 = 0;
	volatile uint32_t timer3 = 0;
	q15_t avg;
	int channel;
	timer = ms;
	//filter out the DC
	for (channel = 0; channel < 4; channel++) {

		int startIndex = ADC_BUF_SIZE - dmaCndtr;
		//copy, align to tmp
		memcpy(&tmpBuf[0], &cbuf[channel][startIndex], (ADC_BUF_SIZE - startIndex) * sizeof(q15_t));

		if (startIndex != 0) {
			memcpy(&tmpBuf[ADC_BUF_SIZE - startIndex], &cbuf[channel][0], startIndex * sizeof(q15_t));
		}

		//remove dc. tmp -> tmp
		arm_mean_q15(&tmpBuf[0], 100, &avg);
		arm_offset_q15(&tmpBuf[0], -avg, &tmpBuf[0], ADC_BUF_SIZE);

		//bandpass FIR filter to get our signal. tmp -> tmp2
		arm_fir_instance_q15 S;
		arm_fir_init_q15(&S, BP_NUM_TAPS, fir1, firState, BLOCK_SIZE);
		for (int i = 0; i < numBlocks; i++) {
			arm_fir_fast_q15(&S, &tmpBuf[0] + (i * blockSize),
					&tmpBuf2[0] + (i * blockSize), blockSize);
		}

		//abs tmp2 -> tmp
		arm_abs_q15(&tmpBuf2[0], &tmpBuf[0], ADC_BUF_SIZE);

//		printf("ch %d\n", channel);
//		for (int i = 0; i < ADC_BUF_SIZE; i+= 8) {
//			printf("%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n",
//					tmpBuf[i], tmpBuf[i+1], tmpBuf[i+2], tmpBuf[i+3],
//					tmpBuf[i+4], tmpBuf[i+5], tmpBuf[i+6], tmpBuf[i+7] );
//		}
		//find an edge, then find some peaks to fit a line
		peakIndex[channel] = 0;
		int searchStart;
		for (searchStart = 0; searchStart < ADC_BUF_SIZE && tmpBuf[searchStart] < 10; searchStart++)
			;
		int foundIndex;
		q15_t foundValue;
		findNextPeak(-1, tmpBuf, searchStart, ADC_BUF_SIZE, &foundIndex, &foundValue);



		timer2 = ms - timer;
	}
	timer3 = ms - timer;

	volatile int16_t delay12 = peakIndex[1] - peakIndex[0];
	volatile int16_t delay13 = peakIndex[2] - peakIndex[0];
	volatile int16_t delay14 = peakIndex[3] - peakIndex[0];

	printf("hit: %d, %d, %d, %d, deltas: %d, %d, %d in %dms\n", peakIndex[0],
			peakIndex[1], peakIndex[2], peakIndex[3], delay12, delay13,
			delay14, timer3);
}

uint32_t helloTimer;

void loop() {
//	if (ms - helloTimer > 5000) {
//		helloTimer = ms;
//		printf("hi\n");
//	}

	if (HAL_DMA_GetState(&hdma_tim4_up) == HAL_DMA_STATE_READY && pbCheckDone(&pb)) {
		//TODO reset the timer counter or it can cut clocks, this seems to transfer immediately
		HAL_DMA_Start_IT(&hdma_tim4_up, ledBuffer, &GPIOB->ODR, LED_BUFFER_SIZE);
	}



#if 0
	memcpy(cbuf[0], sample1 + 1500, ADC_BUF_SIZE * sizeof(q15_t));
	memcpy(cbuf[1], sample2 + 1500, ADC_BUF_SIZE * sizeof(q15_t));
	memcpy(cbuf[2], sample3 + 1500, ADC_BUF_SIZE * sizeof(q15_t));
	memcpy(cbuf[3], sample4 + 1500, ADC_BUF_SIZE * sizeof(q15_t));

	analyzeDelays();
	HAL_Delay(1000);

	return;

#endif

	if (triggerDone) {
		if (ms - lastHitMs > 500) {

#if 0
			printf("Trigger at %d\nch1\tch2\tch3\tch4\n", dmaCndtr);
			for (int i = 0; i < ADC_BUF_SIZE; i++) {
				int index = (ADC_BUF_SIZE + i - dmaCndtr + 1) % ADC_BUF_SIZE;
				printf("%u\t%u\t%u\t%u\n", cbuf[0][index], cbuf[1][index], cbuf[2][index], cbuf[3][index]);
			}
			printf("=========\n");
#endif

			analyzeDelays();
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

