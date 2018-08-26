
#include "stm32f3xx_hal.h"
#include "usb_device.h"
#include "app.h"

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

extern UART_HandleTypeDef huart1;

extern int16_t firBandpass[];
extern const uint16_t sample1[];
extern const uint16_t sample2[];
extern const uint16_t sample3[];
extern const uint16_t sample4[];

#define ADC_BUF_SIZE (1024*3)
#define POST_TRIGGER_SAMPLES (ADC_BUF_SIZE - 512)
#define XCORR_WINDOW_SIZE 1024
#define XCORR_SIZE (XCORR_WINDOW_SIZE*2) //need 2x window for cross correlation result

#define BLOCK_SIZE            32
#define NUM_TAPS              800

int16_t cbuf[4][ADC_BUF_SIZE];
q15_t filtered[4][ADC_BUF_SIZE];

uint32_t peakIndex[4];
q15_t xcorr[XCORR_SIZE];
q15_t firState[BLOCK_SIZE + NUM_TAPS];

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = ADC_BUF_SIZE / BLOCK_SIZE;


volatile int16_t triggerCountdown;
static volatile bool triggerDone = false;
int dmaCndtr;

volatile uint32_t ms;

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
	uint32_t tr1 = (high<<16) | low;
	ADC1->TR1 = tr1;
	ADC2->TR1 = tr1;
	ADC3->TR1 = tr1;
	ADC4->TR1 = tr1;
}

uint32_t findXCorrelationDelay(int ch1, int ch2) {
    //cross correlate a window, finding the peak correlation

    memset(&xcorr[0], 0, sizeof(q15_t) * XCORR_SIZE);

    int startOffset1 = peakIndex[ch1] - XCORR_WINDOW_SIZE / 2;
    int startOffset2 = peakIndex[ch2] - XCORR_WINDOW_SIZE / 2;
    arm_correlate_q15(filtered[ch1] + startOffset1, XCORR_WINDOW_SIZE,
                      filtered[ch2] + startOffset2, XCORR_WINDOW_SIZE,
                      &xcorr[0]);

    //find the peak correlation
    q15_t unused;
    uint32_t index;
    arm_max_q15(&xcorr[0], XCORR_SIZE, &unused, &index);
    return XCORR_WINDOW_SIZE - index +  peakIndex[ch2] - peakIndex[ch1];
}

void setup() {


	HAL_DACEx_DualSetValue(&hdac1, DAC_ALIGN_12B_R, 128, 128);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);

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

	setAdcWd(1925 - 100, 1925 + 100);

	//let things settle on powerup
	HAL_Delay(100);

//	DBGMCU->APB1FZ = 0xffff;
//	DBGMCU->APB2FZ = 0xffff;

	return;


	LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);

	LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_EnableIT_UPDATE(TIM2);

	LL_TIM_EnableCounter(TIM3);

// 	arm_cfft_q15(0, 0,0, 0);

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

uint32_t timer = 0;

uint32_t timer2 = 0;
uint32_t timer3 = 0;


void loop() {


//	if (ms - timer >= 1000) {
//		timer += 1000;
//		printf("Hello %d\n", ms);
//	}
//	return;

    q15_t avg;
    int channel;

    memcpy(cbuf[0], sample1 + 1500, ADC_BUF_SIZE * sizeof(q15_t));
    memcpy(cbuf[1], sample2 + 1500, ADC_BUF_SIZE * sizeof(q15_t));
    memcpy(cbuf[2], sample3 + 1500, ADC_BUF_SIZE * sizeof(q15_t));
    memcpy(cbuf[3], sample4 + 1500, ADC_BUF_SIZE * sizeof(q15_t));

    timer = ms;

    //filter out the DC
    for (channel = 0; channel < 4; channel++) {
        arm_mean_q15(cbuf[channel], 100, &avg);
        arm_offset_q15(cbuf[channel], -avg, cbuf[channel], ADC_BUF_SIZE);
    }

    //hack ch1 and ch4 had reversed polarity
    for (int i = 0; i < ADC_BUF_SIZE; i++) {
    	cbuf[0][i] = -cbuf[0][i];
    	cbuf[3][i] = -cbuf[3][i];
    }


    //bandpass FIR filter
    arm_fir_instance_q15 S;
    for (channel = 0; channel < 4; channel++) {
        arm_fir_init_q15(&S, NUM_TAPS, &firBandpass[0], firState, BLOCK_SIZE);
        for (int i = 0; i < numBlocks; i++) {
            arm_fir_q15(&S, cbuf[channel] + (i * blockSize), filtered[channel] + (i * blockSize), blockSize);
        }
    }

    //find peaks
    for (channel = 0; channel < 4; channel++) {
        q15_t unused;
        arm_max_q15(&filtered[channel][0], ADC_BUF_SIZE, &unused, &peakIndex[channel]);

        //make sure peak index isn't too close to bounds
//        int startOffset = peakIndex[channel] - XCORR_WINDOW_SIZE / 2;
//        if (startOffset < 0)
//            peakIndex[channel] = XCORR_WINDOW_SIZE / 2;
//        if (startOffset + XCORR_WINDOW_SIZE > ADC_BUF_SIZE)
//            peakIndex[channel] = ADC_BUF_SIZE - XCORR_WINDOW_SIZE / 2;
    }
    timer2 = ms - timer;

    volatile int delay12 = findXCorrelationDelay(0,1);
    timer3 = ms - timer;
    volatile int delay13 = findXCorrelationDelay(0,2);
    volatile int delay14 = findXCorrelationDelay(0,3);

    volatile uint32_t totalMs = ms - timer;


	HAL_Delay(1000);


	return;



	if (triggerDone) {


		printf("Trigger at %d\n", dmaCndtr);

		for (int i = 0; i < ADC_BUF_SIZE; i++) {
			int index = (ADC_BUF_SIZE + i - dmaCndtr + 1) % ADC_BUF_SIZE;
			for (int k = 0; k < 4; k++) {
				printf("%u\t", cbuf[k][index]);
			}
			printf("\n");
		}
		printf("=========\n");

		LL_TIM_EnableCounter(TIM3);

		HAL_Delay(300);
		triggerDone = false;
		//TODO figure out why adc watchdog keeps firing immediately after it gets reenabled
		triggerMode = WAITING;
		armAdcWatchdogs();
	}

}

void startEventCapture() {

	disarmAdcWatchdogs();
	triggerCountdown = POST_TRIGGER_SAMPLES;
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

