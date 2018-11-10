#include <stdlib.h>

#include "stm32f3xx_hal.h"


#include "app.h"


#define DEBUG_DATA 0

extern volatile uint32_t ms;


//s=100; bp = fir1(s,[1900/ny, 2100/ny], 'pass')*(2^19);
#define BP_NUM_TAPS              (102)
const q15_t fir1[] = { 1452, 1371, 1228, 1014, 718, 330, -155, -733, -1385,
		-2079, -2766, -3386, -3867, -4139, -4132, -3793, -3086, -2007, -582,
		1126, 3023, 4982, 6856, 8485, 9711, 10392, 10416, 9709, 8253, 6084,
		3299, 48, -3467, -7017, -10347, -13208, -15368, -16638, -16881, -16033,
		-14105, -11187, -7444, -3107, 1545, 6205, 10558, 14305, 17185, 18998,
		19617, 18998, 17185, 14305, 10558, 6205, 1545, -3107, -7444, -11187,
		-14105, -16033, -16881, -16638, -15368, -13208, -10347, -7017, -3467,
		48, 3299, 6084, 8253, 9709, 10416, 10392, 9711, 8485, 6856, 4982, 3023,
		1126, -582, -2007, -3086, -3793, -4132, -4139, -3867, -3386, -2766,
		-2079, -1385, -733, -155, 330, 718, 1014, 1228, 1371, 1452, 0 };


q15_t tmpBuf[ADC_BUF_SIZE];
q15_t tmpBuf2[ADC_BUF_SIZE];
uint32_t peakIndex[4];

#define MAX_PEAKS 4
int peaks[4][MAX_PEAKS] = { 0 };


q15_t means[4][8];


#define BLOCK_SIZE            64
q15_t firState[BLOCK_SIZE + BP_NUM_TAPS];

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = ADC_BUF_SIZE / BLOCK_SIZE;

int findThreshold(q15_t *pInt, int size, int start, q15_t threshold);
int findPeaks(int *peaks, int peaksSize, q15_t *data, int size, int start,
		int dir, q15_t threshold, int maxDistance);




void putMeans(q15_t avg[4]) {
	static int index = 0;
	means[0][index] = avg[0];
	means[1][index] = avg[1];
	means[2][index] = avg[2];
	means[3][index] = avg[3];

	index = (index + 1) & 0x7;
}

static int compare_ints(const void* a, const void* b) {
	q15_t arg1 = *(const q15_t*)a;
	q15_t arg2 = *(const q15_t*)b;
    if (arg1 < arg2) return -1;
    if (arg1 > arg2) return 1;
    return 0;
}

q15_t getMeian(int channel) {
	q15_t cpy[8];
	memcpy(cpy, means[channel], sizeof(q15_t) * 8);
	qsort(cpy, 8, sizeof(q15_t), compare_ints);
	return cpy[3];
}

void seedMeans(int16_t cbuf[4][ADC_BUF_SIZE]) {
	q15_t avg[4];
	for (int chan = 0; chan < 4; chan++) {
		qsort(cbuf[chan], ADC_BUF_SIZE, sizeof(q15_t), compare_ints);
		avg[chan] = cbuf[chan][ADC_BUF_SIZE/2];
	}
	for (int i = 0; i < 8; i++) {
		putMeans(avg);
	}
}

int analyzeDelays(int16_t cbuf[4][ADC_BUF_SIZE], int dmaCndtr) {

	volatile uint32_t timer = 0;
	volatile uint32_t timer2 = 0;
	volatile uint32_t timer3 = 0;
	volatile uint32_t timerTemp = 0;
	q15_t avg[4];
	int channel;
	timer = ms;
	//filter out the DC
	for (channel = 0; channel < 4; channel++) {

		int startIndex = ADC_BUF_SIZE - dmaCndtr;
		//copy, align to tmp
		if (ADC_BUF_SIZE - startIndex > 0) {
			memcpy(&tmpBuf[0], &cbuf[channel][startIndex],
					(ADC_BUF_SIZE - startIndex) * sizeof(q15_t));
		}

		if (startIndex != 0) {
			memcpy(&tmpBuf[ADC_BUF_SIZE - startIndex], &cbuf[channel][0],
					startIndex * sizeof(q15_t));
		}

		q15_t rawMax, rawMin;
		uint32_t rawMaxIndex, rawMinIndex;

		arm_max_q15(&tmpBuf[0], ADC_BUF_SIZE, &rawMax, &rawMaxIndex);
		arm_min_q15(&tmpBuf[0], ADC_BUF_SIZE, &rawMin, &rawMinIndex);

		timerTemp = ms - timer;

		//remove dc. tmp -> tmp
		//TODO optimize this, maybe calc over long period of time
		arm_mean_q15(&tmpBuf[0], 100, &avg[channel]);

//		arm_offset_q15(&tmpBuf[0], -avg[channel], &tmpBuf[0], ADC_BUF_SIZE);
		//use the median instead of this instantaneous average
		arm_offset_q15(&tmpBuf[0], -getMeian(channel), &tmpBuf[0], ADC_BUF_SIZE);

		timerTemp = ms - timer;

		//bandpass FIR filter to get our signal. tmp -> tmp2
		arm_fir_instance_q15 S;
		arm_fir_init_q15(&S, BP_NUM_TAPS, fir1, firState, BLOCK_SIZE);
		for (int i = 0; i < numBlocks; i++) {
			arm_fir_fast_q15(&S, &tmpBuf[0] + (i * blockSize),
					&tmpBuf2[0] + (i * blockSize), blockSize);
		}

		timerTemp = ms - timer;

		//abs tmp2 -> tmp
		arm_abs_q15(&tmpBuf2[0], &tmpBuf[0], ADC_BUF_SIZE);

		timerTemp = ms - timer;

#if DEBUG_DATA
		printf("filtered ch %d\n", channel);
		for (int i = 0; i < ADC_BUF_SIZE; i++) {
			printf("%d\n", tmpBuf2[i]);
		}
#endif
		//find a spot to start searching, basic threshold?
		//compare a strong-near hit vs a far-weak hit
		//should we use max to get a range or some fixed value? perhaps start with fixed?
		//rewind either by finding peaks before, or some fixed amount?
		//fixed amount is easy, but then how to discard junk?
		//if we hop by peaks, then how to know when to stop? a threshold? if so why not set search spot to that peak?
		//to avoid false positives. start at a strong signal
		//keep track of interval to discard/detect false peaks? Probably not necessary with a strong filter

		//peak detection: max value or find the center? even with max, if it plateaus?

		memset(peaks[channel], 0, MAX_PEAKS * sizeof(peaks[0][0]));

		q15_t threshold, maxValue;
		uint32_t maxIndex;
		arm_max_q15(tmpBuf, ADC_BUF_SIZE, &maxValue, &maxIndex);

		threshold = maxValue / 2;
		int start = findThreshold(tmpBuf, ADC_BUF_SIZE, 0, threshold);

//#if DEBUG_DATA
		printf("max=%d\t@%d\tthresh=%d\tstart=%d\n", maxValue, maxIndex, threshold, start);
//#endif

		threshold = maxValue / 50;
		threshold = threshold < 1 ? 1 : threshold;
		threshold = threshold > 5 ? threshold : 5;
		int found = findPeaks(peaks[channel], 2, tmpBuf, ADC_BUF_SIZE, start, -1,
				threshold, 50);

		threshold = maxValue / 5;
		found += findPeaks(peaks[channel] + found, MAX_PEAKS - found, tmpBuf,
				ADC_BUF_SIZE, start, 1, threshold, 30);

		int16_t lastValue = 0;
#if DEBUG_DATA
		printf("ch %d found:\ni\tindex\tdeltaIndex\tvalue\tvalueDelta\n", channel);
		int lastIndex = peaks[channel][0];
		for (int i = 0; i < found; i++) {
			printf("%d\t%d\t%+d\t%d\t%+d\n", i, peaks[channel][i], peaks[channel][i] - lastIndex, tmpBuf[peaks[channel][i]], tmpBuf[peaks[channel][i]] - lastValue);
			lastIndex = peaks[channel][i];
			lastValue = tmpBuf[peaks[channel][i]];
		}
#endif

		//TODO filter any peaks that decline, only looking for upward slope
		//maybe flattening them is sufficient
		lastValue = 0;
		if (found) {
			for (int i = 0; i < found; i++) {
				if (tmpBuf[peaks[channel][i]] - lastValue < 0)
					tmpBuf[peaks[channel][i]] = lastValue;
				lastValue = tmpBuf[peaks[channel][i]];
			}
			//trim off any trailing flat peaks (ones that are the same as the previous sample)
			while (found >= 2
					&& tmpBuf[peaks[channel][found - 1]] == tmpBuf[peaks[channel][found - 2]])
				found--;

//#if DEBUG_DATA
			printf("ch %d flattened:\ni\tindex\tdeltaIndex\tvalue\tvalueDelta\n", channel);
			int lastIndex = peaks[channel][0];
			int16_t lastValue = 0;
			for (int i = 0; i < found; i++) {
				printf("%d\t%d\t%+d\t%d\t%+d\n", i, peaks[channel][i], peaks[channel][i] - lastIndex, tmpBuf[peaks[channel][i]], tmpBuf[peaks[channel][i]] - lastValue);
				lastIndex = peaks[channel][i];
				lastValue = tmpBuf[peaks[channel][i]];
			}
//#endif
		}

		if (found > 1) {
			//calc total then mean
			float32_t Xt = 0, Yt = 0;
			for (int i = 0; i < found; i++) {
				Xt += peaks[channel][i];
				Yt += tmpBuf[peaks[channel][i]];
			}
			//q16.16
			float32_t Xm = Xt / found;
			float32_t Ym = Yt / found;

			//calc lease squares
			//https://www.varsitytutors.com/hotmath/hotmath_help/topics/line-of-best-fit
			float32_t xy = 0, x2 = 0;
			for (int i = 0; i < found; i++) {
				float32_t xd = peaks[channel][i] - Xm;
				float32_t yd = tmpBuf[peaks[channel][i]] - Ym;
				xy += xd * yd;
				x2 += xd * xd;
			}

			float32_t m = xy / x2;
			float32_t b = Ym - m * Xm;
			float32_t xi = -b / m;

			peakIndex[channel] = xi;

			printf("ch=%d\tmin=%d\tmax=%d\tXm=%d\tYm=%d\txy=%d\tx2=%d\tm*1000=%d\tb=%d\txi=%d\n", channel, rawMin, rawMax, (int) Xm,
					(int) Ym, (int) xy, (int) x2, (int) (m*1000.0), (int) b, (int) xi);

		} else if (found == 1){
			printf("Didn't find enough peaks for slope\n");
			peakIndex[channel] = peaks[channel][0];
		} else {
			printf("Didn't find any peaks!\n");
			peakIndex[channel] = start;
		}

//#if DEBUG_DATA
//		if (found < 2 || xi <= 0 || m < .01 || xi >= 2048) {
			printf("filtered ch %d\n", channel);
			union {
				uint32_t packed;
				char bytes[5];
			} hexbuf;
			for (int i = 0; i < ADC_BUF_SIZE; i++) {
				hexbuf.packed = hexShort(tmpBuf[i]);
				hexbuf.bytes[4] = '\n';
				_write(0, hexbuf.bytes, 5);
			}
			printf("=========\n");
			flushConsole();
//		}
//#endif

		timer2 = ms - timer;
	}
	timer3 = ms - timer;

	putMeans(avg);

	volatile int16_t delay12 = peakIndex[1] - peakIndex[0];
	volatile int16_t delay13 = peakIndex[2] - peakIndex[0];
	volatile int16_t delay14 = peakIndex[3] - peakIndex[0];

	printf("hit: %d, %d, %d, %d, deltas: %d, %d, %d in %dms\n", peakIndex[0],
			peakIndex[1], peakIndex[2], peakIndex[3], delay12, delay13, delay14,
			timer3);
	return !(peakIndex[0] && peakIndex[1] && peakIndex[2] && peakIndex[3]);
}

/**
 *
 * @param peaks
 * @param peaksSize
 * @param data
 * @param size
 * @param start
 * @param dir
 * @param threshold
 * @return
 */
int findPeaks(int *peaks, int peaksSize, q15_t *data, int size, int start,
		int dir, q15_t threshold, int maxDistance) {
	int start2 = start;
	//skip over the falling part of the signal
	q15_t last = data[start2];
	for (; start2 >= 0 && start2 < size; start2 += dir) {
		q15_t v = data[start2];
		if (v > last)
			break;
		last = v;
	}
	q15_t max = data[start2];
	int found = -1;
	int stride = 0;
	for (int i = start2; i >= 0 && i < size && abs(start - i) < maxDistance;
			i += dir) {
		q15_t v = data[i];
		if (v > max && v > threshold) {
			max = v;
			found = i;
			stride = 1;
		} else if (found >= 0 && v == max) {
			stride++;
		} else if (found >= 0 && v < max) {
			if (peaksSize <= 1) {
				peaks[0] = found;
				return 1;
			} else {
				found += stride / 2 * dir;
				//find more
				if (dir > 0) {
					peaks[0] = found;
					int res = findPeaks(peaks + 1, peaksSize - 1, data, size, i,
							dir, threshold, maxDistance);
					return res + 1;
				} else {
					int res = findPeaks(peaks, peaksSize - 1, data, size, i,
							dir, threshold, maxDistance);
					//place our result after
					peaks[res] = found;
					return res + 1;
				}
			}
		}
	}
	return 0;
}

int findThreshold(q15_t *pInt, int size, int start, q15_t threshold) {
	for (int i = start; i < size; i++) {
		if (pInt[i] >= threshold)
			return i;
	}
	return 0;
}

