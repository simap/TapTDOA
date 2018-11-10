#ifndef __APP_H__
#define __APP_H__


#include <stdint.h>
#include <stdbool.h>

#define ARM_MATH_CM4
#include "arm_math.h"

#include "fastHex.h"

#define ADC_BUF_SIZE (1024*2)

void setup();

void loop();

int analyzeDelays(int16_t cbuf[4][ADC_BUF_SIZE], int dmaCndtr);
void seedMeans(int16_t cbuf[4][ADC_BUF_SIZE]);
q15_t getMeian(int channel);

#endif
