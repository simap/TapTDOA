#ifndef __APP_H__
#define __APP_H__


#define ADC_BUF_SIZE (1024*2)

void setup();

void loop();

int analyzeDelays(int16_t cbuf[4][ADC_BUF_SIZE], int dmaCndtr);

#endif
