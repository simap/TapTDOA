#include "stm32f3xx_hal.h"
#include "pixelBlaster.h"
#include <string.h>

//scan data looking for header
//buffer data until everything arrives
//spew out data on pins.

static const char * header = "pixels";
static const int headerLen = 6;


void pbInit(PixelBlasterData * d, uint8_t * data, size_t size) {
	d->headerPos = d->readPos = 0;
	d->pixelData = data;
	d->dataSize = size;
	d->readMode = STARTING;
}

int pbCheckDone(PixelBlasterData * pb) {
	if (pb->readMode == DONE) {
		pb->readMode = STARTING;
		return 1;
	}
	return 0;
}

//TODO if needed, could feed buffer positions to the usb stuff instead of copies
void pbRx(PixelBlasterData * pb, uint8_t * buf, int len) {

	for (uint8_t * p = buf; p - buf < len;) {
		switch (pb->readMode) {
		case DONE:
		case STARTING:
			if (*p++ != header[pb->headerPos++]) {
				pb->headerPos = 0;
				continue;
			}
			if (pb->headerPos >= headerLen) {
				pb->readMode = READING;
				break;
			}
			break;
		case READING: {
			int remainingToRead = pb->dataSize - pb->readPos;
			int remainingInBuffer = len + buf - p;
			int toCopy = remainingToRead < remainingInBuffer ?
							remainingToRead : remainingInBuffer;
			memcpy(&pb->pixelData[pb->readPos], p, toCopy);
			pb->readPos += toCopy;
			if (pb->readPos >= pb->dataSize) {
				pb->readMode = DONE;
				pb->readPos = 0;
				pb->headerPos = 0;
			}
			p += toCopy;
			break;
		}
		}
	}
}


