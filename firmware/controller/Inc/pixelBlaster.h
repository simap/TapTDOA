#include <stdint.h>

typedef struct {
	uint8_t * pixelData;
	int dataSize;
	int readPos;
	int headerPos;
	enum {
		STARTING, READING, DONE
	} readMode;
} PixelBlasterData;


void pbInit(PixelBlasterData * d, uint8_t * data, size_t size) ;
int pbCheckDone(PixelBlasterData * pb);
void pbRx(PixelBlasterData * pb, uint8_t * buf, int len);
