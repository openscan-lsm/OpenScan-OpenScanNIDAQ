#include "../Waveform.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// write results to binary file

void main(int argc, char* argv[]) {
	if (argc != 3) {
		fprintf(stderr, "2 arguments required: resolution, undershoot");
	}

	uint32_t resolution = atoi(argv[1]);
	uint32_t undershoot = atoi(argv[2]);

	double zoom = 1;
	uint32_t xOffset = 0;
	uint32_t yOffset = 0;
	uint32_t width = resolution;
	uint32_t height = resolution;
	double galvoOffsetX = 0;
	double galvoOffsetY = 0;

	uint32_t elementsPerLine = undershoot + width + X_RETRACE_LEN;
	uint32_t yLen = height + Y_RETRACE_LEN;
	uint32_t totalElementsPerFrame = elementsPerLine * yLen;   // including y retrace portion

	uint32_t bufferSize = totalElementsPerFrame * 2;
	double* xyWaveform = malloc(sizeof(double) * bufferSize);
	if (xyWaveform == NULL) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	GenerateGalvoWaveformFrame(resolution, zoom, undershoot, xOffset, yOffset,
		width, height, galvoOffsetX, galvoOffsetY, xyWaveform);

	FILE* testFile = fopen("WaveformTest.raw", "wb");
	fwrite(xyWaveform, sizeof(double), bufferSize, testFile);
	fclose(testFile);

	printf("total sample count = %lu\n", (unsigned long)bufferSize);
	free(xyWaveform);


}