#include "../Waveform.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// write results to binary file
static void DumpXYWaveform(uint32_t resolution, uint32_t undershoot) {

    struct WaveformParams params;
    params.width = resolution;
    params.height = resolution;
    params.resolution = resolution;
    params.zoom = 1;
    params.undershoot = undershoot;
    params.xOffset = 0;
    params.yOffset = 0;
    params.galvoOffsetX = 0;
    params.galvoOffsetY = 0;

    uint32_t totalElementsPerFrame = GetScannerWaveformSize(&params);

    uint32_t bufferSize = totalElementsPerFrame * 2;
    double *xyWaveform = malloc(sizeof(double) * bufferSize);
    if (xyWaveform == NULL) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    GenerateGalvoWaveformFrame(&params, xyWaveform);

    FILE *testFile = fopen("WaveformTest.raw", "wb");
    fwrite(xyWaveform, sizeof(double), bufferSize, testFile);
    fclose(testFile);

    printf("total sample count = %lu\n", (unsigned long)bufferSize);
    free(xyWaveform);
}

static void DumpClockWaveform(uint32_t resolution, uint32_t lineDelay) {
    struct WaveformParams WaveformParameters;
    WaveformParameters.width = resolution;
    WaveformParameters.height = resolution;
    WaveformParameters.undershoot = lineDelay;
    WaveformParameters.xOffset = 0;
    WaveformParameters.yOffset = 0;

    uint32_t elementsPerFramePerChan =
        GetClockWaveformSize(&WaveformParameters);
    uint32_t bufferSize = elementsPerFramePerChan;

    uint8_t *lineClockPattern = malloc(elementsPerFramePerChan);
    uint8_t *lineClockFLIM = malloc(elementsPerFramePerChan);
    uint8_t *frameClockFLIM = malloc(elementsPerFramePerChan);

    if (lineClockPattern == NULL || lineClockFLIM == NULL ||
        frameClockFLIM == NULL) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    GenerateLineClock(&WaveformParameters, lineClockPattern);
    GenerateFLIMLineClock(&WaveformParameters, lineClockFLIM);
    GenerateFLIMFrameClock(&WaveformParameters, frameClockFLIM);

    FILE *clockFile = fopen("clock_uint8_numofwaveforms_3.raw", "wb");
    fwrite(lineClockPattern, sizeof(uint8_t), bufferSize, clockFile);
    fwrite(lineClockFLIM, sizeof(uint8_t), bufferSize, clockFile);
    fwrite(frameClockFLIM, sizeof(uint8_t), bufferSize, clockFile);
    fclose(clockFile);

    printf("total sample count = %lu\n", (unsigned long)bufferSize * 3);
    free(lineClockPattern);
    free(lineClockFLIM);
    free(frameClockFLIM);
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        fprintf(
            stderr,
            "3 arguments required: waveformtype, resolution, undershoot/linedelay");
    }

    else {
        if (strcmp(argv[1], "XYWaveform") == 0) {
            uint32_t resolution = atoi(argv[2]);
            uint32_t undershoot = atoi(argv[3]);
            DumpXYWaveform(resolution, undershoot);
        }

        else if (strcmp(argv[1], "ClockWaveform") == 0) {
            uint32_t resolution = atoi(argv[2]);
            uint32_t lineDelay = atoi(argv[3]);
            DumpClockWaveform(resolution, lineDelay);
        }

        else {
            fprintf(stderr, "Invalid Waveform Type");
        }
    }
}
