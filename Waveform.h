#pragma once

#include "OpenScanDeviceLib.h"

#include <stdint.h>


// TODO We should probably scale the retrace length according to
// zoomFactor * width_or_height
static const uint32_t X_RETRACE_LEN = 128;
static const uint32_t Y_RETRACE_LEN = 12;

const struct WaveformParams {
	uint32_t pixelsPerLine;
	uint32_t numScanLines;
	uint32_t lineDelay;
	uint32_t xOffset;
	uint32_t yOffset;
};

void GenerateGalvoWaveform(int32_t effectiveScanLen, int32_t retraceLen,
	int32_t undershootLen, double scanStart, double scanEnd, double *waveform);
void SplineInterpolate(int32_t n, double yFirst, double yLast,
	double slopeFirst, double slopeLast, double *result);
OScDev_RichError *GenerateLineClock(const struct WaveformParams* parameters, uint8_t * lineClock);
OScDev_RichError *GenerateFLIMLineClock(const struct WaveformParams* parameters, uint8_t * lineClockFLIM);
OScDev_RichError* GenerateFLIMFrameClock(const struct WaveformParams* parameters, uint8_t* frameClockFLIM);
OScDev_RichError *GenerateGalvoWaveformFrame(uint32_t resolution, double zoom, uint32_t undershoot,
	uint32_t xStart, uint32_t yStart,
	uint32_t pixelsPerLine, uint32_t linesPerFrame,
	double galvoOffsetX, double galvoOffsetY, double *xyWaveformFrame);