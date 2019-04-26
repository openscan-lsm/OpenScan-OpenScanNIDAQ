#pragma once

#include "OpenScanDeviceLib.h"

#include <stdint.h>

static const uint32_t X_RETRACE_LEN = 128;
static const uint32_t Y_RETRACE_LEN = 12;


OScDev_Error GenerateScaledWaveforms(uint32_t resolution, double zoom, uint32_t lineDelay, uint16_t *xScaled, uint16_t *yScaled,
	double galvoOffsetX, double galvoOffsetY);
void GenerateGalvoWaveform(int32_t effectiveScanLen, int32_t retraceLen,
	int32_t undershootLen, double scanStart, double scanEnd, double *waveform);
void SplineInterpolate(int32_t n, double yFirst, double yLast,
	double slopeFirst, double slopeLast, double* result);
int GenerateLineClock(uint32_t x_resolution, uint32_t numScanLines, uint32_t lineDelay, uint8_t * lineClock);
int GenerateFLIMLineClock(uint32_t x_resolution, uint32_t numScanLines, uint32_t lineDelay, uint8_t * lineClockFLIM);
int GenerateFLIMFrameClock(uint32_t x_resolution, uint32_t numScanLines, uint32_t lineDelay, uint8_t * frameClockFLIM);
int GenerateGalvoWaveformFrame(uint32_t resolution, double zoom, uint32_t lineDelay,
	double galvoOffsetX, double galvoOffsetY, double *xyWaveformFrame);