#pragma once

#include "OpenScanDeviceLib.h"

#include <stdint.h>

const struct WaveformParams {
	uint32_t width; //PixelsPerLine
	uint32_t height; //numScanLines
	uint32_t resolution;
	double zoom;
	uint32_t undershoot; //also LineDelay for clock waveforms
	uint32_t xOffset;
	uint32_t yOffset;
	double galvoOffsetX;
	double galvoOffsetY;
};

void GenerateGalvoWaveform(int32_t effectiveScanLen, int32_t retraceLen,
	int32_t undershootLen, double scanStart, double scanEnd, double *waveform);
void SplineInterpolate(int32_t n, double yFirst, double yLast,
	double slopeFirst, double slopeLast, double *result);
OScDev_RichError *GenerateLineClock(const struct WaveformParams* parameters, uint8_t * lineClock);
OScDev_RichError *GenerateFLIMLineClock(const struct WaveformParams* parameters, uint8_t * lineClockFLIM);
OScDev_RichError* GenerateFLIMFrameClock(const struct WaveformParams* parameters, uint8_t* frameClockFLIM);
int32_t GetLineWaveformSize(const struct WaveformParams* parameters);
int32_t GetClockWaveformSize(const struct WaveformParams* parameters);
int32_t GetScannerWaveformSize(const struct WaveformParams* parameters);
int32_t GetScannerWaveformSizeAfterLastPixel(const struct WaveformParams* parameters);
OScDev_RichError *GenerateGalvoWaveformFrame(const struct WaveformParams* parameters, double* xyWaveformFrame);