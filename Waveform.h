#pragma once

#include <OpenScanDeviceLib.h>

#include <stdint.h>

const struct WaveformParams {
    uint32_t width;  // PixelsPerLine
    uint32_t height; // numScanLines
    uint32_t resolution;
    double zoom;
    uint32_t undershoot; // also LineDelay for clock waveforms
    uint32_t xOffset;
    uint32_t yOffset;
    double galvoOffsetX;
    double galvoOffsetY;
    int32_t xPark;
    int32_t yPark;
    double prevXParkVoltage;
    double prevYParkVoltage;
};

OScDev_RichError *GenerateLineClock(const struct WaveformParams *parameters,
                                    uint8_t *lineClock);
OScDev_RichError *
GenerateFLIMLineClock(const struct WaveformParams *parameters,
                      uint8_t *lineClockFLIM);
OScDev_RichError *
GenerateFLIMFrameClock(const struct WaveformParams *parameters,
                       uint8_t *frameClockFLIM);
int32_t GetLineWaveformSize(const struct WaveformParams *parameters);
int32_t GetClockWaveformSize(const struct WaveformParams *parameters);
int32_t GetScannerWaveformSize(const struct WaveformParams *parameters);
int32_t
GetScannerWaveformSizeAfterLastPixel(const struct WaveformParams *parameters);
int32_t GetParkWaveformSize(const struct WaveformParams *parameters);
OScDev_RichError *
GenerateGalvoWaveformFrame(const struct WaveformParams *parameters,
                           double *xyWaveformFrame);
OScDev_RichError *
GenerateGalvoUnparkWaveform(const struct WaveformParams *parameters,
                            double *xyWaveformFrame);
OScDev_RichError *
GenerateGalvoParkWaveform(const struct WaveformParams *parameters,
                          double *xyWaveformFrame);
