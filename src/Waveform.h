#pragma once

#include <stdbool.h>
#include <stdint.h>

struct WaveformParams {
    uint32_t width;  // PixelsPerLine (ROI)
    uint32_t height; // numScanLines
    uint32_t resolution;
    double zoom;
    uint32_t xOffset;
    uint32_t yOffset;

    double aoRateHz;
    double pixelRateHz;
    double undershootUs;
    double scanPhaseUs;
    double retraceScaleUsPerVolt;

    double xformMatrix[4]; // {a, b, c, d} — row-major 2x2
    double xformOffsetX;   // tx (volts)
    double xformOffsetY;   // ty (volts)
    int32_t xPark;
    int32_t yPark;
    double prevXParkVoltage;
    double prevYParkVoltage;

    bool laserBlankingSupported; // device has >=3 AO
    bool laserManualOn;          // laser always on (blanking disabled)
    double laserOnVoltage;
    double laserOffVoltage;
    double laserOnLeadUs;
    double laserOnLagUs;
};

uint32_t UndershootSamples(const struct WaveformParams *params);
uint32_t ScanSamples(const struct WaveformParams *params);
uint32_t ScanPhaseSamples(const struct WaveformParams *params);
uint32_t RetraceSamples(const struct WaveformParams *params);
uint32_t ParkSamples(const struct WaveformParams *params);

void GenerateLineClock(const struct WaveformParams *parameters,
                       uint8_t *lineClock);
void GenerateFLIMLineClock(const struct WaveformParams *parameters,
                           uint8_t *lineClockFLIM);
void GenerateFLIMFrameClock(const struct WaveformParams *parameters,
                            uint8_t *frameClockFLIM);
int32_t GetLineWaveformSize(const struct WaveformParams *parameters);
int32_t GetClockWaveformSize(const struct WaveformParams *parameters);
int32_t GetScannerWaveformSize(const struct WaveformParams *parameters);
int32_t
GetScannerWaveformSizeAfterLastPixel(const struct WaveformParams *parameters);
int32_t GetParkWaveformSize(const struct WaveformParams *parameters);
void GenerateGalvoWaveformFrame(const struct WaveformParams *parameters,
                                double *xyWaveformFrame);
void GenerateGalvoUnparkWaveform(const struct WaveformParams *parameters,
                                 double *xyWaveformFrame);
void GenerateGalvoParkWaveform(const struct WaveformParams *parameters,
                               double *xyWaveformFrame);

uint32_t LaserOnLeadSamples(const struct WaveformParams *params);
uint32_t LaserOnLagSamples(const struct WaveformParams *params);
void GenerateLaserBlankingWaveform(const struct WaveformParams *params,
                                   double *blanking);
void GenerateLaserBlankingConstant(const struct WaveformParams *params,
                                   uint32_t length, double *out);
