#pragma once

#include <stdbool.h>
#include <stdint.h>

struct WaveformParams {
    uint32_t width;  // PixelsPerLine
    uint32_t height; // numScanLines
    uint32_t resolution;
    double zoom;
    uint32_t undershoot; // also LineDelay for clock waveforms
    uint32_t xOffset;
    uint32_t yOffset;
    double xformMatrix[4]; // {a, b, c, d} — row-major 2x2
    double xformOffsetX;   // tx (volts)
    double xformOffsetY;   // ty (volts)
    int32_t xPark;
    int32_t yPark;
    double prevXParkVoltage;
    double prevYParkVoltage;
};

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
#define SPIRAL_SAMPLE_RATE_HZ 100000.0
#define SPIRAL_CONN_SAMPLES 128

struct SpiralWaveformParams {
    double radius;
    double centerX;
    double centerY;
    double turnSpacing;
    double turnDurationMs;
    double rMin;
    double xformMatrix[4];
    double xformOffsetX;
    double xformOffsetY;
};

struct SpiralGenState;

struct SpiralGenState *
CreateSpiralGenState(const struct SpiralWaveformParams *params);
void DestroySpiralGenState(struct SpiralGenState *state);
void GenerateSpiralChunk(struct SpiralGenState *state, double *xyBuffer,
                         int32_t count);
int32_t GetSpiralArmCycleSamples(const struct SpiralGenState *state);
int32_t GetSpiralArmCycleIndex(const struct SpiralGenState *state);
bool SpiralGenStateAtCycleBoundary(const struct SpiralGenState *state);

void GenerateGalvoUnparkWaveform(const struct WaveformParams *parameters,
                                 double *xyWaveformFrame);
void GenerateGalvoParkWaveform(const struct WaveformParams *parameters,
                               double *xyWaveformFrame);
