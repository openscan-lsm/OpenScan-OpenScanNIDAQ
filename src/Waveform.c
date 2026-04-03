#include "Waveform.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

static const double MIN_RETRACE_US = 50.0;
static const double PARK_UNPARK_DURATION_US = 640.0;

static double ScanAmplitude(const struct WaveformParams *params) {
    return (double)params->width / (params->zoom * params->resolution);
}

uint32_t UndershootSamples(const struct WaveformParams *params) {
    return (uint32_t)round(params->undershootUs * 1e-6 * params->aoRateHz);
}

uint32_t ScanSamples(const struct WaveformParams *params) {
    return (uint32_t)round((double)params->width / params->pixelRateHz *
                           params->aoRateHz);
}

uint32_t ScanPhaseSamples(const struct WaveformParams *params) {
    return (uint32_t)round(params->scanPhaseUs * 1e-6 * params->aoRateHz);
}

uint32_t RetraceSamples(const struct WaveformParams *params) {
    double amplitude = ScanAmplitude(params);
    double retraceUs = params->retraceScaleUsPerVolt * amplitude;
    if (retraceUs < MIN_RETRACE_US)
        retraceUs = MIN_RETRACE_US;
    return (uint32_t)round(retraceUs * 1e-6 * params->aoRateHz);
}

uint32_t ParkSamples(const struct WaveformParams *params) {
    return (uint32_t)round(PARK_UNPARK_DURATION_US * 1e-6 * params->aoRateHz);
}

// n = number of elements
// slope in units of per element
static void SplineInterpolate(int32_t n, double yFirst, double yLast,
                              double slopeFirst, double slopeLast,
                              double *result) {
    double m = n;
    double mm = m * m;
    double mmm = m * m * m;
    double c[4];

    c[0] = slopeFirst / mm + 2.0 * yFirst / mmm + slopeLast / mm -
           2.0 * yLast / mmm;
    c[1] = 3.0 * yLast / mm - slopeLast / m - 2.0 * slopeFirst / m -
           3.0 * yFirst / mm;
    c[2] = slopeFirst;
    c[3] = yFirst;

    for (int32_t x = 0; x < n; x++) {
        result[x] = c[0] * x * x * x + c[1] * x * x + c[2] * x + c[3];
    }
}

// Generate 1D (undershoot + scan + overshoot + retrace).
// The scan part spans voltage scanStart to scanEnd.
// overshootLen extends the linear ramp past scanEnd.
static void GenerateXGalvoWaveform(int32_t effectiveScanLen,
                                   int32_t retraceLen, int32_t undershootLen,
                                   int32_t overshootLen, double scanStart,
                                   double scanEnd, double *waveform) {
    double scanAmplitude = scanEnd - scanStart;
    double step = scanAmplitude / effectiveScanLen;
    int32_t linearLen = undershootLen + effectiveScanLen + overshootLen;

    double undershootStart = scanStart - undershootLen * step;
    for (int i = 0; i < linearLen; ++i) {
        waveform[i] = undershootStart + step * i;
    }

    double overshootEnd = scanEnd + overshootLen * step;
    if (retraceLen > 0) {
        SplineInterpolate(retraceLen, overshootEnd, undershootStart, step,
                          step, waveform + linearLen);
    }
}

// Generate Y waveform for one frame
static void GenerateYGalvoWaveform(int32_t linesPerFrame, int32_t retraceLen,
                                   size_t xLength, double scanStart,
                                   double scanEnd, double *waveform) {
    double scanAmplitude = scanEnd - scanStart;
    double step = scanAmplitude / linesPerFrame;

    for (int j = 0; j < linesPerFrame; ++j) {
        for (unsigned i = 0; i < xLength; ++i) {
            waveform[i + j * xLength] = scanStart + step * j;
            if ((j >= linesPerFrame - 1) &&
                (i >= xLength - (unsigned)retraceLen)) {
                break;
            }
        }
    }

    // Smooth Y transitions during each line's X retrace
    for (int j = 0; j < linesPerFrame - 1; ++j) {
        double yThis = scanStart + step * j;
        double yNext = scanStart + step * (j + 1);
        SplineInterpolate(retraceLen, yThis, yNext, 0, 0,
                          waveform + (j + 1) * xLength - retraceLen);
    }

    // Frame retrace at end
    if (retraceLen > 0) {
        double lastLineY = scanStart + step * (linesPerFrame - 1);
        SplineInterpolate(retraceLen, lastLineY, scanStart, 0, 0,
                          waveform + (linesPerFrame * xLength) - retraceLen);
    }
}

void GenerateLineClock(const struct WaveformParams *parameters,
                       uint8_t *lineClock) {
    uint32_t undershootSmp = UndershootSamples(parameters);
    uint32_t scanPhaseSmp = ScanPhaseSamples(parameters);
    uint32_t scanSmp = ScanSamples(parameters);
    uint32_t height = parameters->height;

    uint32_t highStart = undershootSmp + scanPhaseSmp;
    uint32_t x_length = (uint32_t)GetLineWaveformSize(parameters);
    for (uint32_t j = 0; j < height; j++)
        for (uint32_t i = 0; i < x_length; i++)
            lineClock[i + j * x_length] =
                ((i >= highStart) && (i < highStart + scanSmp)) ? 1 : 0;
}

void GenerateFLIMLineClock(const struct WaveformParams *parameters,
                           uint8_t *lineClockFLIM) {
    uint32_t undershootSmp = UndershootSamples(parameters);
    uint32_t scanPhaseSmp = ScanPhaseSamples(parameters);
    uint32_t scanSmp = ScanSamples(parameters);
    uint32_t height = parameters->height;

    uint32_t highStart = undershootSmp + scanPhaseSmp + scanSmp;
    uint32_t x_length = (uint32_t)GetLineWaveformSize(parameters);
    for (uint32_t j = 0; j < height; j++)
        for (uint32_t i = 0; i < x_length; i++)
            lineClockFLIM[i + j * x_length] = (i >= highStart) ? 1 : 0;
}

void GenerateFLIMFrameClock(const struct WaveformParams *parameters,
                            uint8_t *frameClockFLIM) {
    uint32_t undershootSmp = UndershootSamples(parameters);
    uint32_t scanPhaseSmp = ScanPhaseSamples(parameters);
    uint32_t scanSmp = ScanSamples(parameters);
    uint32_t height = parameters->height;

    uint32_t highStart = undershootSmp + scanPhaseSmp + scanSmp;
    uint32_t x_length = (uint32_t)GetLineWaveformSize(parameters);

    for (uint32_t j = 0; j < height; ++j)
        for (uint32_t i = 0; i < x_length; ++i)
            frameClockFLIM[i + j * x_length] =
                ((j == height - 1) && (i > highStart)) ? 1 : 0;
}

int32_t GetLineWaveformSize(const struct WaveformParams *parameters) {
    return (int32_t)(UndershootSamples(parameters) + ScanSamples(parameters) +
                     ScanPhaseSamples(parameters) +
                     RetraceSamples(parameters));
}

int32_t GetClockWaveformSize(const struct WaveformParams *parameters) {
    uint32_t elementsPerLine = (uint32_t)GetLineWaveformSize(parameters);
    uint32_t height = parameters->height;
    return (int32_t)(elementsPerLine * height);
}

int32_t GetScannerWaveformSize(const struct WaveformParams *parameters) {
    uint32_t elementsPerLine = (uint32_t)GetLineWaveformSize(parameters);
    uint32_t height = parameters->height;
    return (int32_t)(elementsPerLine * height);
}

int32_t
GetScannerWaveformSizeAfterLastPixel(const struct WaveformParams *parameters) {
    return (int32_t)(ScanPhaseSamples(parameters) +
                     RetraceSamples(parameters));
}

int32_t GetParkWaveformSize(const struct WaveformParams *parameters) {
    return (int32_t)ParkSamples(parameters);
}

void GenerateGalvoWaveformFrame(const struct WaveformParams *parameters,
                                double *xyWaveformFrame) {
    uint32_t pixelsPerLine = parameters->width;
    uint32_t linesPerFrame = parameters->height;
    uint32_t resolution = parameters->resolution;
    double zoom = parameters->zoom;
    uint32_t xOffset = parameters->xOffset;
    uint32_t yOffset = parameters->yOffset;
    const double *m = parameters->xformMatrix;
    double tx = parameters->xformOffsetX;
    double ty = parameters->xformOffsetY;

    uint32_t undershootSmp = UndershootSamples(parameters);
    uint32_t scanSmp = ScanSamples(parameters);
    uint32_t scanPhaseSmp = ScanPhaseSamples(parameters);
    uint32_t retraceSmp = RetraceSamples(parameters);

    double xStart = (-0.5 * resolution + xOffset) / (zoom * resolution);
    double yStart = (-0.5 * resolution + yOffset) / (zoom * resolution);
    double xEnd = xStart + (double)pixelsPerLine / (zoom * resolution);
    double yEnd = yStart + (double)linesPerFrame / (zoom * resolution);

    size_t xLength = undershootSmp + scanSmp + scanPhaseSmp + retraceSmp;
    size_t yLength = linesPerFrame;

    double *xWaveform = (double *)malloc(sizeof(double) * xLength);
    double *yWaveform = (double *)malloc(sizeof(double) * (yLength * xLength));
    GenerateXGalvoWaveform((int32_t)scanSmp, (int32_t)retraceSmp,
                           (int32_t)undershootSmp, (int32_t)scanPhaseSmp,
                           xStart, xEnd, xWaveform);
    GenerateYGalvoWaveform((int32_t)linesPerFrame, (int32_t)retraceSmp,
                           xLength, yStart, yEnd, yWaveform);

    for (unsigned j = 0; j < yLength; ++j) {
        for (unsigned i = 0; i < xLength; ++i) {
            double x = xWaveform[i];
            double y = yWaveform[i + j * xLength];
            xyWaveformFrame[i + j * xLength] = m[0] * x + m[1] * y + tx;
            xyWaveformFrame[i + j * xLength + yLength * xLength] =
                m[2] * x + m[3] * y + ty;
        }
    }

    free(xWaveform);
    free(yWaveform);
}

static void InverseTransform2x2(const double *m, double tx, double ty,
                                double ox, double oy, double *lx, double *ly) {
    double det = m[0] * m[3] - m[1] * m[2];
    double cx = ox - tx;
    double cy = oy - ty;
    *lx = (m[3] * cx - m[1] * cy) / det;
    *ly = (-m[2] * cx + m[0] * cy) / det;
}

void GenerateGalvoUnparkWaveform(const struct WaveformParams *parameters,
                                 double *xyWaveformFrame) {
    uint32_t resolution = parameters->resolution;
    double zoom = parameters->zoom;
    uint32_t xOffset = parameters->xOffset;
    uint32_t yOffset = parameters->yOffset;
    const double *m = parameters->xformMatrix;
    double tx = parameters->xformOffsetX;
    double ty = parameters->xformOffsetY;

    uint32_t undershootSmp = UndershootSamples(parameters);
    uint32_t scanSmp = ScanSamples(parameters);
    double step = ScanAmplitude(parameters) / scanSmp;

    double xStart, yStart;
    InverseTransform2x2(m, tx, ty, parameters->prevXParkVoltage,
                        parameters->prevYParkVoltage, &xStart, &yStart);

    double xEnd = (-0.5 * resolution + xOffset) / (zoom * resolution) -
                  undershootSmp * step;
    double yEnd = (-0.5 * resolution + yOffset) / (zoom * resolution);

    size_t length = ParkSamples(parameters);
    double *xWaveform = (double *)malloc(sizeof(double) * length);
    double *yWaveform = (double *)malloc(sizeof(double) * length);

    SplineInterpolate((int32_t)length, xStart, xEnd, 0, 0, xWaveform);
    SplineInterpolate((int32_t)length, yStart, yEnd, 0, 0, yWaveform);

    for (unsigned i = 0; i < length; ++i) {
        double x = xWaveform[i];
        double y = yWaveform[i];
        xyWaveformFrame[i] = m[0] * x + m[1] * y + tx;
        xyWaveformFrame[i + length] = m[2] * x + m[3] * y + ty;
    }

    free(xWaveform);
    free(yWaveform);
}

void GenerateGalvoParkWaveform(const struct WaveformParams *parameters,
                               double *xyWaveformFrame) {
    uint32_t resolution = parameters->resolution;
    double zoom = parameters->zoom;
    uint32_t xOffset = parameters->xOffset;
    uint32_t yOffset = parameters->yOffset;
    int32_t xPark = parameters->xPark;
    int32_t yPark = parameters->yPark;
    const double *m = parameters->xformMatrix;
    double tx = parameters->xformOffsetX;
    double ty = parameters->xformOffsetY;

    uint32_t undershootSmp = UndershootSamples(parameters);
    uint32_t scanSmp = ScanSamples(parameters);
    double step = ScanAmplitude(parameters) / scanSmp;

    double xStart = (-0.5 * resolution + xOffset) / (zoom * resolution) -
                    undershootSmp * step;
    double yStart = (-0.5 * resolution + yOffset) / (zoom * resolution);
    double xEnd = (-0.5 * resolution + xPark) / (zoom * resolution);
    double yEnd = (-0.5 * resolution + yPark) / (zoom * resolution);

    size_t length = ParkSamples(parameters);
    double *xWaveform = (double *)malloc(sizeof(double) * length);
    double *yWaveform = (double *)malloc(sizeof(double) * length);

    SplineInterpolate((int32_t)length, xStart, xEnd, 0, 0, xWaveform);
    SplineInterpolate((int32_t)length, yStart, yEnd, 0, 0, yWaveform);

    for (unsigned i = 0; i < length; ++i) {
        double x = xWaveform[i];
        double y = yWaveform[i];
        xyWaveformFrame[i] = m[0] * x + m[1] * y + tx;
        xyWaveformFrame[i + length] = m[2] * x + m[3] * y + ty;
    }

    free(xWaveform);
    free(yWaveform);
}
