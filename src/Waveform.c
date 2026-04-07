#include "Waveform.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

// TODO We should probably scale the retrace length according to
// zoomFactor * width_or_height
static const uint32_t X_RETRACE_LEN = 128;

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

// Generate 1D (undershoot + trace + retrace).
// The trace part spans voltage scanStart to scanEnd.
static void GenerateXGalvoWaveform(int32_t effectiveScanLen,
                                   int32_t retraceLen, int32_t undershootLen,
                                   double scanStart, double scanEnd,
                                   double *waveform) {
    double scanAmplitude = scanEnd - scanStart;
    double step = scanAmplitude / effectiveScanLen;
    int32_t linearLen = undershootLen + effectiveScanLen;

    // Generate the linear scan curve
    double undershootStart = scanStart - undershootLen * step;
    for (int i = 0; i < linearLen; ++i) {
        waveform[i] = undershootStart + step * i;
    }

    // Generate the rescan curve
    // Slope at start end end are both equal to the linear scan
    if (retraceLen > 0) {
        SplineInterpolate(retraceLen, scanEnd, undershootStart, step, step,
                          waveform + linearLen);
    }
}

// Generate Y waveform for one frame
static void GenerateYGalvoWaveform(int32_t linesPerFrame, int32_t retraceLen,
                                   size_t xLength, double scanStart,
                                   double scanEnd, double *waveform) {
    (void)retraceLen; // Unused

    double scanAmplitude = scanEnd - scanStart;
    double step = scanAmplitude / linesPerFrame;

    // Generate staircase for one frame
    for (int j = 0; j < linesPerFrame; ++j) {
        for (unsigned i = 0; i < xLength; ++i) {
            waveform[i + j * xLength] = scanStart + step * j;
            // stop at last x retrace
            if ((j >= linesPerFrame - 1) && (i >= xLength - X_RETRACE_LEN)) {
                break;
            }
        }
    }

    // Smooth Y transitions during each line's X retrace
    for (int j = 0; j < linesPerFrame - 1; ++j) {
        double yThis = scanStart + step * j;
        double yNext = scanStart + step * (j + 1);
        SplineInterpolate(X_RETRACE_LEN, yThis, yNext, 0, 0,
                          waveform + (j + 1) * xLength - X_RETRACE_LEN);
    }

    // Generate the rescan curve at end of frame
    if (X_RETRACE_LEN > 0) {
        double lastLineY = scanStart + step * (linesPerFrame - 1);
        SplineInterpolate(X_RETRACE_LEN, lastLineY, scanStart, 0, 0,
                          waveform + (linesPerFrame * xLength) -
                              X_RETRACE_LEN);
    }
}

/* Line clock pattern for NI DAQ to output from one of its digital IOs */
void GenerateLineClock(const struct WaveformParams *parameters,
                       uint8_t *lineClock) {
    uint32_t lineDelay = parameters->undershoot;
    uint32_t width = parameters->width;
    uint32_t height = parameters->height;

    uint32_t x_length = lineDelay + width + X_RETRACE_LEN;
    for (uint32_t j = 0; j < height; j++)
        for (uint32_t i = 0; i < x_length; i++)
            lineClock[i + j * x_length] =
                ((i >= lineDelay) && (i < lineDelay + width)) ? 1 : 0;
}

// High voltage right after a line acquisition is done
// like a line clock of reversed polarity
// specially for B&H FLIM application
void GenerateFLIMLineClock(const struct WaveformParams *parameters,
                           uint8_t *lineClockFLIM) {
    uint32_t lineDelay = parameters->undershoot;
    uint32_t width = parameters->width;
    uint32_t height = parameters->height;

    uint32_t x_length = lineDelay + width + X_RETRACE_LEN;
    for (uint32_t j = 0; j < height; j++)
        for (uint32_t i = 0; i < x_length; i++)
            lineClockFLIM[i + j * x_length] = (i >= lineDelay + width) ? 1 : 0;
}

// Frame clock for B&H FLIM
// High voltage at the end of the frame
void GenerateFLIMFrameClock(const struct WaveformParams *parameters,
                            uint8_t *frameClockFLIM) {
    uint32_t lineDelay = parameters->undershoot;
    uint32_t width = parameters->width;
    uint32_t height = parameters->height;

    uint32_t x_length = lineDelay + width + X_RETRACE_LEN;

    for (uint32_t j = 0; j < height; ++j)
        for (uint32_t i = 0; i < x_length; ++i)
            frameClockFLIM[i + j * x_length] =
                ((j == height - 1) && (i > lineDelay + width)) ? 1 : 0;
}

int32_t GetLineWaveformSize(const struct WaveformParams *parameters) {
    return parameters->undershoot + parameters->width + X_RETRACE_LEN;
}

int32_t GetClockWaveformSize(const struct WaveformParams *parameters) {
    uint32_t elementsPerLine = GetLineWaveformSize(parameters);
    uint32_t height = parameters->height;
    return elementsPerLine * height;
}

int32_t GetScannerWaveformSize(const struct WaveformParams *parameters) {
    uint32_t elementsPerLine = GetLineWaveformSize(parameters);
    uint32_t height = parameters->height;
    uint32_t yLen = height;
    return elementsPerLine * yLen; // including y retrace portion
}

int32_t
GetScannerWaveformSizeAfterLastPixel(const struct WaveformParams *parameters) {
    (void)parameters; // Unused
    return X_RETRACE_LEN;
}

int32_t GetParkWaveformSize(const struct WaveformParams *parameters) {
    (void)parameters; // Unused
    uint32_t elementsPerLine = X_RETRACE_LEN;
    return elementsPerLine;
}

/*
Generate X and Y waveforms in analog format (voltage) for a whole frame scan
Format: X|Y in a 1D array for NI DAQ to simultaneously output in two channels
Analog voltage range (-0.5V, 0.5V) at zoom 1
Including Y retrace waveform that moves the slow galvo back to its starting
position
*/
void GenerateGalvoWaveformFrame(const struct WaveformParams *parameters,
                                double *xyWaveformFrame) {
    uint32_t pixelsPerLine = parameters->width; // ROI size
    uint32_t linesPerFrame = parameters->height;
    uint32_t resolution = parameters->resolution;
    double zoom = parameters->zoom;
    uint32_t undershoot = parameters->undershoot;
    uint32_t xOffset = parameters->xOffset; // ROI offset
    uint32_t yOffset = parameters->yOffset;
    const double *m = parameters->xformMatrix;
    double tx = parameters->xformOffsetX;
    double ty = parameters->xformOffsetY;

    // Voltage ranges of the ROI
    double xStart = (-0.5 * resolution + xOffset) / (zoom * resolution);
    double yStart = (-0.5 * resolution + yOffset) / (zoom * resolution);
    double xEnd = xStart + pixelsPerLine / (zoom * resolution);
    double yEnd = yStart + linesPerFrame / (zoom * resolution);

    size_t xLength = undershoot + pixelsPerLine + X_RETRACE_LEN;
    size_t yLength = linesPerFrame;

    double *xWaveform = (double *)malloc(sizeof(double) * xLength);
    double *yWaveform = (double *)malloc(sizeof(double) * (yLength * xLength));
    GenerateXGalvoWaveform(pixelsPerLine, X_RETRACE_LEN, undershoot, xStart,
                           xEnd, xWaveform);
    GenerateYGalvoWaveform(linesPerFrame, X_RETRACE_LEN, xLength, yStart, yEnd,
                           yWaveform);

    for (unsigned j = 0; j < yLength; ++j) {
        for (unsigned i = 0; i < xLength; ++i) {
            double x = xWaveform[i];
            double y = yWaveform[i + j * xLength];
            xyWaveformFrame[i + j * xLength] = m[0] * x + m[1] * y + tx;
            xyWaveformFrame[i + j * xLength + yLength * xLength] =
                m[2] * x + m[3] * y + ty;
        }
    }

    // TODO When we are scanning multiple frames, the Y retrace can be
    // simultaneous with the last line's X retrace. (Spline interpolate
    // with zero slope at each end of retrace.)
    // TODO Simpler to use interleaved x,y format?

    free(xWaveform);
    free(yWaveform);
}

// Generate waveform from parking to start before one frame
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
    int32_t undershoot = parameters->undershoot;
    const double *m = parameters->xformMatrix;
    double tx = parameters->xformOffsetX;
    double ty = parameters->xformOffsetY;

    // Inverse-transform previous park voltage to logical space
    double xStart, yStart;
    InverseTransform2x2(m, tx, ty, parameters->prevXParkVoltage,
                        parameters->prevYParkVoltage, &xStart, &yStart);

    double xEnd =
        (-0.5 * resolution + xOffset - undershoot) / (zoom * resolution);
    double yEnd = (-0.5 * resolution + yOffset) / (zoom * resolution);

    size_t length = X_RETRACE_LEN;
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

//
// Fermat spiral scans
//

static const double GOLDEN_ANGLE = 2.39996322972865332; // M_PI * (3 - sqrt(5))

enum SpiralPhase {
    SPIRAL_PHASE_OUTWARD,
    SPIRAL_PHASE_PERIPH_CONN,
    SPIRAL_PHASE_INWARD,
    SPIRAL_PHASE_CENTER_CONN,
};

struct SpiralGenState {
    double c;
    double thetaMin;
    double thetaMax;
    int32_t samplesPerArm;
    int32_t centerConnSamples;
    double centerX;
    double centerY;
    double dtheta;
    double xformMatrix[4];
    double xformOffsetX;
    double xformOffsetY;

    int32_t armCycleIndex;
    enum SpiralPhase phase;
    int32_t sampleInPhase;

    double *connX;
    double *connY;
};

static void OutwardVelocity(const struct SpiralGenState *s, double theta,
                            double angle, double *dxdi, double *dydi) {
    double sqrtTh = sqrt(theta);
    double drdt = s->c / (2.0 * sqrtTh);
    double r = s->c * sqrtTh;
    *dxdi = (drdt * cos(angle) - r * sin(angle)) * s->dtheta;
    *dydi = (drdt * sin(angle) + r * cos(angle)) * s->dtheta;
}

static void InwardVelocity(const struct SpiralGenState *s, double theta,
                           double angle, double *dxdi, double *dydi) {
    double sqrtTh = sqrt(theta);
    double drdt = s->c / (2.0 * sqrtTh);
    double r = s->c * sqrtTh;
    // Inward: dangle/dtheta = -1, dtheta/di = -dtheta
    *dxdi = -(drdt * cos(angle) + r * sin(angle)) * s->dtheta;
    *dydi = -(drdt * sin(angle) - r * cos(angle)) * s->dtheta;
}

static double InwardPsi(const struct SpiralGenState *s) {
    return 2.0 * s->thetaMax + s->armCycleIndex * GOLDEN_ANGLE +
           SPIRAL_CONN_SAMPLES * s->dtheta;
}

static void ComputePeriphConn(struct SpiralGenState *s) {
    double outAngle = s->thetaMax + s->armCycleIndex * GOLDEN_ANGLE;
    double R = s->c * sqrt(s->thetaMax);
    double x0 = s->centerX + R * cos(outAngle);
    double y0 = s->centerY + R * sin(outAngle);
    double dx0, dy0;
    OutwardVelocity(s, s->thetaMax, outAngle, &dx0, &dy0);

    double psi = InwardPsi(s);
    double inAngle = -s->thetaMax + psi;
    double x1 = s->centerX + R * cos(inAngle);
    double y1 = s->centerY + R * sin(inAngle);
    double dx1, dy1;
    InwardVelocity(s, s->thetaMax, inAngle, &dx1, &dy1);

    SplineInterpolate(SPIRAL_CONN_SAMPLES, x0, x1, dx0, dx1, s->connX);
    SplineInterpolate(SPIRAL_CONN_SAMPLES, y0, y1, dy0, dy1, s->connY);
}

static void ComputeCenterConn(struct SpiralGenState *s) {
    double psi = InwardPsi(s);
    double inAngle = -s->thetaMin + psi;
    double rMin = s->c * sqrt(s->thetaMin);
    double x0 = s->centerX + rMin * cos(inAngle);
    double y0 = s->centerY + rMin * sin(inAngle);
    double dx0, dy0;
    InwardVelocity(s, s->thetaMin, inAngle, &dx0, &dy0);

    int32_t nextCycle = s->armCycleIndex + 1;
    double outAngle = s->thetaMin + nextCycle * GOLDEN_ANGLE;
    double x1 = s->centerX + rMin * cos(outAngle);
    double y1 = s->centerY + rMin * sin(outAngle);
    double dx1, dy1;
    OutwardVelocity(s, s->thetaMin, outAngle, &dx1, &dy1);

    SplineInterpolate(s->centerConnSamples, x0, x1, dx0, dx1, s->connX);
    SplineInterpolate(s->centerConnSamples, y0, y1, dy0, dy1, s->connY);
}

struct SpiralGenState *
CreateSpiralGenState(const struct SpiralWaveformParams *params) {
    struct SpiralGenState *s =
        (struct SpiralGenState *)calloc(1, sizeof(struct SpiralGenState));

    s->c = params->turnSpacing / sqrt(2.0 * M_PI);
    if (s->c <= 0.0)
        s->c = 1e-9;
    s->thetaMax = (params->radius / s->c) * (params->radius / s->c);

    double rMin = params->rMin;
    if (rMin < 1e-6)
        rMin = 1e-6;
    if (rMin > params->radius * 0.9)
        rMin = params->radius * 0.9;
    s->thetaMin = (rMin / s->c) * (rMin / s->c);

    double numTurns = (s->thetaMax - s->thetaMin) / (2.0 * M_PI);
    double samplesPerTurn =
        params->turnDurationMs * 1e-3 * SPIRAL_SAMPLE_RATE_HZ;
    s->samplesPerArm = (int32_t)(numTurns * samplesPerTurn);
    if (s->samplesPerArm < 2)
        s->samplesPerArm = 2;

    s->dtheta = (s->thetaMax - s->thetaMin) / (double)s->samplesPerArm;

    double delta_c = GOLDEN_ANGLE - 2.0 * (s->thetaMax - s->thetaMin) -
                     SPIRAL_CONN_SAMPLES * s->dtheta;
    delta_c = fmod(delta_c, 2.0 * M_PI);
    if (delta_c < 0.0)
        delta_c += 2.0 * M_PI;
    s->centerConnSamples = (int32_t)ceil(delta_c / s->dtheta);
    if (s->centerConnSamples < 2)
        s->centerConnSamples = 2;

    s->centerX = params->centerX;
    s->centerY = params->centerY;
    for (int i = 0; i < 4; ++i)
        s->xformMatrix[i] = params->xformMatrix[i];
    s->xformOffsetX = params->xformOffsetX;
    s->xformOffsetY = params->xformOffsetY;

    s->armCycleIndex = 0;
    s->phase = SPIRAL_PHASE_OUTWARD;
    s->sampleInPhase = 0;

    int32_t maxConn = SPIRAL_CONN_SAMPLES;
    if (s->centerConnSamples > maxConn)
        maxConn = s->centerConnSamples;
    s->connX = (double *)malloc(sizeof(double) * maxConn);
    s->connY = (double *)malloc(sizeof(double) * maxConn);

    return s;
}

void DestroySpiralGenState(struct SpiralGenState *state) {
    if (state) {
        free(state->connX);
        free(state->connY);
        free(state);
    }
}

void GenerateSpiralChunk(struct SpiralGenState *state, double *xyBuffer,
                         int32_t count) {
    const double *m = state->xformMatrix;
    double tx = state->xformOffsetX;
    double ty = state->xformOffsetY;

    for (int32_t i = 0; i < count; ++i) {
        double lx = 0.0, ly = 0.0;

        switch (state->phase) {
        case SPIRAL_PHASE_OUTWARD: {
            double theta =
                state->thetaMin + state->sampleInPhase * state->dtheta;
            double r = state->c * sqrt(theta);
            double angle = theta + state->armCycleIndex * GOLDEN_ANGLE;
            lx = state->centerX + r * cos(angle);
            ly = state->centerY + r * sin(angle);

            state->sampleInPhase++;
            if (state->sampleInPhase >= state->samplesPerArm) {
                ComputePeriphConn(state);
                state->phase = SPIRAL_PHASE_PERIPH_CONN;
                state->sampleInPhase = 0;
            }
            break;
        }
        case SPIRAL_PHASE_PERIPH_CONN: {
            lx = state->connX[state->sampleInPhase];
            ly = state->connY[state->sampleInPhase];

            state->sampleInPhase++;
            if (state->sampleInPhase >= SPIRAL_CONN_SAMPLES) {
                state->phase = SPIRAL_PHASE_INWARD;
                state->sampleInPhase = 0;
            }
            break;
        }
        case SPIRAL_PHASE_INWARD: {
            double theta =
                state->thetaMax - state->sampleInPhase * state->dtheta;
            double r = state->c * sqrt(theta);
            double psi = InwardPsi(state);
            double angle = -theta + psi;
            lx = state->centerX + r * cos(angle);
            ly = state->centerY + r * sin(angle);

            state->sampleInPhase++;
            if (state->sampleInPhase >= state->samplesPerArm) {
                ComputeCenterConn(state);
                state->phase = SPIRAL_PHASE_CENTER_CONN;
                state->sampleInPhase = 0;
            }
            break;
        }
        case SPIRAL_PHASE_CENTER_CONN: {
            lx = state->connX[state->sampleInPhase];
            ly = state->connY[state->sampleInPhase];

            state->sampleInPhase++;
            if (state->sampleInPhase >= state->centerConnSamples) {
                state->armCycleIndex++;
                state->phase = SPIRAL_PHASE_OUTWARD;
                state->sampleInPhase = 0;
            }
            break;
        }
        }

        xyBuffer[i] = m[0] * lx + m[1] * ly + tx;
        xyBuffer[i + count] = m[2] * lx + m[3] * ly + ty;
    }
}

int32_t GetSpiralArmCycleSamples(const struct SpiralGenState *state) {
    return 2 * state->samplesPerArm + SPIRAL_CONN_SAMPLES +
           state->centerConnSamples;
}

int32_t GetSpiralArmCycleIndex(const struct SpiralGenState *state) {
    return state->armCycleIndex;
}

bool SpiralGenStateAtCycleBoundary(const struct SpiralGenState *state) {
    return state->phase == SPIRAL_PHASE_OUTWARD && state->sampleInPhase == 0;
}

int32_t GetSpiralSamplesPerArm(const struct SpiralGenState *state) {
    return state->samplesPerArm;
}

int32_t GetSpiralCenterConnSamples(const struct SpiralGenState *state) {
    return state->centerConnSamples;
}

// Generate waveform from start to parking after one frame
void GenerateGalvoParkWaveform(const struct WaveformParams *parameters,
                               double *xyWaveformFrame) {
    uint32_t resolution = parameters->resolution;
    double zoom = parameters->zoom;
    uint32_t xOffset = parameters->xOffset;
    uint32_t yOffset = parameters->yOffset;
    int32_t undershoot = parameters->undershoot;
    int32_t xPark = parameters->xPark;
    int32_t yPark = parameters->yPark;
    const double *m = parameters->xformMatrix;
    double tx = parameters->xformOffsetX;
    double ty = parameters->xformOffsetY;

    double xStart =
        (-0.5 * resolution + xOffset - undershoot) / (zoom * resolution);
    double yStart = (-0.5 * resolution + yOffset) / (zoom * resolution);
    double xEnd = (-0.5 * resolution + xPark) / (zoom * resolution);
    double yEnd = (-0.5 * resolution + yPark) / (zoom * resolution);

    size_t length = X_RETRACE_LEN;
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
