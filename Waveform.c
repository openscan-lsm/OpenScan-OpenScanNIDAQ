#include "Waveform.h"

#include <math.h>
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

    // Generate the rescan curve at end of frame
    if (X_RETRACE_LEN > 0) {
        SplineInterpolate(X_RETRACE_LEN, scanEnd, scanStart, 0, 0,
                          waveform + (linesPerFrame * xLength) -
                              X_RETRACE_LEN);
    }
}

/* Line clock pattern for NI DAQ to output from one of its digital IOs */
OScDev_RichError *GenerateLineClock(const struct WaveformParams *parameters,
                                    uint8_t *lineClock) {
    uint32_t lineDelay = parameters->undershoot;
    uint32_t width = parameters->width;
    uint32_t height = parameters->height;

    uint32_t x_length = lineDelay + width + X_RETRACE_LEN;
    for (uint32_t j = 0; j < height; j++)
        for (uint32_t i = 0; i < x_length; i++)
            lineClock[i + j * x_length] =
                ((i >= lineDelay) && (i < lineDelay + width)) ? 1 : 0;

    return OScDev_RichError_OK;
}

// High voltage right after a line acquisition is done
// like a line clock of reversed polarity
// specially for B&H FLIM application
OScDev_RichError *
GenerateFLIMLineClock(const struct WaveformParams *parameters,
                      uint8_t *lineClockFLIM) {
    uint32_t lineDelay = parameters->undershoot;
    uint32_t width = parameters->width;
    uint32_t height = parameters->height;

    uint32_t x_length = lineDelay + width + X_RETRACE_LEN;
    for (uint32_t j = 0; j < height; j++)
        for (uint32_t i = 0; i < x_length; i++)
            lineClockFLIM[i + j * x_length] = (i >= lineDelay + width) ? 1 : 0;

    return OScDev_RichError_OK;
}

// Frame clock for B&H FLIM
// High voltage at the end of the frame
OScDev_RichError *
GenerateFLIMFrameClock(const struct WaveformParams *parameters,
                       uint8_t *frameClockFLIM) {
    uint32_t lineDelay = parameters->undershoot;
    uint32_t width = parameters->width;
    uint32_t height = parameters->height;

    uint32_t x_length = lineDelay + width + X_RETRACE_LEN;

    for (uint32_t j = 0; j < height; ++j)
        for (uint32_t i = 0; i < x_length; ++i)
            frameClockFLIM[i + j * x_length] =
                ((j == height - 1) && (i > lineDelay + width)) ? 1 : 0;

    return OScDev_RichError_OK;
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
OScDev_RichError *
GenerateGalvoWaveformFrame(const struct WaveformParams *parameters,
                           double *xyWaveformFrame) {
    uint32_t pixelsPerLine = parameters->width; // ROI size
    uint32_t linesPerFrame = parameters->height;
    uint32_t resolution = parameters->resolution;
    double zoom = parameters->zoom;
    uint32_t undershoot = parameters->undershoot;
    uint32_t xOffset = parameters->xOffset; // ROI offset
    uint32_t yOffset = parameters->yOffset;
    double galvoOffsetX = parameters->galvoOffsetX; // Adjustment Offset
    double galvoOffsetY = parameters->galvoOffsetY;

    // Voltage ranges of the ROI
    double xStart = (-0.5 * resolution + xOffset) / (zoom * resolution);
    double yStart = (-0.5 * resolution + yOffset) / (zoom * resolution);
    double xEnd = xStart + pixelsPerLine / (zoom * resolution);
    double yEnd = yStart + linesPerFrame / (zoom * resolution);

    size_t xLength = undershoot + pixelsPerLine + X_RETRACE_LEN;
    size_t yLength = linesPerFrame;

    double *xWaveform = (double *)malloc(sizeof(double) * xLength);
    double *yWaveform =
        (double *)malloc(sizeof(double) * (yLength * xLength)); // change size
    GenerateXGalvoWaveform(pixelsPerLine, X_RETRACE_LEN, undershoot, xStart,
                           xEnd, xWaveform);
    GenerateYGalvoWaveform(linesPerFrame, X_RETRACE_LEN, xLength, yStart, yEnd,
                           yWaveform);

    // convert to optical degree assuming 10V equal to 30 optical degree
    // TODO We shouldn't make such an assumption! Also I think the variable
    // names are the other way around ("inDegree" means "in volts" here).
    double offsetXinDegree = galvoOffsetX / 3.0;
    double offsetYinDegree = galvoOffsetY / 3.0;

    // effective scan waveform for a whole frame
    for (unsigned j = 0; j < yLength; ++j) {
        for (unsigned i = 0; i < xLength; ++i) {
            // first half is X waveform,
            // x line scan repeated yLength times (sawteeth)
            // galvo x stays at starting position after one frame is scanned
            xyWaveformFrame[i + j * xLength] = xWaveform[i] + offsetXinDegree;

            // xyWaveformFrame[i + j*xLength] = xWaveform[i];
            //  second half is Y waveform
            //  at each x (fast) scan line, y value is constant
            //  effectively y retrace takes (Y_RETRACE_LENGTH * xLength) steps
            xyWaveformFrame[i + j * xLength + yLength * xLength] =
                yWaveform[i + j * xLength] + offsetYinDegree;
        }
    }

    // TODO When we are scanning multiple frames, the Y retrace can be
    // simultaneous with the last line's X retrace. (Spline interpolate
    // with zero slope at each end of retrace.)
    // TODO Simpler to use interleaved x,y format?

    free(xWaveform);
    free(yWaveform);

    return OScDev_RichError_OK;
}

// Generate waveform from parking to start before one frame
OScDev_RichError *
GenerateGalvoUnparkWaveform(const struct WaveformParams *parameters,
                            double *xyWaveformFrame) {
    uint32_t resolution = parameters->resolution;
    double zoom = parameters->zoom;
    uint32_t xOffset = parameters->xOffset; // ROI offset
    uint32_t yOffset = parameters->yOffset;
    double galvoOffsetX = parameters->galvoOffsetX; // Adjustment Offset
    double galvoOffsetY = parameters->galvoOffsetY;
    int32_t undershoot = parameters->undershoot;
    double xParkVoltage = parameters->prevXParkVoltage;
    double yParkVoltage = parameters->prevYParkVoltage;

    // Voltage ranges of the ROI
    double xStart = xParkVoltage;
    double yStart = yParkVoltage;
    double xEnd =
        (-0.5 * resolution + xOffset - undershoot) / (zoom * resolution);
    double yEnd = (-0.5 * resolution + yOffset) / (zoom * resolution);

    size_t length = X_RETRACE_LEN;
    double *xWaveform = (double *)malloc(sizeof(double) * length);
    double *yWaveform = (double *)malloc(sizeof(double) * length);

    SplineInterpolate((int32_t)length, xStart, xEnd, 0, 0, xWaveform);
    SplineInterpolate((int32_t)length, yStart, yEnd, 0, 0, yWaveform);

    double offsetXinDegree = galvoOffsetX / 3.0;
    double offsetYinDegree = galvoOffsetY / 3.0;

    // effective scan waveform for a whole frame
    for (unsigned i = 0; i < length; ++i) {
        // first half is X waveform,
        // x line scan repeated yLength times (sawteeth)
        xyWaveformFrame[i] = xWaveform[i] + offsetXinDegree;

        // second half is Y waveform
        xyWaveformFrame[i + length] = yWaveform[i] + offsetYinDegree;
    }

    free(xWaveform);
    free(yWaveform);

    return OScDev_RichError_OK;
}

// Generate waveform from start to parking after one frame
OScDev_RichError *
GenerateGalvoParkWaveform(const struct WaveformParams *parameters,
                          double *xyWaveformFrame) {
    uint32_t resolution = parameters->resolution;
    double zoom = parameters->zoom;
    uint32_t xOffset = parameters->xOffset; // ROI offset
    uint32_t yOffset = parameters->yOffset;
    double galvoOffsetX = parameters->galvoOffsetX; // Adjustment Offset
    double galvoOffsetY = parameters->galvoOffsetY;
    int32_t undershoot = parameters->undershoot;
    int32_t xPark = parameters->xPark;
    int32_t yPark = parameters->yPark;

    // Voltage ranges of the ROI
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

    double offsetXinDegree = galvoOffsetX / 3.0;
    double offsetYinDegree = galvoOffsetY / 3.0;

    // effective scan waveform for a whole frame
    for (unsigned i = 0; i < length; ++i) {
        // first half is X waveform,
        // x line scan repeated yLength times (sawteeth)
        // galvo x stays at starting position after one frame is scanned
        xyWaveformFrame[i] = xWaveform[i] + offsetXinDegree;

        // xyWaveformFrame[i + j*xLength] = xWaveform[i];
        //  second half is Y waveform
        xyWaveformFrame[i + length] = yWaveform[i] + offsetYinDegree;
    }

    free(xWaveform);
    free(yWaveform);

    return OScDev_RichError_OK;
}
