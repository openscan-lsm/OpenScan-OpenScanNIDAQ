#include "Waveform.h"

#include <math.h>
#include <stdlib.h>


inline int
VoltsToDACUnits(double p, double zoom, double galvoOffset, uint16_t *result)
{
	/*The DAC units run from -10V to 10V, pk-pk 60 optical degrees.  
	0V is at 32768.0.  
	3276.8 = 1V 
	0.33 V per optical degree 
	*/

	double scaled = round(p / zoom * 3276.8 + 32768.0 + (galvoOffset/3)*3276.8);
	if (scaled < 0 || scaled > UINT16_MAX)
		return -1;
	*result = (uint16_t)scaled;
	return 0;
}


int
GenerateScaledWaveforms(uint32_t resolution, double zoom, uint16_t *xScaled, uint16_t *yScaled,
	double galvoOffsetX, double galvoOffsetY)
{
	size_t xLength = X_UNDERSHOOT + resolution + X_RETRACE_LEN;
	size_t yLength = resolution;
	
	double *xWaveform = (double *)malloc(sizeof(double) * xLength);
	double *yWaveform = (double *)malloc(sizeof(double) * yLength);

	GenerateGalvoWaveform(resolution, X_RETRACE_LEN, X_UNDERSHOOT, -0.5, 0.5, xWaveform);
	GenerateGalvoWaveform(resolution, 0, 0, -0.5, 0.5, yWaveform);

	for (int i = 0; i < xLength; ++i) {
		if (VoltsToDACUnits(xWaveform[i], zoom, galvoOffsetX, &(xScaled[i])) != 0)
			return -1;
	}
	for (int j = 0; j < yLength; ++j) {
		if (VoltsToDACUnits(yWaveform[j], zoom, galvoOffsetY, &(yScaled[j])) != 0)
			return -1;
	}

	free(xWaveform);
	free(yWaveform);

	return 0;
}


void
GenerateGalvoWaveform(int32_t effectiveScanLen, int32_t retraceLen,
	int32_t undershootLen, double scanStart, double scanEnd, double *waveform)
{
	double scanAmplitude = scanEnd - scanStart;
	double step = scanAmplitude / (effectiveScanLen - 1);
	int32_t linearLen = undershootLen + effectiveScanLen;

	// Generate the linear scan curve
	double undershootStart = scanStart - undershootLen * step;
	for (int i = 0; i < linearLen; ++i)
	{
		waveform[i] = undershootStart + scanAmplitude * ((double)i / (effectiveScanLen - 1));
	}

	// Generate the rescan curve
	// Slope at start end end are both equal to the linear scan
	if (retraceLen > 0)
	{
		SplineInterpolate(retraceLen, scanEnd, undershootStart, step, step, waveform + linearLen);
	}
}


void SplineInterpolate(int32_t n, double yFirst, double yLast,
	double slopeFirst, double slopeLast, double* result)
{
	double c[4];

	c[0] = slopeFirst / (n*n) + 2.0 / (n*n*n)*yFirst + slopeLast / (n*n) - 2.0 / (n*n*n)*yLast;
	c[1] = 3.0 / (n*n)*yLast - slopeLast / n - 2.0 / n*slopeFirst - 3.0 / (n*n)*yFirst;
	c[2] = slopeFirst;
	c[3] = yFirst;

	for (int32_t x = 0; x < n; x++)
	{
		result[x] = c[0] * x*x*x + c[1] * x*x + c[2] * x + c[3];
	}
}

/* Line clock pattern for NI DAQ to output from one of its digital IOs */
int GenerateLineClock(uint32_t x_resolution, uint32_t numScanLines, uint8_t * lineClock)
{
	uint32_t x_length = X_UNDERSHOOT + x_resolution + X_RETRACE_LEN;
	for (uint32_t j = 0; j < numScanLines; j++)
		for (uint32_t i = 0; i < x_length; i++)
			lineClock[i + j*x_length] =
			((i >= X_UNDERSHOOT) && (i < X_UNDERSHOOT + x_resolution)) ? 1 : 0;

	return 0;
}

// High voltage right after a line acquisition is done
// like a line clock of reversed polarity
// specially for B&H FLIM application
int GenerateFLIMLineClock(uint32_t x_resolution, uint32_t numScanLines, uint8_t * lineClockFLIM)
{
	uint32_t x_length = X_UNDERSHOOT + x_resolution + X_RETRACE_LEN;
	for (uint32_t j = 0; j < numScanLines; j++)
		for (uint32_t i = 0; i < x_length; i++)
			lineClockFLIM[i + j*x_length] = (i >= X_UNDERSHOOT + x_resolution) ? 1 : 0;

	return 0;
}

// Frame clock for B&H FLIM
// High voltage at the end of the frame
int GenerateFLIMFrameClock(uint32_t x_resolution, uint32_t numScanLines, uint8_t * frameClockFLIM)
{
	uint32_t x_length = X_UNDERSHOOT + x_resolution + X_RETRACE_LEN;
	uint32_t y_length = numScanLines;

	for (uint32_t j = 0; j < y_length; ++j)
		for (uint32_t i = 0; i < x_length; ++i)
			frameClockFLIM[i + j * x_length] =
			((j == numScanLines - 1) && (i > X_UNDERSHOOT + x_resolution)) ? 1 : 0;

	return 0;
}


/*
Generate X and Y waveforms in analog format (voltage) for a whole frame scan
Format: X|Y in a 1D array for NI DAQ to simultaneously output in two channels
Analog voltage range (-0.5V, 0.5V) at zoom 1
Including Y retrace waveform that moves the slow galvo back to its starting position
*/
int
GenerateGalvoWaveformFrame(uint32_t resolution, double zoom, double galvoOffsetX, double galvoOffsetY, double * xyWaveformFrame)
{
	size_t xLength = X_UNDERSHOOT + resolution + X_RETRACE_LEN;
	size_t numScanLines = resolution;  // num of scan lines
	size_t yLength = numScanLines + Y_RETRACE_LEN;
	double *xWaveform = (double *)malloc(sizeof(double) * xLength);
	double *yWaveform = (double *)malloc(sizeof(double) * yLength);
	GenerateGalvoWaveform(resolution, X_RETRACE_LEN, X_UNDERSHOOT, -0.5 / zoom, 0.5 / zoom, xWaveform);
	GenerateGalvoWaveform(resolution, Y_RETRACE_LEN, 0, -0.5 / zoom, 0.5 / zoom, yWaveform);

	// convert to optical degree assuming 10V equal to 30 optical degree
	double offsetXinDegree = galvoOffsetX / 3.0;
	double offsetYinDegree = galvoOffsetY / 3.0;

	// effective scan waveform for a whole frame
	for (unsigned j = 0; j < yLength; ++j)
	{
		for (unsigned i = 0; i < xLength; ++i)
		{
			// first half is X waveform,
			// x line scan repeated yLength times (sawteeth) 
			// galvo x stays at starting position after one frame is scanned
			xyWaveformFrame[i + j*xLength] = (j < numScanLines) ?
				(xWaveform[i] + offsetXinDegree) : (xWaveform[0] + offsetXinDegree);
			//xyWaveformFrame[i + j*xLength] = xWaveform[i];
			// second half is Y waveform
			// at each x (fast) scan line, y value is constant
			// effectively y retrace takes (Y_RETRACE_LENGTH * xLength) steps
			xyWaveformFrame[i + j*xLength + yLength*xLength] = (yWaveform[j] + offsetYinDegree);
		}
	}

	free(xWaveform);
	free(yWaveform);

	return 0;
}