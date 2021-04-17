#include "OScNIDAQDevicePrivate.h"
#include "Waveform.h"

#include <OpenScanDeviceLib.h>
#include <NIDAQmx.h>
#include <stdbool.h>
#include <string.h>


static OScDev_RichError *ConfigureScannerTiming(OScDev_Device *device, struct ScannerConfig *config, OScDev_Acquisition *acq);
static OScDev_RichError *WriteScannerOutput(OScDev_Device *device, struct ScannerConfig *config, OScDev_Acquisition *acq);


// Initialize, configure, and arm the scanner, whatever its current state
OScDev_RichError *SetUpScanner(OScDev_Device *device, struct ScannerConfig *config, OScDev_Acquisition *acq)
{
	OScDev_RichError *err;
	bool mustCommit = false;

	if (!config->aoTask)
	{
		err = CreateDAQmxError(DAQmxCreateTask("Scanner", &config->aoTask));
		if (err)
		{
			err = OScDev_Error_Wrap(err, "Failed to create scanner task");
			return err;
		}

		char aoTerminals[256];
		strncpy(aoTerminals, GetData(device)->deviceName, sizeof(aoTerminals) - 1);
		strncat(aoTerminals, "/ao0:1", sizeof(aoTerminals) - strlen(aoTerminals) - 1);

		err = CreateDAQmxError(DAQmxCreateAOVoltageChan(config->aoTask, aoTerminals,
			"Galvos", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		if (err)
		{
			err = OScDev_Error_Wrap(err, "Failed to create ao channels for scanner");
			goto error;
		}

		config->mustReconfigureTiming = true;
		config->mustRewriteOutput = true;
		mustCommit = true;
	}

	if (config->mustReconfigureTiming)
	{
		err = ConfigureScannerTiming(device, config, acq);
		if (err)
			goto error;
		config->mustReconfigureTiming = false;
		mustCommit = true;
	}

	if (config->mustRewriteOutput)
	{
		err = WriteScannerOutput(device, config, acq);
		if (err)
			goto error;
		config->mustRewriteOutput = false;
		mustCommit = true;
	}

	if (mustCommit)
	{
		err = CreateDAQmxError(DAQmxTaskControl(config->aoTask, DAQmx_Val_Task_Commit));
		if (err)
		{
			err = OScDev_Error_Wrap(err, "Failed to commit task for scanner");
			goto error;
		}
	}

	return OScDev_RichError_OK;

error:
	if (ShutdownScanner(device, config))
		OScDev_Log_Error(device, "Failed to clean up scanner task after error");
	return err;
}


// Remove all DAQmx configuration for the scanner
OScDev_RichError *ShutdownScanner(OScDev_Device *device, struct ScannerConfig *config)
{
	OScDev_RichError *err;
	if (config->aoTask)
	{
		err = CreateDAQmxError(DAQmxClearTask(config->aoTask));
		if (err)
		{
			err = OScDev_Error_Wrap(err, "Failed to clear scanner task");
			return err;
		}
		config->aoTask = 0;
	}
	return OScDev_RichError_OK;
}


OScDev_RichError *StartScanner(OScDev_Device *device, struct ScannerConfig *config)
{
	OScDev_RichError *err;
	err = CreateDAQmxError(DAQmxStartTask(config->aoTask));
	if (err)
	{
		err = OScDev_Error_Wrap(err, "Failed to start scanner task");
		ShutdownScanner(device, config); // Force re-setup next time
		return err;
	}
	return OScDev_RichError_OK;
}


OScDev_RichError *StopScanner(OScDev_Device *device, struct ScannerConfig *config)
{
	OScDev_RichError *err;
	err = CreateDAQmxError(DAQmxStopTask(config->aoTask));
	if (err)
	{
		err = OScDev_Error_Wrap(err, "Failed to stop scanner task");
		ShutdownScanner(device, config); // Force re-setup next time
		return err;
	}
	return OScDev_RichError_OK;
}


static OScDev_RichError *ConfigureScannerTiming(OScDev_Device *device, struct ScannerConfig *config, OScDev_Acquisition *acq)
{
	OScDev_RichError *err;
	double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
	uint32_t xOffset, yOffset, width, height;
	OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);

	uint32_t elementsPerLine = GetData(device)->lineDelay + width + X_RETRACE_LEN;
	uint32_t yLen = height + Y_RETRACE_LEN;
	int32 elementsPerFramePerChan = elementsPerLine * height;
	int32 totalElementsPerFramePerChan = elementsPerLine * yLen;

	err = CreateDAQmxError(DAQmxCfgSampClkTiming(config->aoTask, "",
		pixelRateHz / GetData(device)->binFactor,
		DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, totalElementsPerFramePerChan));
	if (err)
	{
		err = OScDev_Error_Wrap(err, "Failed to configure timing for scanner");
		return err;
	}

	return OScDev_RichError_OK;
}


static OScDev_RichError *WriteScannerOutput(OScDev_Device *device, struct ScannerConfig *config, OScDev_Acquisition *acq)
{
	OScDev_RichError *err;
	uint32_t resolution = OScDev_Acquisition_GetResolution(acq);
	double zoomFactor = OScDev_Acquisition_GetZoomFactor(acq);
	uint32_t xOffset, yOffset, width, height;
	OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);

	uint32_t elementsPerLine = GetData(device)->lineDelay + width + X_RETRACE_LEN;
	uint32_t yLen = height + Y_RETRACE_LEN;
	int32 elementsPerFramePerChan = elementsPerLine * height;  // without y retrace portion
	int32 totalElementsPerFramePerChan = elementsPerLine * yLen;   // including y retrace portion

	double *xyWaveformFrame = (double*)malloc(sizeof(double) * totalElementsPerFramePerChan * 2);

	err = GenerateGalvoWaveformFrame(resolution, zoomFactor,
		GetData(device)->lineDelay,
		xOffset, yOffset, width, height,
		GetData(device)->offsetXY[0],
		GetData(device)->offsetXY[1],
		xyWaveformFrame);
	if (err)
		return err;

	int32 numWritten = 0;
	err = CreateDAQmxError(DAQmxWriteAnalogF64(config->aoTask,
		totalElementsPerFramePerChan, FALSE, 10.0,
		DAQmx_Val_GroupByChannel, xyWaveformFrame, &numWritten, NULL));
	if (err)
	{
		err = OScDev_Error_Wrap(err, "Failed to write scanner waveforms");
		goto cleanup;
	}
	if (numWritten != totalElementsPerFramePerChan)
	{
		err = OScDev_Error_Wrap(err, "Failed to write complete scan waveform");
		goto cleanup;
	}

cleanup:
	free(xyWaveformFrame);
	return err;
}