#include "OScNIDAQDevicePrivate.h"

#include <OpenScanDeviceLib.h>
#include <NIDAQmx.h>

#include <math.h>
#include <stdio.h>


static int32 CreateDetectorTask(OScDev_Device *device, struct DetectorConfig *config);
static int32 ConfigureDetectorTiming(OScDev_Device *device, struct DetectorConfig *config);
static int32 ConfigureDetectorTrigger(OScDev_Device *device, struct DetectorConfig *config);
static int32 ConfigureDetectorCallback(OScDev_Device *device, struct DetectorConfig *config);
static int32 CVICALLBACK DetectorDataCallback(TaskHandle taskHandle,
	int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData);
static int32 HandleRawData(OScDev_Device *device);


// Initialize, configure, and arm the detector, whatever its current state
int32 SetUpDetector(OScDev_Device *device, struct DetectorConfig *config)
{
	bool mustCommit = false;

	int32 nierr;
	if (!config->aiTask)
	{
		nierr = CreateDetectorTask(device, config);
		if (nierr)
			return nierr;
		config->mustReconfigureTiming = true;
		config->mustReconfigureTrigger = true;
		config->mustReconfigureCallback = true;
		mustCommit = true;
	}

	if (config->mustReconfigureTiming)
	{
		nierr = ConfigureDetectorTiming(device, config);
		if (nierr)
			goto error;
		config->mustReconfigureTiming = false;
		mustCommit = true;
	}

	if (config->mustReconfigureTrigger)
	{
		nierr = ConfigureDetectorTrigger(device, config);
		if (nierr)
			goto error;
		config->mustReconfigureTrigger = false;
		mustCommit = true;
	}

	if (config->mustReconfigureCallback)
	{
		nierr = ConfigureDetectorCallback(device, config);
		if (nierr)
			goto error;
		config->mustReconfigureCallback = false;
		mustCommit = true;
	}

	if (mustCommit)
	{
		nierr = DAQmxTaskControl(config->aiTask, DAQmx_Val_Task_Commit);
		if (nierr)
		{
			LogNiError(device, nierr, "committing task for detector");
			goto error;
		}
	}

	return 0;

error:
	if (ShutdownDetector(device, config))
		OScDev_Log_Error(device, "Failed to clean up detector task after error");
	return nierr;
}


// Remove all DAQmx configuration for the detector
// This can be called to force task recreation the next time the detector is
// armed
int32 ShutdownDetector(OScDev_Device *device, struct DetectorConfig *config)
{
	if (config->aiTask)
	{
		int32 nierr = DAQmxClearTask(config->aiTask);
		if (nierr)
		{
			LogNiError(device, nierr, "clearing detector task");
			return nierr;
		}
		config->aiTask = 0;
	}
	return 0;
}


int32 StartDetector(OScDev_Device *device, struct DetectorConfig *config)
{
	int32 nierr;
	nierr = DAQmxStartTask(config->aiTask);
	if (nierr)
	{
		LogNiError(device, nierr, "starting detector task");
		ShutdownDetector(device, config); // Force re-setup next time
		return nierr;
	}
	return 0;
}


int32 StopDetector(OScDev_Device *device, struct DetectorConfig *config)
{
	// The task may have been cleared in the case of an error
	if (!config->aiTask)
		return 0;

	int32 nierr;
	nierr = DAQmxStopTask(config->aiTask);
	if (nierr)
	{
		LogNiError(device, nierr, "stopping detector task");
		ShutdownDetector(device, config); // Force re-setup next time
		return nierr;
	}
	return 0;
}


static int32 GetAIVoltageRange(OScDev_Device *device, double *minVolts, double *maxVolts)
{
	float64 ranges[2 * 64];
	memset(ranges, 0, sizeof(ranges));

	// TODO How does this relate to the setting "Input Voltage Range"?
	// BUG: This should be AIVoltageRngs, but keeping existing behavior for now
	int32 nierr = DAQmxGetDevAOVoltageRngs(GetData(device)->deviceName, ranges,
		sizeof(ranges) / sizeof(float64));
	if (nierr)
	{
		LogNiError(device, nierr, "getting voltage range for detector");
		return nierr;
	}

	*minVolts = INFINITY;
	*maxVolts = -INFINITY;
	for (int i = 0; i < sizeof(ranges) / sizeof(float64) / 2; ++i)
	{
		if (ranges[2 * i] == 0.0 && ranges[2 * i + 1] == 0.0)
		{
			if (i == 0)
				return -1; // Unlikely error
			break;
		}

		// Find the range with the highest max voltage
		// FIXME This is not robust: there may be multiple ranges with the same
		// maximum but different minima, which would change the sample value ranges
		if (ranges[2 * i + 1] > *maxVolts)
		{
			*minVolts = ranges[2 * i];
			*maxVolts = ranges[2 * i + 1];
		}
	}
	return 0;
}


static int32 CreateDetectorTask(OScDev_Device *device, struct DetectorConfig *config)
{
	int32 nierr;
	nierr = DAQmxCreateTask("Detector", &config->aiTask);
	if (nierr)
	{
		LogNiError(device, nierr, "creating detector task");
		return nierr;
	}

	char aiPorts[1024] = "";
	for (size_t i = 0; i < GetData(device)->channelCount; ++i)
	{
		char port[256];
		sm_get(GetData(device)->channelMap_,
			GetData(device)->selectedDispChan_[i],
			port, sizeof(port));

		if (strlen(aiPorts) > 0)
			strncat(aiPorts, ",", sizeof(aiPorts) - strlen(aiPorts) - 1);
		strncat(aiPorts, port, sizeof(aiPorts) - strlen(aiPorts) - 1);
	}

	double minVolts, maxVolts;
	nierr = GetAIVoltageRange(device, &minVolts, &maxVolts);
	if (nierr)
		goto error;

	nierr = DAQmxCreateAIVoltageChan(config->aiTask,
		aiPorts, "",
		DAQmx_Val_Cfg_Default,
		minVolts, maxVolts,
		DAQmx_Val_Volts, NULL);
	if (nierr)
	{
		LogNiError(device, nierr, "creating ai channel for detector");
		goto error;
	}

	nierr = DAQmxGetReadNumChans(config->aiTask, &GetData(device)->numAIChannels);
	if (nierr)
	{
		LogNiError(device, nierr, "getting number of ai channels for detector");
		goto error;
	}

	return 0;

error:
	if (ShutdownDetector(device, config))
		OScDev_Log_Error(device, "Failed to clean up detector task after error");
	return nierr;
}


static int32 ConfigureDetectorTiming(OScDev_Device *device, struct DetectorConfig *config)
{
	int32 nierr = DAQmxCfgSampClkTiming(config->aiTask,
		"", 1E6*GetData(device)->scanRate,
		DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
		GetData(device)->resolution * GetData(device)->binFactor);
	if (nierr)
	{
		LogNiError(device, nierr, "configuring timing for detector");
		return nierr;
	}

	return 0;
}


static int32 ConfigureDetectorTrigger(OScDev_Device *device, struct DetectorConfig *config)
{
	// For now we hard-code the trigger to PFI12, which is the output of CTR0

	// Alternative: virtually connect counter (line clock) output terminal and
	// acquisition triggerIn terminal without physical wiring:
	//nierr = DAQmxConnectTerms("/PXI1Slot2/Ctr0InternalOutput", "/PXI1Slot2/PFI8", DAQmx_Val_DoNotInvertPolarity);
	//nierr = DAQmxCfgDigEdgeStartTrig(acqTaskHandle_, "/PXI1Slot2/PFI8", DAQmx_Val_Rising);

	char triggerSource[256] = "/";
	strncat(triggerSource, GetData(device)->deviceName,
		sizeof(triggerSource) - strlen(triggerSource) - 1);
	strncat(triggerSource, "/PFI12",
		sizeof(triggerSource) - strlen(triggerSource) - 1);

	int32 nierr = DAQmxCfgDigEdgeStartTrig(config->aiTask,
		triggerSource, DAQmx_Val_Rising);
	if (nierr)
	{
		LogNiError(device, nierr, "setting start trigger for detector task");
		return nierr;
	}

	nierr = DAQmxSetStartTrigRetriggerable(config->aiTask, 1);
	if (nierr)
	{
		LogNiError(device, nierr, "setting detector task retriggerable");
		return nierr;
	}
	return 0;
}


static int32 UnconfigureDetectorCallback(OScDev_Device *device, struct DetectorConfig *config)
{
	int32 nierr = DAQmxRegisterEveryNSamplesEvent(config->aiTask,
		DAQmx_Val_Acquired_Into_Buffer,
		0, 0, NULL, NULL);
	if (nierr)
	{
		LogNiError(device, nierr, "unregistering callback for detector");
		return nierr;
	}
	return 0;
}


static int32 ConfigureDetectorCallback(OScDev_Device *device, struct DetectorConfig *config)
{
	int32 nierr;

	// Registering the callback in DAQmx is not idempotent, so we need to
	// clear any existing callback.
	nierr = UnconfigureDetectorCallback(device, config);
	if (nierr)
		return nierr;

	// TODO: It probably makes sense to scale the buffer based on the time
	// duration of data it holds (e.g. 500 ms), rather than the number of
	// lines. We could still provide a user-settable scaling factor.

	uint32_t pixelsPerLine = GetData(device)->resolution;
	uint32_t pixelsPerFrame = pixelsPerLine * GetData(device)->resolution;
	uint32_t samplesPerChanPerLine = pixelsPerLine * GetData(device)->binFactor;
	uint32_t numChannels = GetData(device)->numAIChannels;
	size_t bufferSize = GetData(device)->numLinesToBuffer *
		samplesPerChanPerLine *
		numChannels;

	char msg[1024];
	snprintf(msg, sizeof(msg) - 1, "Using DAQmx input buffer of size %zd", bufferSize);
	OScDev_Log_Debug(device, msg);

	nierr = DAQmxCfgInputBuffer(config->aiTask, (uInt32)bufferSize);
	if (nierr)
	{
		LogNiError(device, nierr, "configuring input buffer for detector");
		return nierr;
	}

	// Allocate buffer into which we read data. Set it to be large enough
	// to read all available data from the input buffer in one go.
	GetData(device)->rawDataCapacity = bufferSize;
	GetData(device)->rawDataSize = 0;
	GetData(device)->rawDataBuffer = realloc(GetData(device)->rawDataBuffer,
		sizeof(float64) * GetData(device)->rawDataCapacity);

	// Allocate frame buffers for the enabled channels
	for (uint32_t ch = 0; ch < numChannels; ++ch)
	{
		GetData(device)->frameBuffers[ch] = realloc(
			GetData(device)->frameBuffers[ch],
			sizeof(uint16_t) * pixelsPerFrame);
	}
	// Free the frame buffers for unused channels
	for (uint32_t ch = numChannels; ch < OSc_Total_Channel_Num; ++ch)
	{
		free(GetData(device)->frameBuffers[ch]);
		GetData(device)->frameBuffers[ch] = NULL;
	}

	// Set DAQmxRead*() with DAQmx_Val_Auto to immediately return all
	// available samples instead of waiting for the requested number of
	// samples to become available.
	nierr = DAQmxSetReadReadAllAvailSamp(config->aiTask, TRUE);
	if (nierr)
	{
		LogNiError(device, nierr,
			"setting the Read All Available Samples property for the detector task");
		return nierr;
	}

	// TODO: It probably makes sense to scale the callback frequency so that
	// it is called at roughly 10 Hz. For now it is called once per line.

	nierr = DAQmxRegisterEveryNSamplesEvent(config->aiTask,
		DAQmx_Val_Acquired_Into_Buffer,
		samplesPerChanPerLine,
		0, DetectorDataCallback, device);
	if (nierr)
	{
		LogNiError(device, nierr, "registering callback for detector");
		return nierr;
	}

	return 0;
}


static int32 DetectorDataCallback(TaskHandle taskHandle,
	int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData)
{
	OScDev_Device *device = (OScDev_Device*)(callbackData);

	if (taskHandle != GetData(device)->detectorConfig.aiTask)
		return 0;
	if (everyNsamplesEventType != DAQmx_Val_Acquired_Into_Buffer)
		return 0;

	char msg[1024];
	snprintf(msg, sizeof(msg) - 1, "Detector callback (%d samples)", nSamples);
	OScDev_Log_Debug(device, msg);

	uint32_t numChannels = GetData(device)->numAIChannels;
	int32 nierr;

	// TODO We should use DAQmxReadBinaryU16, so that users have access to raw
	// ADC values. This is often critical for correct interpretation or
	// processing of the data.

	// Read all available samples without waiting (because we have set the
	// Read All Available Samples property on the task)
	int32 samplesPerChanRead;
	nierr = DAQmxReadAnalogF64(taskHandle,
		DAQmx_Val_Auto,
		0.0,
		DAQmx_Val_GroupByScanNumber,
		GetData(device)->rawDataBuffer + GetData(device)->rawDataSize,
		(uInt32)(GetData(device)->rawDataCapacity - GetData(device)->rawDataSize),
		&samplesPerChanRead,
		NULL);
	if (nierr == DAQmxErrorTimeoutExceeded)
	{
		OScDev_Log_Error(device, "Error: DAQ read data timeout");
		return 0;
	}

	if (nierr)
	{
		LogNiError(device, nierr, "reading detector samples");
		goto error;
	}
	
	if (samplesPerChanRead == 0)
	{
		OScDev_Log_Error(device, "Error: DAQ failed to read any sample");
		return 0;
	}

	GetData(device)->rawDataSize += samplesPerChanRead * numChannels;

	nierr = HandleRawData(device);
	if (nierr)
		goto error;

	return 0;

error:
	if (GetData(device)->detectorConfig.aiTask)
		ShutdownDetector(device, &GetData(device)->detectorConfig);
	return nierr;
}


// Process data in rawDataBuffer and place the result into frameBuffers
static int32 HandleRawData(OScDev_Device *device)
{
	// Some amount of data (rawDataSize samples) is in the rawDataBuffer.
	// Since we have C channels and are binning every B samples per channel,
	// we can only process a multiple of (C * B) samples at a time. We
	// shift any remaining samples to the front of rawDataBuffer so that they
	// can be handled when more data is available.

	size_t availableSamples = GetData(device)->rawDataSize;
	uint32_t numChannels = GetData(device)->numAIChannels;
	uint32_t binFactor = GetData(device)->binFactor;
	size_t leftoverSamples = availableSamples % (numChannels * binFactor);
	size_t samplesToProcess = availableSamples - leftoverSamples;
	size_t pixelsToProducePerChan = samplesToProcess / (numChannels * binFactor);

	double inputVoltageRange = GetData(device)->inputVoltageRange;

	float64 *rawDataBuffer = GetData(device)->rawDataBuffer;

	// Process raw data and fill in frame buffers
	for (size_t p = 0; p < pixelsToProducePerChan; ++p)
	{
		size_t rawPixelStart = p * numChannels * binFactor;
		size_t pixelIndex = GetData(device)->framePixelsFilled++;

		for (size_t ch = 0; ch < numChannels; ++ch)
		{
			size_t rawChannelStart = rawPixelStart + ch;

			double sum = 0.0;
			for (size_t s = 0; s < binFactor; ++s)
				sum += rawDataBuffer[rawChannelStart + s];
			double avg = sum / binFactor;

			// Q: Why is the maximum 32767, not 65535?
			// TODO In any case, this scaling may introduce nasty artifacts in the
			// histogram; we should be using the raw 16-bit samples, as commented
			// in DetectorDataCallback().
			double scaled = 32767.0 * avg / inputVoltageRange;
			uint16_t pixel = scaled >= 0.0 ? (uint16_t)scaled : 0;

			GetData(device)->frameBuffers[ch][pixelIndex] = pixel;
		}
	}

	// Shift the leftover raw samples to the front of the buffer for future
	// consumption
	memmove(rawDataBuffer, rawDataBuffer + samplesToProcess,
		sizeof(float64) * leftoverSamples);
	GetData(device)->rawDataSize = leftoverSamples;

	size_t pixelsPerFrame = GetData(device)->resolution *
		GetData(device)->resolution;
	char msg[OScDev_MAX_STR_LEN + 1];
	snprintf(msg, OScDev_MAX_STR_LEN, "Read %zd pixels", GetData(device)->framePixelsFilled);
	OScDev_Log_Debug(device, msg);

	if (GetData(device)->framePixelsFilled == pixelsPerFrame)
	{
		// TODO This method of communication is unreliable without a mutex
		// TODO But we should use a condition variable in any case
		GetData(device)->oneFrameScanDone = true;

		// TODO This reset should occur at start of frame
		GetData(device)->framePixelsFilled = 0;
	}

	return 0;
}