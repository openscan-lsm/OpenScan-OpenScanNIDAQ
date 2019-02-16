#include "OScNIDAQDevicePrivate.h"

#include <OpenScanDeviceLib.h>
#include <NIDAQmx.h>

#include <math.h>
#include <stdio.h>


static int32 CreateDetectorTask(OScDev_Device *device, struct DetectorConfig *config);
static int32 ConfigureDetectorTiming(OScDev_Device *device, struct DetectorConfig *config);
static int32 ConfigureDetectorTrigger(OScDev_Device *device, struct DetectorConfig *config);
static int32 ConfigureDetectorCallback(OScDev_Device *device, struct DetectorConfig *config);
static int32 ReadLineCallback(TaskHandle taskHandle, int32 everyNsamplesEventType,
	uInt32 nSamples, void* callbackData);


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

	GetData(device)->totalRead = 0;

	return 0;

error:
	if (ShutdownDetector(device, config))
		OScDev_Log_Error(device, "Failed to clean up detector task after error");
	return nierr;
}


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

	// BUG: This should be AIVoltageRngs, but keeping old behavior for now
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

	// TODO Is this the best place to configure the input buffer?
	// See also ConfigureDetectorCallback()
	nierr = DAQmxCfgInputBuffer(config->aiTask,
		GetData(device)->numLinesToBuffer *
		GetData(device)->resolution *
		GetData(device)->binFactor *
		GetData(device)->numAIChannels);
	if (nierr)
	{
		LogNiError(device, nierr, "configuring input buffer for detector");
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


static int32 ConfigureDetectorCallback(OScDev_Device *device, struct DetectorConfig *config)
{
	int32 nierr = DAQmxRegisterEveryNSamplesEvent(config->aiTask,
		DAQmx_Val_Acquired_Into_Buffer,
		GetData(device)->resolution * GetData(device)->binFactor,
		0, ReadLineCallback, device);
	if (nierr)
	{
		LogNiError(device, nierr, "registering callback for detector");
		return nierr;
	}

	return 0;
}


// read from PMT line by line
// non-interlaced acquisition. 
// evary line data in format: Channel1 | Channel 2 | Channel 3 | ...
static int32 ReadLineCallback(TaskHandle taskHandle, int32 everyNsamplesEventType,
	uInt32 nSamples, void* callbackData)
{
	OScDev_Device *device = (OScDev_Device*)(callbackData);

	int32 readPerChan;
	int32_t prevPercentRead = -1;
	uInt32 totalSamplesPerLine = GetData(device)->numAIChannels * GetData(device)->resolution * GetData(device)->binFactor;
	int32 currLine = 1 + GetData(device)->totalRead / GetData(device)->numAIChannels / GetData(device)->binFactor / GetData(device)->resolution;
	// rawLineData format with GroupByChannel (non-interlaced) is:
	// CH1 pixel 1 meas 1..binFactor | CH1 p2 m1..binFactor | ... | CH1 pN m1..binFactor || CH2 p1 m1..binFactor |...
	int32 nierr = DAQmxReadAnalogF64(GetData(device)->detectorConfig.aiTask,
		-1, 10.0, DAQmx_Val_GroupByChannel,
		GetData(device)->rawLineData, totalSamplesPerLine, &readPerChan, NULL);
	if (nierr)
	{
		LogNiError(device, nierr, "reading from detector task");
		goto error;
	}

	if (readPerChan > 0)
	{
		//  append data line by line to the frame data array
		for (uint32_t i = 0; i < totalSamplesPerLine; i += GetData(device)->binFactor)
		{
			// pixel averaging
			GetData(device)->avgLineData[i / GetData(device)->binFactor] =
				GetData(device)->rawLineData[i] / GetData(device)->binFactor;

			for (unsigned j = 1; j < (unsigned)GetData(device)->binFactor; j++)
			{
				GetData(device)->avgLineData[i / GetData(device)->binFactor] +=
					(GetData(device)->rawLineData[i + j] / GetData(device)->binFactor);
			}

			// convert processed line and append to output image frame
			double avgSample = GetData(device)->avgLineData[i / GetData(device)->binFactor] /
				GetData(device)->inputVoltageRange * 32767.0;
			GetData(device)->imageData[i / GetData(device)->binFactor +
				GetData(device)->totalRead / GetData(device)->binFactor] =
				avgSample < 0 ? 0 : (uint16_t)avgSample;
		}

		GetData(device)->totalRead += (GetData(device)->numAIChannels * readPerChan); // update total elements acquired

		if (currLine % 128 == 0)
		{
			char msg[OScDev_MAX_STR_LEN + 1];
			snprintf(msg, OScDev_MAX_STR_LEN, "Read %d lines", currLine);
			OScDev_Log_Debug(device, msg);
		}
	}
	else
	{
		OScDev_Log_Error(device, "Callback received but no data read");
	}

	if (GetData(device)->totalRead == GetData(device)->resolution * totalSamplesPerLine)
	{
		GetData(device)->oneFrameScanDone = true;
		GetData(device)->totalRead = 0;
		OScDev_Log_Debug(device, "End of scanning one frame");
	}

	return 0;

error:
	if (GetData(device)->detectorConfig.aiTask)
		ShutdownDetector(device, GetData(device)->detectorConfig.aiTask);
	return nierr;
}