#include "OScNIDAQDevicePrivate.h"
#include "Waveform.h"

#include <OpenScanDeviceLib.h>
#include <NIDAQmx.h>


static OScDev_RichError *CreateClockTasks(OScDev_Device *device, struct ClockConfig *config, OScDev_Acquisition *acq);
static OScDev_RichError *ConfigureClockTiming(OScDev_Device *device, struct ClockConfig *config, OScDev_Acquisition *acq);
static OScDev_RichError *ConfigureClockTriggers(OScDev_Device *device, struct ClockConfig *config);
static OScDev_RichError *WriteClockOutput(OScDev_Device *device, struct ClockConfig *config, OScDev_Acquisition *acq);


// Initialize, configure, and arm the clock, whatever its current state
OScDev_RichError *SetUpClock(OScDev_Device *device, struct ClockConfig *config, OScDev_Acquisition *acq)
{
	OScDev_RichError* err;
	int32 nierr;

	bool mustCommit = false;

	if (!config->doTask || !config->lineCtrTask)
	{
		// In case one of the two tasks exists
		err = ShutdownClock(device, config);
		if (err)
			return err;

		err = CreateClockTasks(device, config, acq);
		if (err)
			return err;
		config->mustReconfigureTiming = true;
		config->mustReconfigureTriggers = true;
		config->mustRewriteOutput = true;
	}

	if (config->mustReconfigureTiming)
	{
		err = ConfigureClockTiming(device, config, acq);
		if (err)
			goto error;
			
		config->mustReconfigureTiming = false;
		mustCommit = true;
	}

	if (config->mustReconfigureTriggers)
	{
		err = ConfigureClockTriggers(device, config);
		if (err)
			goto error;
			
		config->mustReconfigureTriggers = false;
		mustCommit = true;
	}

	if (config->mustRewriteOutput)
	{
		err = WriteClockOutput(device, config, acq);
		if (err)
			goto error;
			
		config->mustRewriteOutput = false;
		mustCommit = true;
	}

	if (mustCommit)
	{
		nierr = DAQmxTaskControl(config->doTask, DAQmx_Val_Task_Commit);
		if (nierr)
		{
			err = CreateDAQmxError(nierr);
			err = OScDev_Error_Wrap(err, "Failed to commit clock do task");
			goto error;
		}

		nierr = DAQmxTaskControl(config->lineCtrTask, DAQmx_Val_Task_Commit);
		if (nierr)
		{
			err = CreateDAQmxError(nierr);
			err = OScDev_Error_Wrap(err, "Failed to commit clock lineCtr task");
			goto error;
		}
	}

	return OScDev_RichError_OK;

error:
	if (ShutdownClock(device, config))
		err = OScDev_Error_Wrap(err, "Failed to clean up clock task(s) after error");
	return err;
}


// Remove all DAQmx configuration for the clock
OScDev_RichError *ShutdownClock(OScDev_Device *device, struct ClockConfig *config)
{
	OScDev_RichError* err;
	int32 nierr1 = 0, nierr2 = 0;
	OScDev_RichError* err1 = NULL, *err2 = NULL;

	if (config->doTask)
	{
		nierr1 = DAQmxClearTask(config->doTask);
		if (nierr1) 
		{
			err1 = CreateDAQmxError(nierr1);
			err1 = OScDev_Error_Wrap(err1, "Failed to clear clock do task");
		}
		config->doTask = 0;
	}

	if (config->lineCtrTask)
	{
		nierr2 = DAQmxClearTask(config->lineCtrTask);
		if (nierr2)
		{
			err2 = CreateDAQmxError(nierr2);
			err2 = OScDev_Error_Wrap(err2, "Failed to clear clock lineCtr task");
		}
		config->lineCtrTask = 0;
	}

	if (nierr1) {
		// todo destroy err2
		return err1;
	}
	else {
		// todo destroy err1
		return err2;
	}
}


OScDev_RichError *StartClock(OScDev_Device *device, struct ClockConfig *config)
{
	OScDev_RichError* err;
	int32 nierr;

	nierr = DAQmxStartTask(config->doTask);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to start clock do task");
		ShutdownClock(device, config);
		return err;
	}

	nierr = DAQmxStartTask(config->lineCtrTask);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to start clock lineCtr task");
		ShutdownClock(device, config);
		return err;
	}

	return OScDev_RichError_OK;
}


OScDev_RichError *StopClock(OScDev_Device *device, struct ClockConfig *config)
{
	OScDev_RichError *err;
	int32 nierr;

	nierr = DAQmxStopTask(config->doTask);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to stop clock do task");
		ShutdownClock(device, config);
		return err;
	}

	nierr = DAQmxStopTask(config->lineCtrTask);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to stop clock lineCtr task");
		ShutdownClock(device, config);
		return err;
	}

	return OScDev_RichError_OK;
}


static OScDev_RichError *CreateClockTasks(OScDev_Device *device, struct ClockConfig *config, OScDev_Acquisition *acq)
{
	OScDev_RichError* err;
	int32 nierr;
	nierr = DAQmxCreateTask("ClockDO", &config->doTask);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to create clock do task");
		return err;
	}

	// P0.5 = line clock
	// P0.6 = inverted line clock (for FLIM)
	// P0.7 = frame clock
	// This needs to be port0 to support buffered output

	char doTerminals[256];
	strncpy(doTerminals, GetData(device)->deviceName, sizeof(doTerminals) - 1);
	strncat(doTerminals, "/port0/line5:7",
		sizeof(doTerminals) - strlen(doTerminals) - 1);
	nierr = DAQmxCreateDOChan(config->doTask, doTerminals, "ClockDO",
		DAQmx_Val_ChanPerLine);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to create clock do channel");
		return err;
	}

	nierr = DAQmxGetReadNumChans(config->doTask, &GetData(device)->numDOChannels);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to get number of channels from clock do task");
		return err;
	}

	nierr = DAQmxCreateTask("ClockCtr", &config->lineCtrTask);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to create clock lineCtr task");
		return err;
	}

	double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
	uint32_t xOffset, yOffset, width, height;
	OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);

	uint32_t elementsPerLine = GetData(device)->lineDelay + width + X_RETRACE_LEN;
	double effectiveScanPortion = (double)width / elementsPerLine;
	double lineFreqHz = pixelRateHz / GetData(device)->binFactor / elementsPerLine;
	double scanPhase = GetData(device)->binFactor / pixelRateHz * GetData(device)->lineDelay;

	char ctrTerminals[256];
	strncpy(ctrTerminals, GetData(device)->deviceName, sizeof(ctrTerminals) - 1);
	strncat(ctrTerminals, "/ctr0",
		sizeof(ctrTerminals) - strlen(ctrTerminals) - 1);
	nierr = DAQmxCreateCOPulseChanFreq(config->lineCtrTask, ctrTerminals, "ClockLineCTR",
		DAQmx_Val_Hz, DAQmx_Val_Low, scanPhase, lineFreqHz, effectiveScanPortion);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to create clock co pulse channel");
		return err;
	}

	return OScDev_RichError_OK;
}


static OScDev_RichError *ConfigureClockTiming(OScDev_Device *device, struct ClockConfig *config, OScDev_Acquisition *acq)
{
	OScDev_RichError *err;
	int32 nierr;

	double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
	uint32_t xOffset, yOffset, width, height;
	OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);

	uint32_t elementsPerLine = GetData(device)->lineDelay + width + X_RETRACE_LEN;
	uint32_t yLen = height + Y_RETRACE_LEN;
	int32 elementsPerFramePerChan = elementsPerLine * height;
	int32 totalElementsPerFramePerChan = elementsPerLine * yLen;

	nierr = DAQmxCfgSampClkTiming(config->doTask, "",
		pixelRateHz / GetData(device)->binFactor,
		DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, elementsPerFramePerChan);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to configure timing for clock do task");
		return err;
	}

	double effectiveScanPortion = (double)width / elementsPerLine;
	double lineFreqHz = pixelRateHz / GetData(device)->binFactor / elementsPerLine;
	double scanPhase = GetData(device)->binFactor / pixelRateHz * GetData(device)->lineDelay;

	nierr = DAQmxSetChanAttribute(config->lineCtrTask, "",
		DAQmx_CO_Pulse_Freq, lineFreqHz);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to set clock lineCtr frequency");
		return err;
	}

	nierr = DAQmxSetChanAttribute(config->lineCtrTask, "",
		DAQmx_CO_Pulse_Freq_InitialDelay, scanPhase);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to set clock lineCtr initial delay");
		return err;

	}

	nierr = DAQmxSetChanAttribute(config->lineCtrTask, "",
		DAQmx_CO_Pulse_DutyCyc, effectiveScanPortion);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to set clock lineCtr duty cycle");
		return err;
	}

	nierr = DAQmxCfgImplicitTiming(config->lineCtrTask, DAQmx_Val_FiniteSamps,
		height);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to configure timing for clock lineCtr");
		return err;
	}

	return OScDev_RichError_OK;
}


static OScDev_RichError *ConfigureClockTriggers(OScDev_Device *device, struct ClockConfig *config)
{
	OScDev_RichError *err;
	char triggerSource[256] = "/";
	strncat(triggerSource, GetData(device)->deviceName,
		sizeof(triggerSource) - strlen(triggerSource) - 1);
	strncat(triggerSource, "/ao/StartTrigger",
		sizeof(triggerSource) - strlen(triggerSource) - 1);

	int32 nierr;
	nierr = DAQmxCfgDigEdgeStartTrig(config->doTask,
		triggerSource, DAQmx_Val_Rising);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to configure trigger for clock do task");
		return err;
	}

	nierr = DAQmxSetStartTrigRetriggerable(config->doTask, 1);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to set retriggerable clock do task");
		return err;
	}

	nierr = DAQmxCfgDigEdgeStartTrig(config->lineCtrTask,
		triggerSource, DAQmx_Val_Rising);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to configure trigger for clock lineCtr task");
		return err;
	}

	return OScDev_RichError_OK;
}


static OScDev_RichError *WriteClockOutput(OScDev_Device *device, struct ClockConfig *config, OScDev_Acquisition *acq)
{
	OScDev_RichError *err;
	uint32_t xOffset, yOffset, width, height;
	OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);

	uint32_t elementsPerLine = GetData(device)->lineDelay + width + X_RETRACE_LEN;
	uint32_t yLen = height + Y_RETRACE_LEN;
	int32 elementsPerFramePerChan = elementsPerLine * height;  // without y retrace portion
	int32 totalElementsPerFramePerChan = elementsPerLine * yLen;   // including y retrace portion

	// Q: Why do we use elementsPerFramePerChan, not totalElementsPerFramePerChan?

	// digital line clock pattern for triggering acquisition line by line
	uInt8 *lineClockPattern = (uInt8*)malloc(elementsPerFramePerChan);
	// digital line clock pattern for FLIM
	uInt8 *lineClockFLIM = (uInt8*)malloc(elementsPerFramePerChan);
	// digital frame clock pattern for FLIM
	uInt8 *frameClockFLIM = (uInt8*)malloc(elementsPerFramePerChan);
	// combination of lineClock, lineClockFLIM, and frameClock
	uInt8 *lineClockPatterns = (uInt8*)malloc(
		elementsPerFramePerChan * GetData(device)->numDOChannels);

	// TODO: why use elementsPerLine instead of elementsPerFramePerChan?
	err = GenerateLineClock(width, height,
		GetData(device)->lineDelay,	lineClockPattern);
	if (err)
		return OScDev_Error_Create("Waveform Out Of Range");
	err = GenerateFLIMLineClock(width, height,
		GetData(device)->lineDelay, lineClockFLIM);
	if (err)
		return OScDev_Error_Create("Waveform Out Of Range");
	err = GenerateFLIMFrameClock(width, height,
		GetData(device)->lineDelay, frameClockFLIM);
	if (err)
		return OScDev_Error_Create("Waveform Out Of Range");

	// combine line, inverted line, and frame clocks
	// TODO: make it more generic
	for (int i = 0; i < elementsPerFramePerChan; i++)
	{
		lineClockPatterns[i] = lineClockPattern[i];
		lineClockPatterns[i + elementsPerFramePerChan] = lineClockFLIM[i];
		lineClockPatterns[i + 2 * elementsPerFramePerChan] = frameClockFLIM[i];
	}

	int32 numWritten = 0;
	int32 nierr = DAQmxWriteDigitalLines(config->doTask,
		elementsPerFramePerChan, FALSE, 10.0,
		DAQmx_Val_GroupByChannel, lineClockPatterns, &numWritten, NULL);
	if (nierr)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to write clock do waveforms");
		goto cleanup;
	}
	if (numWritten != elementsPerFramePerChan)
	{
		err = CreateDAQmxError(nierr);
		err = OScDev_Error_Wrap(err, "Failed to write complete clock waveform");
		goto cleanup;
	}

cleanup:
	free(lineClockPattern);
	free(lineClockFLIM);
	free(frameClockFLIM);
	free(lineClockPatterns);
	return err;
}