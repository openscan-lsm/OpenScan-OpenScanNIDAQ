/* Device-specific scanner and/or detector implementations */
/* Interact with physical hardware (NI DAQ) */
/* and rely on specific device library (NIDAQmx) */

#include "OScNIDAQ.h"
#include "Waveform.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <Windows.h>


static bool g_NIDAQ_initialized = false;
static size_t g_openDeviceCount = 0;


static inline uint16_t DoubleToFixed16(double d, int intBits)
{
	int fracBits = 16 - intBits;
	return (uint16_t)round(d * (1 << fracBits));
}


static OSc_Error EnsureNIDAQInitialized(void)
{
	if (g_NIDAQ_initialized)
		return OSc_Error_OK;
	g_NIDAQ_initialized = true;
	return OSc_Error_OK;
}



static OSc_Error DeinitializeNIDAQ(void)
{
	if (!g_NIDAQ_initialized)
		return OSc_Error_OK;
	g_NIDAQ_initialized = false;
	return OSc_Error_OK;
}


static void PopulateDefaultParameters(struct OScNIDAQPrivateData *data)
{
	data->detectorOnly = false;
	data->scannerOnly = false;
	data->enableCallback = true;

	data->settingsChanged = true;
	data->timingSettingsChanged = true;
	data->waveformSettingsChanged = true;
	data->acqSettingsChanged = true;
	data->isEveryNSamplesEventRegistered = false;
	data->oneFrameScanDone = false;

	data->scanRate = 1.0;  // MHz
	data->resolution = 512;
	data->zoom = 1.0;
	data->binFactor = 2;
	data->inputVoltageRange = 10.0;
	data->channels = CHANNEL1;
	data->numDOChannels = 1;

	InitializeCriticalSection(&(data->acquisition.mutex));
	data->acquisition.thread = NULL;
	InitializeConditionVariable(&(data->acquisition.acquisitionFinishCondition));
	data->acquisition.running = false;
	data->acquisition.armed = false;
	data->acquisition.started = false;
	data->acquisition.stopRequested = false;
	data->acquisition.acquisition = NULL;
}

// TODO: automatically mapping deviceName using DAQmxGetSysDevNames()
OSc_Error NIDAQEnumerateInstances(OSc_Device ***devices, size_t *deviceCount)
{
	OSc_Return_If_Error(EnsureNIDAQInitialized());

	struct OScNIDAQPrivateData *data = calloc(1, sizeof(struct OScNIDAQPrivateData));
	//strncpy(data->rioResourceName, "RIO0", OSc_MAX_STR_LEN);
	strncpy(data->deviceName, "PXI1Slot2", OSc_MAX_STR_LEN);
	

	OSc_Device *device;
	OSc_Error err;
	if (OSc_Check_Error(err, OSc_Device_Create(&device, &OpenScan_NIDAQ_Device_Impl, data)))
	{
		char msg[OSc_MAX_STR_LEN + 1] = "Failed to create device ";
		strcat(msg, data->rioResourceName);
		OSc_Log_Error(NULL, msg);
		return err; // TODO
	}

	PopulateDefaultParameters(GetData(device));

	*devices = malloc(sizeof(OSc_Device *));
	*deviceCount = 1;
	(*devices)[0] = device;

	return OSc_Error_OK;
}

// same to Initialize() in old OpenScan format
OSc_Error OpenDAQ(OSc_Device *device)
{
	//LogMessage("Initializing NI DAQ...", true);
	OSc_Error err;
	if (OSc_Check_Error(err, StartDAQ(device))) {
		/*
		char msg[OSc_MAX_STR_LEN + 1] = "Failed to create device ";
		strcat(msg, data->rioResourceName);
		OSc_Log_Error(NULL, msg);
		*/
		return err;
	}

	if (OSc_Check_Error(err, SetTriggers(device))) {
		return err;
	}

	//LogMessage("DAQ initialized.", true);

	return OSc_Error_OK;
}

// same to InitDAQ() in old OpenScan format
OSc_Error StartDAQ(OSc_Device *device)
{
	OSc_Return_If_Error(EnsureNIDAQInitialized());
	int32 nierr;
	// initialize scan waveform task
	if (!GetData(device)->scanWaveformTaskHandle_)
	{
		nierr = DAQmxCreateTask("", &GetData(device)->scanWaveformTaskHandle_);
		if (nierr != 0)
		{
			//LogMessage("Error creating scanWaveformTaskHandle_");
			return OSc_Error_Unknown_Enum_Value_Name;
		}
		//LogMessage("Created scan waveform task", true);
		nierr = DAQmxCreateAOVoltageChan(GetData(device)->scanWaveformTaskHandle_, "PXI1Slot2/ao0:1", "",
			-10.0, 10.0, DAQmx_Val_Volts, NULL);
		if (nierr != 0)
		{
			//LogMessage("Failed to create AO channel");
			goto Error;
		}
		//LogMessage("Created AO voltage channels for X and Y scan waveforms", true);
	}

	// initialize line clock task
	if (!GetData(device)->lineClockTaskHandle_)
	{
		nierr = DAQmxCreateTask("", &GetData(device)->lineClockTaskHandle_);
		if (nierr != 0)
		{
			return OSc_Error_Unknown_Enum_Value_Name;
		}
		//LogMessage("Created line clock task", true);
		// wire p0.5 to /PXI1Slot2/PFI7 to trigger acquisition line by line
		// has to use port0 since it supports buffered opertation
		// p0.6 to generate line clock for FLIM
		// p0.7 to generate frame clock for FLIM
		//nierr = DAQmxCreateDOChan(lineClockTaskHandle_, "PXI1Slot2/port0/line5:6",
		//	"", DAQmx_Val_ChanForAllLines );
		nierr = DAQmxCreateDOChan(GetData(device)->lineClockTaskHandle_,  "PXI1Slot2/port0/line5:7",
			"", DAQmx_Val_ChanPerLine);
		if (nierr != 0)
		{
			goto Error;
		}

		// get number of physical DO channels for image acquisition
		nierr = DAQmxGetReadNumChans(GetData(device)->lineClockTaskHandle_, &GetData(device)->numDOChannels);
		if (nierr != 0)
		{
			return OSc_Error_Unknown_Enum_Value_Name;
		}
		//LogMessage("Created " + boost::lexical_cast<std::string>(numDOChannels_) +
		//" physical DO channels for line clocks and frame clock.", true);
	}

	// init counter
	if (!GetData(device)->counterTaskHandle_)
	{
		nierr = DAQmxCreateTask("", &GetData(device)->counterTaskHandle_);
		if (nierr != 0)
		{
			return OSc_Error_Unknown_Enum_Value_Name;
		}
		//LogMessage("Created counter task", true);

		// Create CO Channel.
		uint32_t elementsPerLine = X_UNDERSHOOT + GetData(device)->resolution + X_RETRACE_LEN;
		uint32_t scanLines = GetData(device)->resolution;
		double effectiveScanPortion = (double)GetData(device)->resolution / elementsPerLine;
		double lineFreq = (double)((1E6*GetData(device)->scanRate) / GetData(device)->binFactor / elementsPerLine);  // unit: Hz
																					 // adjustment corresponding to galvo undershoot at each line
																					 // the delay (in second) between the command scan waveform and the actual scanner response
		double scanPhase = (double)(GetData(device)->binFactor / (1E6*GetData(device)->scanRate) * X_UNDERSHOOT);
		nierr = DAQmxCreateCOPulseChanFreq(GetData(device)->counterTaskHandle_, "PXI1Slot2/ctr0",
			"", DAQmx_Val_Hz, DAQmx_Val_Low, scanPhase, lineFreq, effectiveScanPortion); // CTR0 OUT = PFI12
		if (nierr != 0)
		{
			goto Error;
		}

		// define how many lines to scan
		nierr = DAQmxCfgImplicitTiming(GetData(device)->counterTaskHandle_, DAQmx_Val_FiniteSamps, scanLines);
		if (nierr != 0)
		{
			goto Error;
		}
		//LogMessage("Configured counter timing", true);

	}

	// initialize acquisition task
	if (!GetData(device)->acqTaskHandle_)
	{
		nierr = DAQmxCreateTask("", &GetData(device)->acqTaskHandle_);
		if (nierr != 0)
		{
			return OSc_Error_Unknown_Enum_Value_Name;
		}
		//LogMessage("Created acquisition task", true);
		nierr = DAQmxCreateAIVoltageChan(GetData(device)->acqTaskHandle_, "PXI1Slot2/ai0:2", "",
			DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL);
		if (nierr != 0)
		{
			goto Error;
		}
		//LogMessage("Created AI voltage channels for image acquisition", true);
		// get number of physical AI channels for image acquisition
		// difference from GetNumberOfChannels() which indicates number of channels to display
		nierr = DAQmxGetReadNumChans(GetData(device)->acqTaskHandle_, &GetData(device)->acquisition.numAIChannels);
		if (nierr != 0)
		{
			return OSc_Error_Unknown_Enum_Value_Name;
		}
		//LogMessage(boost::lexical_cast<std::string>(numAIChannels_) +
		//" physical AI channels available.", true);
		//*/
	}
	++g_openDeviceCount;

	return OSc_Error_OK;
Error:
	if (GetData(device)->scanWaveformTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->scanWaveformTaskHandle_);
		DAQmxClearTask(GetData(device)->scanWaveformTaskHandle_);
		GetData(device)->scanWaveformTaskHandle_ = 0;
	}

	if (GetData(device)->lineClockTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->lineClockTaskHandle_);
		DAQmxClearTask(GetData(device)->lineClockTaskHandle_);
		GetData(device)->lineClockTaskHandle_ = 0;
	}

	if (GetData(device)->counterTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->counterTaskHandle_);
		DAQmxClearTask(GetData(device)->counterTaskHandle_);
		GetData(device)->counterTaskHandle_ = 0;
	}

	if (GetData(device)->acqTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->acqTaskHandle_);
		DAQmxClearTask(GetData(device)->acqTaskHandle_);
		GetData(device)->acqTaskHandle_ = 0;
	}

	OSc_Error err;
	if (nierr != 0)
	{
		/*
		LogMessage("Failed initializing tasks; task cleared");
		err = TranslateNIError(nierr);
		*/
		err = OSc_Error_Unknown_Enum_Value_Name;
	}
	else
	{
		// TODO: Specify what exact error it is 
		err = OSc_Error_Unknown_Enum_Value_Name;
	}
	return err;

}


// same to Shutdown() in old OpenScan format
OSc_Error CloseDAQ(OSc_Device *device)
{
	//TODO
	//StopAcquisitionAndWait(device, acq);
	--g_openDeviceCount;
	if (g_openDeviceCount == 0)
		OSc_Return_If_Error(DeinitializeNIDAQ());

	return OSc_Error_OK;
}




// Set up how image acq, line clock, and scan waveform are triggered
OSc_Error SetTriggers(OSc_Device *device)
{
	// Use AO StartTrigger to trigger the line clock.
	// This is an internal trigger signal.
	char aoStartTrigName[256];
	OSc_Error err;
	if (OSc_Check_Error(err, GetTerminalNameWithDevPrefix(GetData(device)->scanWaveformTaskHandle_, "ao/StartTrigger", aoStartTrigName))) {
		return err;
	}
	//LogMessage("Get AO Start Trigger name to trigger line clock", true);


	// Configure counter
	// line clock generation is triggered by AO StartTrigger internally
	// and thus sync'ed to scan waveform generation
	int32 nierr = DAQmxCfgDigEdgeStartTrig(GetData(device)->counterTaskHandle_, aoStartTrigName, DAQmx_Val_Rising);
	if (nierr != 0)
	{
		//LogMessage(GetNIDetailedErrorForMostRecentCall().c_str());
		goto Error;
	}

	//LogMessage("Configured digital edge start trigger for counter", true);

	nierr = DAQmxCfgDigEdgeStartTrig(GetData(device)->acqTaskHandle_, "/PXI1Slot2/PFI12", DAQmx_Val_Rising);
	if (nierr != 0)
	{
		//LogMessage(GetNIDetailedErrorForMostRecentCall().c_str());
		goto Error;
	}
	//LogMessage("Configured digital edge start trigger for image acquisition", true);

	nierr = DAQmxSetStartTrigRetriggerable(GetData(device)->acqTaskHandle_, 1);
	if (nierr != 0)
	{
		//LogMessage(GetNIDetailedErrorForMostRecentCall().c_str());
		goto Error;
	}

	// connect line clock output terminal to acq start trigger (input) terminal
	// it seems it only works for terminals with valid names (port0 doesn't work; PFI lines are ok)
	// TODO: line clock output right now has to use port0 for buffered operation
	//       so need to find another way to generate line clock without buffer requirement
	//nierr = DAQmxConnectTerms("/PXI1Slot2/PFI1", "/PXI1Slot2/PFI7", DAQmx_Val_DoNotInvertPolarity);

	return OSc_Error_OK;

Error:
	if (GetData(device)->scanWaveformTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->scanWaveformTaskHandle_);
		DAQmxClearTask(GetData(device)->scanWaveformTaskHandle_);
		GetData(device)->scanWaveformTaskHandle_ = 0;
	}

	if (GetData(device)->lineClockTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->lineClockTaskHandle_);
		DAQmxClearTask(GetData(device)->lineClockTaskHandle_);
		GetData(device)->lineClockTaskHandle_ = 0;
	}

	if (GetData(device)->counterTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->counterTaskHandle_);
		DAQmxClearTask(GetData(device)->counterTaskHandle_);
		GetData(device)->counterTaskHandle_ = 0;
	}
	if (GetData(device)->acqTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->acqTaskHandle_);
		DAQmxClearTask(GetData(device)->acqTaskHandle_);
		GetData(device)->acqTaskHandle_ = 0;
	}
	/*
	err;
	if (nierr != 0)
	{
		LogMessage("Failed setting triggers; task cleared");
		err = TranslateNIError(nierr);
	}
	else
	{
		err = 1000;
	}
	*/
	return OSc_Error_Unknown;
}

OSc_Error GetTerminalNameWithDevPrefix(TaskHandle taskHandle, const char terminalName[], char triggerName[])
{
	int32	error = 0;
	char	device[256];
	int32	productCategory;
	uInt32	numDevices, i = 1;

	int32 nierr = DAQmxGetTaskNumDevices(taskHandle, &numDevices);
	if (nierr != 0)
		return nierr;
	while (i <= numDevices) {
		nierr = DAQmxGetNthTaskDevice(taskHandle, i++, device, 256);
		if (nierr != 0)
			return nierr;
		nierr = DAQmxGetDevProductCategory(device, &productCategory);
		if (nierr != 0)
			return nierr;
		if (productCategory != DAQmx_Val_CSeriesModule && productCategory != DAQmx_Val_SCXIModule) {
			*triggerName++ = '/';
			strcat(strcat(strcpy(triggerName, device), "/"), terminalName);
			break;
		}
	}
	return OSc_Error_OK;
}


static OSc_Error SendParameters(OSc_Device *device)
{
	OSc_Return_If_Error(StartDAQ(device));

		return OSc_Error_OK;
}

static OSc_Error WriteWaveforms(OSc_Device *device)
{
	uint32_t elementsPerLine = X_UNDERSHOOT + GetData(device)->resolution + X_RETRACE_LEN;
	uint32_t numScanLines = GetData(device)->resolution;
	uint32_t yLen = GetData(device)->resolution + Y_RETRACE_LEN;
	int32 elementsPerFramePerChan = elementsPerLine * numScanLines;  // without y retrace portion
	int32 totalElementsPerFramePerChan = elementsPerLine * yLen;   // including y retrace portion

																   // 1st half of galvo scan waveform for AO0 (X) and 2nd half for AO1 (Y)
	double *xyWaveformFrame = (double*)malloc(sizeof(double) * totalElementsPerFramePerChan * 2);

	// digital line clock pattern for triggering acquisition line by line
	uInt8 *lineClockPattern = (uInt8*)malloc(sizeof(uInt8) *elementsPerFramePerChan);
	// digital line clock pattern for FLIM
	uInt8 *lineClockFLIM = (uInt8*)malloc(sizeof(uInt8) *elementsPerFramePerChan);
	// digital frame clock pattern for FLIM
	uInt8 *frameClockFLIM = (uInt8*)malloc(sizeof(uInt8) *elementsPerFramePerChan);
	// combination of lineClock, lineClockFLIM, and frameClock
	uInt8 *lineClockPatterns = (uInt8*)malloc(sizeof(uInt8) *elementsPerFramePerChan * GetData(device)->numDOChannels);
	int err = GenerateGalvoWaveformFrame(GetData(device)->resolution, GetData(device)->zoom, xyWaveformFrame);
	if (err != 0)
		return OSc_Error_Waveform_Out_Of_Range;

	int32 numWritten = 0;
	int32 nierr = DAQmxWriteAnalogF64(GetData(device)->scanWaveformTaskHandle_, totalElementsPerFramePerChan, FALSE, 10.0,
		DAQmx_Val_GroupByChannel, xyWaveformFrame, &numWritten, NULL);
	if (nierr != 0)
	{
		//LogMessage("Write scanwaveform error:", true);
		goto Error;
	}
	if (numWritten != totalElementsPerFramePerChan)
	{
		//LogMessage("Failed to write complete scan waveform");
		goto Error;
	}
	//LogMessage("One frame waveform written to DAQ memory", true);

	// TODO: why use elementsPerLine instead of elementsPerFramePerChan?
	err = GenerateLineClock(GetData(device)->resolution, numScanLines, lineClockPattern);
	if (err != 0)
		return OSc_Error_Waveform_Out_Of_Range;
	err = GenerateFLIMLineClock(GetData(device)->resolution, numScanLines, lineClockFLIM);
	if (err != 0)
		return OSc_Error_Waveform_Out_Of_Range;
	err = GenerateFLIMFrameClock(GetData(device)->resolution, numScanLines, frameClockFLIM);
	if (err != 0)
		return OSc_Error_Waveform_Out_Of_Range;


	// combine two line clocks
	// TODO: make it more generic
	for (int32 i = 0; i < elementsPerFramePerChan; i++)
	{
		lineClockPatterns[i] = lineClockPattern[i];
		lineClockPatterns[i + elementsPerFramePerChan] = lineClockFLIM[i];
		lineClockPatterns[i + 2 * elementsPerFramePerChan] = frameClockFLIM[i];
	}
	free(xyWaveformFrame);
	free(lineClockPattern);
	free(lineClockFLIM);
	free(frameClockFLIM);
	free(lineClockPatterns);

	return OSc_Error_OK;

Error:
	if (GetData(device)->scanWaveformTaskHandle_)
	{
		free(xyWaveformFrame);
		DAQmxStopTask(GetData(device)->scanWaveformTaskHandle_);
		DAQmxClearTask(GetData(device)->scanWaveformTaskHandle_);
		GetData(device)->scanWaveformTaskHandle_ = 0;
	}
	if (GetData(device)->lineClockTaskHandle_)
	{
		free(lineClockPattern);
		DAQmxStopTask(GetData(device)->lineClockTaskHandle_);
		DAQmxClearTask(GetData(device)->lineClockTaskHandle_);
		GetData(device)->lineClockTaskHandle_ = 0;
	}
	/*
	err;
	if (nierr != 0)
	{
		LogMessage("Failed writing waveforms; task cleared");
		err = TranslateNIError(nierr);
	}
	else
	{
		err = OSc_Error_Waveform_Out_Of_Range;
	}
	*/
	return OSc_Error_Unknown;

}

/*
OSc_Error ReloadWaveform(OSc_Device *device)
{
	OSc_Log_Debug(device, "Sending parameters...");
	OSc_Return_If_Error(SendParameters(device));

	OSc_Log_Debug(device, "Setting resolution...");
	OSc_Return_If_Error(SetResolution(device,
		GetData(device)->resolution));

	OSc_Log_Debug(device, "Setting up scan...");
	OSc_Return_If_Error(InitScan(device));

	return OSc_Error_OK;
}
*/
// DAQ version; start all tasks
// Arm acquisition task first. Then make sure the (digital) line clock output 
// is armed before the (analog) waveform output. 
// This will ensure both tasks will start at the same time.
static OSc_Error StartScan(OSc_Device *device)
{
	int32 nierr;
	if (!GetData(device)->scannerOnly) {
		nierr = DAQmxStartTask(GetData(device)->acqTaskHandle_);
		if (nierr != 0)
		{
			goto Error;
		}
		//LogMessage("Armed acquisition", true);
	}
	else {

	}

	nierr = DAQmxStartTask(GetData(device)->counterTaskHandle_);
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Armed counter (line clock) generation", true);

	nierr = DAQmxStartTask(GetData(device)->scanWaveformTaskHandle_);
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Armed scan waveform generation. Starting scan...", true);
	return OSc_Error_OK;

Error:
	if (GetData(device)->scanWaveformTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->scanWaveformTaskHandle_);
		DAQmxClearTask(GetData(device)->scanWaveformTaskHandle_);
		GetData(device)->scanWaveformTaskHandle_ = 0;
	}

	if (GetData(device)->counterTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->counterTaskHandle_);
		DAQmxClearTask(GetData(device)->counterTaskHandle_);
		GetData(device)->counterTaskHandle_ = 0;
	}

	if (GetData(device)->acqTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->acqTaskHandle_);
		DAQmxClearTask(GetData(device)->acqTaskHandle_);
		GetData(device)->acqTaskHandle_ = 0;
	}
	/*
	int err;
	if (nierr != 0)
	{
		//LogMessage("Failed starting tasks; task cleared");
		err = TranslateNIError(nierr);
	}
	else
	{
		err = OSc_Error_Unknown;
	}
	*/
	return OSc_Error_Unknown;
}


static OSc_Error StopScan(OSc_Device *device)
{
	int32 nierr;

	if (!GetData(device)->scannerOnly) {
		nierr = DAQmxStopTask(GetData(device)->acqTaskHandle_);
		if (nierr != 0)
		{
			//LogMessage("Error stopping acquisition task", true);
			goto Error;
		}
		//LogMessage("Stopped acquisition task", true);
	}
	else {

	}


	nierr = DAQmxStopTask(GetData(device)->counterTaskHandle_);
	if (nierr != 0)
	{
		//LogMessage("Error stopping line clock task", true);
		goto Error;
	}
	//LogMessage("Stopped line clock task", true);

	// When scanRate is low, it takes longer to finish generating scan waveform.
	// Since acquisition only takes a portion of the total scan time,
	// it may occur that waveform task is stopped right after acquisition is done
	// but the waveform generation is not done yet -- thus nierr 200010:
	// "Finite acquisition or generation has been stopped before the requested number
	// of samples were acquired or generated."
	// So need to wait some miliseconds till waveform generation is done before stop the task.
	uint32_t xLen = GetData(device)->resolution + X_RETRACE_LEN;
	// TODO: casting
	uint32_t waitScanToFinish = (5 + 1E-3 * (double)(xLen * Y_RETRACE_LEN * GetData(device)->binFactor / GetData(device)->scanRate));
	//LogMessage("Wait " + boost::lexical_cast<std::string>(waitScanToFinish) +
		//" ms for scan to finish...", true);
	Sleep(waitScanToFinish);
	nierr = DAQmxStopTask(GetData(device)->scanWaveformTaskHandle_);
	if (nierr != 0)
	{
		//LogMessage("Error stopping scan waveform task; NI DAQmx error code: " +
			//boost::lexical_cast<std::string>(nierr), true);
		//LogMessage(GetNIDetailedErrorForMostRecentCall().c_str());
		goto Error;
	}
	//LogMessage("Stopped scan waveform task", true);

	return OSc_Error_OK;
Error:
	if (GetData(device)->scanWaveformTaskHandle_)
	{
		DAQmxClearTask(GetData(device)->scanWaveformTaskHandle_);
		GetData(device)->scanWaveformTaskHandle_ = 0;
	}

	if (GetData(device)->counterTaskHandle_)
	{
		DAQmxClearTask(GetData(device)->counterTaskHandle_);
		GetData(device)->counterTaskHandle_ = 0;
	}

	if (GetData(device)->acqTaskHandle_)
	{
		DAQmxClearTask(GetData(device)->acqTaskHandle_);
		GetData(device)->acqTaskHandle_ = 0;
	}
	/*
	int err;
	if (nierr != 0)
	{
		//LogMessage("Failed stopping tasks; task cleared");
		err = TranslateNIError(nierr);
	}
	else
	{
		err = OSc_Error_Unknown_Enum_Value_Name;
	}
	*/
	return OSc_Error_Unknown;

}
static unsigned GetImageWidth(OSc_Device *device) {
	return  GetData(device)->resolution;
}

static unsigned GetImageHeight(OSc_Device *device) {
	return  GetData(device)->resolution;
}
// DAQ version; acquire from multiple channels
static OSc_Error ReadImage(OSc_Device *device, OSc_Acquisition *acq)
{
	
	//uint32_t resolution_ = GetData(device)->resolution;
	uint32_t elementsPerLine = X_UNDERSHOOT + GetData(device)->resolution + X_RETRACE_LEN;
	uint32_t scanLines = GetData(device)->resolution;
	int32 elementsPerFramePerChan = elementsPerLine * scanLines;
	size_t nPixels = GetImageWidth(device) * GetImageHeight(device);

	// TODO
	GetData(device)->imageData = realloc(GetData(device)->imageData,
		sizeof(uint16_t) * GetData(device)->acquisition.numAIChannels * nPixels);
	GetData(device)->rawLineData = realloc(GetData(device)->rawLineData, 
		sizeof(float64) * GetData(device)->acquisition.numAIChannels * GetData(device)->resolution * GetData(device)->binFactor);
	GetData(device)->avgLineData = realloc(GetData(device)->avgLineData,
		sizeof(float64) * GetData(device)->acquisition.numAIChannels * GetData(device)->resolution);

	GetData(device)->ch1Buffer = realloc(GetData(device)->ch1Buffer, sizeof(uint16_t) * nPixels);
	GetData(device)->ch2Buffer = realloc(GetData(device)->ch2Buffer, sizeof(uint16_t) * nPixels);
	GetData(device)->ch3Buffer = realloc(GetData(device)->ch3Buffer, sizeof(uint16_t) * nPixels);

	GetData(device)->oneFrameScanDone = GetData(device)->scannerOnly;
	//GetData(device)->oneFrameScanDone = true;

	// initialize channel buffers
	for (size_t i = 0; i < nPixels; ++i)
	{
		GetData(device)->ch1Buffer[i] = 0;
		GetData(device)->ch2Buffer[i] = 255;
		GetData(device)->ch3Buffer[i] = 32767;
	}

	for (size_t i = 0; i < GetData(device)->acquisition.numAIChannels * nPixels; ++i)
	{
		GetData(device)->imageData[i] = 16383;
	}

	OSc_Error err;
	if (OSc_Check_Error(err, StartScan(device))) {
		return err;
	}

	// Wait until one frame is scanned
	int timeout = 0;
	while (!GetData(device)->oneFrameScanDone) {
		Sleep(50);
		timeout++;
		//if (timeout == 100) {
		//	break;
		//}
	}

	if (OSc_Check_Error(err, StopScan(device))) {
		return err;
	}

	// SplitChannels
	// skik if set to scanner only mode
	if (!GetData(device)->scannerOnly)
	{
		if (OSc_Check_Error(err, SplitChannels(device))) { return err; }
	}

	acq->frameCallback(acq, 0, GetData(device)->ch1Buffer, acq->data);
	acq->frameCallback(acq, 1, GetData(device)->ch2Buffer, acq->data);
	acq->frameCallback(acq, 2, GetData(device)->ch3Buffer, acq->data);

	Sleep(1000);

	return OSc_Error_OK;
}

// split all-channel image buffer to separate channel buffers
// * works when DAQ acquires in GroupByChannel (non-interlaced) mode
OSc_Error SplitChannels(OSc_Device *device)
{
	// imageData_ if displayed as 2D image will have N channels on each row
	// data is stored line by line with N channels in a row per line
	uint32_t rawImageWidth = GetImageWidth(device) * GetData(device)->acquisition.numAIChannels;
	uint32_t rawImageHeight = GetImageHeight(device);
	uint32_t xLength = GetImageWidth(device);
	uint32_t yLength = GetImageHeight(device);
	// convert big image buffer to separate channel buffers
	for (uint32_t chan = 0; chan < GetData(device)->acquisition.numAIChannels; chan++)
		for (uint32_t currRow = 0; currRow < yLength; currRow++)
			for (uint32_t currCol = 0; currCol < xLength; currCol++)
				switch (chan) {
				case 0:
					GetData(device)->ch1Buffer[currCol + currRow * xLength] =
						GetData(device)->imageData[currCol + chan*xLength + currRow*rawImageWidth];
					break;
				case 1:
					GetData(device)->ch2Buffer[currCol + currRow * xLength] =
						GetData(device)->imageData[currCol + chan*xLength + currRow*rawImageWidth];
					break;

				case 2:
					GetData(device)->ch3Buffer[currCol + currRow * xLength] =
						GetData(device)->imageData[currCol + chan*xLength + currRow*rawImageWidth];
					break;

				default:
					//LogMessage("More than 2 channels available or something wrong", true);
					break;
				}
	//LogMessage("Finished reading one image and splitting data to channel buffers", true);
	return OSc_Error_OK;
}

static OSc_Error AcquireFrame(OSc_Device *device, OSc_Acquisition *acq, unsigned kalmanCounter)
{
	return OSc_Error_OK;
}


static void FinishAcquisition(OSc_Device *device)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	GetData(device)->acquisition.running = false;
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	CONDITION_VARIABLE *cv = &(GetData(device)->acquisition.acquisitionFinishCondition);
	WakeAllConditionVariable(cv);
}


static DWORD WINAPI AcquisitionLoop(void *param)
{
	OSc_Device *device = (OSc_Device *)param;
	OSc_Acquisition *acq = GetData(device)->acquisition.acquisition;
	OSc_Error err;
	if (OSc_Check_Error(err, SnapImage(device, acq))) {
		FinishAcquisition(device);
		return 0;
	}
	FinishAcquisition(device);
	return 0;
}


OSc_Error RunAcquisitionLoop(OSc_Device *device, OSc_Acquisition *acq)
{
	GetData(device)->acquisition.acquisition = acq;
	DWORD id;
	GetData(device)->acquisition.thread =
		CreateThread(NULL, 0, AcquisitionLoop, device, 0, &id);
	return OSc_Error_OK;
}



OSc_Error StopAcquisitionAndWait(OSc_Device *device, OSc_Acquisition *acq)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (!GetData(device)->acquisition.running)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			return OSc_Error_OK;
		}

		GetData(device)->acquisition.stopRequested = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return WaitForAcquisitionToFinish(device);
}


static bool IsCapturing(OSc_Device *device) {
	bool running = false;
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		running = (GetData(device)->acquisition.running) ? true : false;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return running;
}


// Iscapturing in old openscanDAQ
OSc_Error IsAcquisitionRunning(OSc_Device *device, bool *isRunning)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	*isRunning = GetData(device)->acquisition.running;
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return OSc_Error_OK;
}


OSc_Error WaitForAcquisitionToFinish(OSc_Device *device)
{
	OSc_Error err = OSc_Error_OK;
	CONDITION_VARIABLE *cv = &(GetData(device)->acquisition.acquisitionFinishCondition);

	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	while (GetData(device)->acquisition.running)
	{
		SleepConditionVariableCS(cv, &(GetData(device)->acquisition.mutex), INFINITE);
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return err;
}

OSc_Error ReconfigTiming(OSc_Device *device)
{
	uint32_t elementsPerLine = X_UNDERSHOOT + GetData(device)->resolution + X_RETRACE_LEN;
	uint32_t scanLines = GetData(device)->resolution;
	uint32_t yLen = scanLines + Y_RETRACE_LEN;
	int32 elementsPerFramePerChan = elementsPerLine * scanLines;
	int32 totalElementsPerFramePerChan = elementsPerLine * yLen;

	int32 nierr = DAQmxCfgSampClkTiming(GetData(device)->scanWaveformTaskHandle_, "", 1E6*GetData(device)->scanRate / GetData(device)->binFactor,
		DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, totalElementsPerFramePerChan); // reload
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Configured sample clock timing for scan waveform", true);

	nierr = DAQmxCfgSampClkTiming(GetData(device)->acqTaskHandle_, "", 1E6*GetData(device)->scanRate,
		DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, GetData(device)->resolution * GetData(device)->binFactor); // reload
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Configured sample clock timing for acquisition", true);

	// update counter timing-related parameters
	double effectiveScanPortion = (double)GetData(device)->resolution / elementsPerLine;
	double lineFreq = (double)((1E6*GetData(device)->scanRate) / GetData(device)->binFactor/ elementsPerLine);  // unit: Hz
																				 // adjustment corresponding to galvo undershoot at each line
																				 // the delay (in second) between the command scan waveform and the actual scanner response
	double scanPhase = (double)(GetData(device)->binFactor / (1E6*GetData(device)->scanRate) * X_UNDERSHOOT);

	nierr = DAQmxSetChanAttribute(GetData(device)->counterTaskHandle_, "", DAQmx_CO_Pulse_Freq, lineFreq);
	nierr = DAQmxSetChanAttribute(GetData(device)->counterTaskHandle_, "", DAQmx_CO_Pulse_Freq_InitialDelay, scanPhase);
	nierr = DAQmxSetChanAttribute(GetData(device)->counterTaskHandle_, "", DAQmx_CO_Pulse_DutyCyc, effectiveScanPortion);
	nierr = DAQmxCfgImplicitTiming(GetData(device)->counterTaskHandle_, DAQmx_Val_FiniteSamps, scanLines);
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Configured timing for counter generated line clock", true);

	return OSc_Error_OK;

Error:
	if (GetData(device)->scanWaveformTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->scanWaveformTaskHandle_);
		DAQmxClearTask(GetData(device)->scanWaveformTaskHandle_);
		GetData(device)->scanWaveformTaskHandle_ = 0;
	}

	if (GetData(device)->lineClockTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->lineClockTaskHandle_);
		DAQmxClearTask(GetData(device)->lineClockTaskHandle_);
		GetData(device)->lineClockTaskHandle_ = 0;
	}

	if (GetData(device)->counterTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->counterTaskHandle_);
		DAQmxClearTask(GetData(device)->counterTaskHandle_);
		GetData(device)->counterTaskHandle_ = 0;
	}

	if (GetData(device)->acqTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->acqTaskHandle_);
		DAQmxClearTask(GetData(device)->acqTaskHandle_);
		GetData(device)->acqTaskHandle_ = 0;
	}
	OSc_Error err;
	if (nierr != 0)
	{
		//LogMessage("Failed configuring timing; task cleared");
		//err = TranslateNIError(nierr);
		err = OSc_Error_Unknown;
	}
	else
	{
		err = OSc_Error_Unknown;
	}
	return err;

}

OSc_Error SnapImage(OSc_Device *device, OSc_Acquisition *acq) {
	/*
	bool isRunning = false;
	IsAcquisitionRunning(device, &isRunning);
	if(isRunning)
		return OSc_Error_Acquisition_Running;
	*/

	OSc_Error err;
	// if any of DAQ tasks are not initialized
	if (!GetData(device)->scanWaveformTaskHandle_ || !GetData(device)->lineClockTaskHandle_ || !GetData(device)->acqTaskHandle_ || !GetData(device)->counterTaskHandle_)
	{
		if (OSc_Check_Error(err, StartDAQ(device))) {
			return err;
		}

		if (OSc_Check_Error(err, SetTriggers(device))) {
			return err;
		}

		//LogMessage("DAQ re-initialized.", true);

		// all the timing, waveforms, etc. have to reset as well
		// as new tasks, although bear the same names as previously failed/cleared ones,
		// have different memory address (pointers) and their relationship to
		// timing, triggers, etc. need to be re-established.
		/*
		timingSettingsChanged_ = true;
		waveformSettingsChanged_ = true;
		acqSettingsChanged_ = true;
		*/
		GetData(device)->timingSettingsChanged = true;
		GetData(device)->waveformSettingsChanged = true;
		GetData(device)->acqSettingsChanged = true;
	}

	if (GetData(device)->timingSettingsChanged)
	{
		if (OSc_Check_Error(err, ReconfigTiming(device))) {
			return err;
		}
		GetData(device)->timingSettingsChanged = false;
		GetData(device)->settingsChanged = true;
	}

	if (GetData(device)->waveformSettingsChanged)
	{
		//LogMessage("Writing scan waveform and line clock pattern to DAQ...", true);
		if (OSc_Check_Error(err, WriteWaveforms(device))) {
			return err;
		}

		GetData(device)->waveformSettingsChanged = false;
		GetData(device)->settingsChanged = true;
	}

	// first check if existing EveryNSamplesEvent needs to be unregistered
	// to allow new event to get registered when acqSettings has changed since previous scan
	if (GetData(device)->enableCallback && GetData(device)->acqSettingsChanged && GetData(device)->isEveryNSamplesEventRegistered && !GetData(device)->scannerOnly)
	{
		if (OSc_Check_Error(err, UnregisterLineAcqEvent(device))) {
			return err;
		}

		GetData(device)->isEveryNSamplesEventRegistered = false;
	}

	// Re-register event when resolution or binFactor has changed
	if (GetData(device)->enableCallback && GetData(device)->acqSettingsChanged && !GetData(device)->scannerOnly)
	{
		if (OSc_Check_Error(err, RegisterLineAcqEvent(device))) {
			return err;
		}

		GetData(device)->acqSettingsChanged = false;
		GetData(device)->settingsChanged = true;
		GetData(device)->isEveryNSamplesEventRegistered = true;
	}

	// commit tasks whenever settings have changed
	if (GetData(device)->settingsChanged)
	{
		if (OSc_Check_Error(err, CommitTasks(device))) {
			return err;
		}
		GetData(device)->settingsChanged = false;
	}
	if (OSc_Check_Error(err, ReadImage(device, acq))) {
		return err;
	}

	//// TODO: DLL appears working fine even without DAQmxRegisterDoneEvent and Task_Commit
	//nierr = DAQmxRegisterDoneEvent(lineClockTaskHandle_, 0, DoneCallbackWrapper, this);
	//nierr = DAQmxRegisterDoneEvent(scanWaveformTaskHandle_, 0, DoneCallbackWrapper, this);
	//nierr = DAQmxRegisterDoneEvent(acqTaskHandle_, 0, DoneCallbackWrapper, this);


	return OSc_Error_OK;
}

// DAQmx Commit the settings into hardware 
// This allows for very efficient restarts
OSc_Error CommitTasks(OSc_Device *device)
{
	int32 nierr = DAQmxTaskControl(GetData(device)->acqTaskHandle_, DAQmx_Val_Task_Commit);
	if (nierr != 0)
	{
		goto Error;
	}

	nierr = DAQmxTaskControl(GetData(device)->counterTaskHandle_, DAQmx_Val_Task_Commit);
	if (nierr != 0)
	{
		goto Error;
	}

	nierr = DAQmxTaskControl(GetData(device)->scanWaveformTaskHandle_, DAQmx_Val_Task_Commit);
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Committed DAQmx settings to hardware", true);

	return OSc_Error_OK;

Error:
	if (GetData(device)->scanWaveformTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->scanWaveformTaskHandle_);
		DAQmxClearTask(GetData(device)->scanWaveformTaskHandle_);
		GetData(device)->scanWaveformTaskHandle_ = 0;
	}

	if (GetData(device)->lineClockTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->lineClockTaskHandle_);
		DAQmxClearTask(GetData(device)->lineClockTaskHandle_);
		GetData(device)->lineClockTaskHandle_ = 0;
	}

	if (GetData(device)->counterTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->counterTaskHandle_);
		DAQmxClearTask(GetData(device)->counterTaskHandle_);
		GetData(device)->counterTaskHandle_ = 0;
	}
	if (GetData(device)->acqTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->acqTaskHandle_);
		DAQmxClearTask(GetData(device)->acqTaskHandle_);
		GetData(device)->acqTaskHandle_ = 0;
	}
	OSc_Error err;
	if (nierr != 0)
	{
		//LogMessage("Failed committing tasks; task cleared");
		err = OSc_Error_Unknown;
	}
	else
	{
		err = OSc_Error_Unknown;
	}
	return err;
}


// Unregister DAQ line acquisition event
OSc_Error UnregisterLineAcqEvent(OSc_Device *device)
{
	int32 nierr = DAQmxRegisterEveryNSamplesEvent(GetData(device)->acqTaskHandle_, DAQmx_Val_Acquired_Into_Buffer,
		GetData(device)->resolution * GetData(device)->binFactor, 0, NULL, NULL);
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Ungistered line acquisition callback event", true);

	return OSc_Error_OK;

Error:
	if (GetData(device)->acqTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->acqTaskHandle_);
		DAQmxClearTask(GetData(device)->acqTaskHandle_);
		GetData(device)->acqTaskHandle_ = 0;
	}
	OSc_Error err;
	if (nierr != 0)
	{
		//LogMessage("Failed unregistering event; task cleared");
		err = OSc_Error_Unknown;
	}
	else
	{
		err = OSc_Error_Unknown;
	}
	return err;
}


int32 CVICALLBACK ReadLineCallbackWrapper(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData) {
	//OpenScanDAQ * this_ = reinterpret_cast<OpenScanDAQ*>(callbackData);
	//return this_->ReadLineCallback(taskHandle, everyNsamplesEventType, nSamples);
}

// register DAQ line acquisition event
OSc_Error RegisterLineAcqEvent(OSc_Device *device)
{
	// nSamples actually means nSamples per channel (refer to https://goo.gl/6zjMgB)
	int32 nierr = DAQmxRegisterEveryNSamplesEvent(GetData(device)->acqTaskHandle_, DAQmx_Val_Acquired_Into_Buffer,
		GetData(device)->resolution * GetData(device)->binFactor, 0, ReadLineCallback, device);  // readimage
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Registered line acquisition callback event", true);
	return OSc_Error_OK;
Error:
	if (GetData(device)->acqTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->acqTaskHandle_);
		DAQmxClearTask(GetData(device)->acqTaskHandle_);
		GetData(device)->acqTaskHandle_ = 0;
	}
	int err;
	if (nierr != 0)
	{
		//LogMessage("Failed registering EveryNSamplesEvent; task cleared");
		err = OSc_Error_Unknown;
	}
	else
	{
		err = OSc_Error_Unknown;
	}

	return err;
}

// EveryNSamplesCallback()
// read from PMT line by line
// non-interlaced acquisition. 
// evary line data in format: Channel1 | Channel 2 | Channel 3 | ...
OSc_Error ReadLineCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData)
{
	OSc_Device *device= (OSc_Device*)(callbackData);
	struct OScNIDAQPrivateData* privateDevice = GetData(device);
	int32 readPerChan;

	int32_t prevPercentRead = -1;
	uInt32 totalSamplesPerLine = GetData(device)->acquisition.numAIChannels * GetData(device)->resolution * GetData(device)->binFactor;
	int32 currLine = 1 + GetData(device)->totalRead / GetData(device)->acquisition.numAIChannels / GetData(device)->binFactor / GetData(device)->resolution;
	// rawLineData format with GroupByChannel (non-interlaced) is:
	// CH1 pixel 1 meas 1..binFactor | CH1 p2 m1..binFactor | ... | CH1 pN m1..binFactor || CH2 p1 m1..binFactor |...
	int32 nierr = DAQmxReadAnalogF64(GetData(device)->acqTaskHandle_, -1, 10.0, DAQmx_Val_GroupByChannel,
		GetData(device)->rawLineData, totalSamplesPerLine, &readPerChan, NULL);

	// Debug: Get ni err string
	char errbuf[1024];
	DAQmxGetErrorString(nierr, errbuf, sizeof(errbuf));

	if (nierr != 0)
	{
		goto Error;
	}

	if (readPerChan > 0)
	{
		//  append data line by line to the frame data array
		for (uint32_t i = 0; i < totalSamplesPerLine; i += GetData(device)->binFactor)
		{
			// pixel averaging
			GetData(device)->avgLineData[i / GetData(device)->binFactor] = GetData(device)->rawLineData[i] / GetData(device)->binFactor;
			for (unsigned j = 1; j < GetData(device)->binFactor; j++)
				GetData(device)->avgLineData[i / GetData(device)->binFactor] += (GetData(device)->rawLineData[i + j] / GetData(device)->binFactor);
			// convert processed line and append to output image frame
			GetData(device)->imageData[i / GetData(device)->binFactor + GetData(device)->totalRead / GetData(device)->binFactor] =
				(int16)abs(GetData(device)->avgLineData[i / GetData(device)->binFactor] / GetData(device)->inputVoltageRange * 32768);
		}

		GetData(device)->totalRead += (GetData(device)->acquisition.numAIChannels * readPerChan); // update total elements acquired
													  //int32 currLine = totalRead_ / numAIChannels_ / binFactor_ / resolution_;
		if (currLine % 128 == 0)
		{

		}
	}
	else
	{
		//LogMessage("Callback received but no data read");
	}

	if (GetData(device)->totalRead == GetData(device)->resolution * totalSamplesPerLine)
	{
		GetData(device)->oneFrameScanDone = true;
		GetData(device)->totalRead = 0;
		//LogMessage("End of scanning one frame", true);
	}

	return OSc_Error_OK;

Error:
	/*
	if (taskHandle)
	{
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
		taskHandle = 0;
	}
	*/
	if (GetData(device)->acqTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->acqTaskHandle_);
		DAQmxClearTask(GetData(device)->acqTaskHandle_);
		GetData(device)->acqTaskHandle_ = 0;
	}
	return OSc_Error_Unknown;
}

OSc_Error SetScanParameters(OSc_Device *device)
{
	return OSc_Error_OK;
}
