#include "OScNIDAQ.h"
#include "OScNIDAQDevicePrivate.h"
#include "OpenScanLibPrivate.h"
#include "Waveform.h"

#include <NIDAQmx.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <Windows.h>


static bool g_NIDAQ_initialized = false;
static size_t g_openDeviceCount = 0;

// Originally OScNIDAQDevice.c starts...
static OSc_Device **g_devices;
static size_t g_deviceCount;

static OSc_Error NIDAQGetModelName(const char **name)
{
	*name = "OpenScan-NIDAQ";
	return OSc_Error_OK;
}

static OSc_Error NIDAQGetInstances(OSc_Device ***devices, size_t *count)
{
	if (!g_devices)
		OSc_Return_If_Error(EnumerateInstances(&g_devices, &g_deviceCount));
	*devices = g_devices;
	*count = g_deviceCount;
	return OSc_Error_OK;
}


static OSc_Error NIDAQReleaseInstance(OSc_Device *device)
{
	free(GetData(device));
	device->implData = NULL;
	return OSc_Error_OK;
}


static OSc_Error NIDAQGetName(OSc_Device *device, char *name)
{
	strncpy(name, GetData(device)->deviceName, OSc_MAX_STR_LEN);
	return OSc_Error_OK;
}



static OSc_Error NIDAQOpen(OSc_Device *device)
{
	int32 nierr = DAQmxResetDevice(GetData(device)->deviceName); // TODO wrong function
	if (nierr)
	{
		char msg[OSc_MAX_STR_LEN + 1] = "Cannot reset NI DAQ card ";
		strcat(msg, GetData(device)->deviceName);
		OSc_Log_Error(device, msg);
		return OSc_Error_Unknown; // TODO Detailed info
	}
	OSc_Return_If_Error(OpenDAQ(device));
	return OSc_Error_OK;
}


static OSc_Error NIDAQClose(OSc_Device *device)
{
	// TODO CloseDAQ(). stop and close all tasks
	OSc_Error err = CloseDAQ(device);
	DeleteCriticalSection(&(GetData(device)->acquisition.mutex));
	WakeConditionVariable(&(GetData(device)->acquisition.acquisitionFinishCondition));
	return err;
}


static OSc_Error NIDAQHasScanner(OSc_Device *device, bool *hasScanner)
{
	*hasScanner = true;
	return OSc_Error_OK;
}


static OSc_Error NIDAQHasDetector(OSc_Device *device, bool *hasDetector)
{
	*hasDetector = true;
	return OSc_Error_OK;
}

static OSc_Error NIDAQGetSettings(OSc_Device *device, OSc_Setting ***settings, size_t *count)
{
	OSc_Return_If_Error(PrepareSettings(device));
	*settings = GetData(device)->settings;
	*count = GetData(device)->settingCount;
	return OSc_Error_OK;

}

static OSc_Error NIDAQGetAllowedResolutions(OSc_Device *device, size_t **widths, size_t **heights, size_t *count)
{
	static size_t resolutions[] = { 256, 512, 1024, 2048 };
	*widths = *heights = resolutions;
	*count = sizeof(resolutions) / sizeof(size_t);
	return OSc_Error_OK;
}

static OSc_Error NIDAQGetResolution(OSc_Device *device, size_t *width, size_t *height)
{
	*width = *height = GetData(device)->resolution;
	return OSc_Error_OK;
}


static OSc_Error NIDAQSetResolution(OSc_Device *device, size_t width, size_t height)
{
	if (width == GetData(device)->resolution)
		return OSc_Error_OK;
	GetData(device)->resolution = (uint32_t)width;
	GetData(device)->settingsChanged = true;
	return OSc_Error_OK;
}

static OSc_Error NIDAQGetImageSize(OSc_Device *device, uint32_t *width, uint32_t *height)
{
	*width = GetData(device)->resolution;
	*height = GetData(device)->resolution;
	return OSc_Error_OK;
}

// Same as OpenScanDAQ::GetNumberOfChannels()
static OSc_Error NIDAQGetNumberOfChannels(OSc_Device *device, uint32_t *nChannels)
{
	*nChannels = GetData(device)->channels == CHANNELS1_2_3 ? 3 :
		CHANNELS_1_AND_2 ? 2 : 1;
	return OSc_Error_OK;
}

static OSc_Error NIDAQGetBytesPerSample(OSc_Device *device, uint32_t *bytesPerSample)
{
	*bytesPerSample = 2;
	return OSc_Error_OK;
}


static OSc_Error ArmImpl(OSc_Device *device, OSc_Acquisition *acq)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (GetData(device)->acquisition.running &&
			GetData(device)->acquisition.armed)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			if (GetData(device)->acquisition.started)
				return OSc_Error_Acquisition_Running;
			else
				return OSc_Error_OK;
		}
		GetData(device)->acquisition.stopRequested = false;
		GetData(device)->acquisition.running = true;
		GetData(device)->acquisition.armed = false;
		GetData(device)->acquisition.started = false;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	if (GetData(device)->settingsChanged)
	{
		// TODO: Reload param
		OSc_Return_If_Error(ReconfigTiming(device));
		GetData(device)->settingsChanged = false;
	}

	OSc_Return_If_Error(SetScanParameters(device));

	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		GetData(device)->acquisition.armed = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	return OSc_Error_OK;
}

static OSc_Error NIDAQArmScanner(OSc_Device *device, OSc_Acquisition *acq)
{
	return ArmImpl(device, acq);
}


static OSc_Error NIDAQStartScanner(OSc_Device *device, OSc_Acquisition *acq)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (!GetData(device)->acquisition.running ||
			!GetData(device)->acquisition.armed)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			return OSc_Error_Not_Armed;
		}
		if (GetData(device)->acquisition.started)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			return OSc_Error_Acquisition_Running;
		}

		GetData(device)->acquisition.started = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	return RunAcquisitionLoop(device, acq);
}

static OSc_Error NIDAQStopScanner(OSc_Device *device, OSc_Acquisition *acq)
{
	return StopAcquisitionAndWait(device, acq);
}


static OSc_Error NIDAQArmDetector(OSc_Device *device, OSc_Acquisition *acq)
{
	return ArmImpl(device, acq);
}

static OSc_Error NIDAQStartDetector(OSc_Device *device, OSc_Acquisition *acq)
{
	// We don't yet support running detector as trigger source
	return OSc_Error_Unsupported_Operation;
}


static OSc_Error NIDAQStopDetector(OSc_Device *device, OSc_Acquisition *acq)
{
	return StopAcquisitionAndWait(device, acq);
}

static OSc_Error NIDAQIsRunning(OSc_Device *device, bool *isRunning)
{
	return IsAcquisitionRunning(device, isRunning);
}


static void ParseDeviceNameList(const char *names,
	void(*callback)(const char *name))
{
	// The value of the Device Names property is a list, separated by ", "
	const char *sep = NULL;
	const char *prevSep = names;
	const char *end = names + strlen(names);
	for (;;)
	{
		// Skip any spaces after comma
		while (*prevSep == ' ')
			++prevSep;

		sep = strchr(prevSep, ',');
		if (!sep)
			sep = end;

		size_t len = sep - prevSep;
		if (len == 0 || len > OSc_MAX_STR_LEN)
			continue;
		char name[OSc_MAX_STR_LEN + 1];
		strncpy(name, prevSep, len);

		callback(name);

		if (sep == end)
			return;
		prevSep = sep;
	}
}

// what is this for?
static void CreateDevice(const char* name)
{
	if (!g_devices)
	{
		g_devices = malloc(sizeof(OSc_Device *));
	}
	else
	{
		g_devices = realloc(g_devices, g_deviceCount * sizeof(OSc_Device *));
	}

	struct OScNIDAQPrivateData *data = calloc(1, sizeof(struct OScNIDAQPrivateData));

	OSc_Device *device;
	OSc_Error err;
	if (OSc_Check_Error(err, OSc_Device_Create(&device, &OpenScan_NIDAQ_Device_Impl, data)))
	{
		char msg[OSc_MAX_STR_LEN + 1] = "Failed to create device ";
		strcat(msg, name);
		OSc_Log_Error(device, msg);
		return;
	}

	strncpy(GetData(device)->deviceName, name, OSc_MAX_STR_LEN);

	g_devices[g_deviceCount++] = device;
}


static OSc_Error NIDAQWait(OSc_Device *device)
{
	return WaitForAcquisitionToFinish(device);
}
/*
static OSc_Error EnumerateInstances(OSc_Device ***devices, size_t *count)
{
if (g_devices)
return OSc_Error_OK;

char deviceNames[4096];
int32 nierr = DAQmxGetSysDevNames(deviceNames, sizeof(deviceNames));
if (nierr)
{
OSc_Log_Error(NULL, "Failed to enumerate NI DAQ devices");
return OSc_Error_Unknown;
}

ParseDeviceNameList(deviceNames, CreateDevice);

return OSc_Error_OK;
}
*/

struct OSc_Device_Impl OpenScan_NIDAQ_Device_Impl = {
	.GetModelName = NIDAQGetModelName,
	.GetInstances = NIDAQGetInstances,
	.ReleaseInstance = NIDAQReleaseInstance,
	.GetName = NIDAQGetName,
	.Open = NIDAQOpen,
	.Close = NIDAQClose,
	.HasScanner = NIDAQHasScanner,
	.HasDetector = NIDAQHasDetector,
	// New added
	.GetSettings = NIDAQGetSettings,
	.GetAllowedResolutions = NIDAQGetAllowedResolutions,
	.GetResolution = NIDAQGetResolution,
	.SetResolution = NIDAQSetResolution,
	.GetImageSize = NIDAQGetImageSize,
	.GetNumberOfChannels = NIDAQGetNumberOfChannels,
	.GetBytesPerSample = NIDAQGetBytesPerSample,
	.ArmScanner = NIDAQArmScanner,
	.StartScanner = NIDAQStartScanner,
	.StopScanner = NIDAQStopScanner,
	.ArmDetector = NIDAQArmDetector,
	.StartDetector = NIDAQStartDetector,
	.StopDetector = NIDAQStopDetector,
	.IsRunning = NIDAQIsRunning,
	.Wait = NIDAQWait,
};
// OScNIDAQDevice.c ends

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


static OSc_Error EnumerateInstances(OSc_Device ***devices, size_t *deviceCount)
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
	/*
	TaskHandle scanWaveformTaskHandle_ = GetData(device)->scanWaveformTaskHandle_;
	TaskHandle lineClockTaskHandle_ = GetData(device)->lineClockTaskHandle_;
	TaskHandle counterTaskHandle_ = GetData(device)->counterTaskHandle_;
	TaskHandle acqTaskHandle_ = GetData(device)->acqTaskHandle_;
	uint32_t resolution_ = GetData(device)->resolution;
	uint32_t numDOChannels_ = GetData(device)->numDOChannels;
	double scanRate_ = GetData(device)->scanRate;
	uint32_t binFactor_ = GetData(device)->binFactor;
	double zoom_ = GetData(device)->zoom;
	uInt32 numAIChannels_ = GetData(device)->acquisition.numAIChannels;
	*/


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
	/*
	TaskHandle scanWaveformTaskHandle_ = GetData(device)->scanWaveformTaskHandle_;
	TaskHandle lineClockTaskHandle_ = GetData(device)->lineClockTaskHandle_;
	TaskHandle counterTaskHandle_ = GetData(device)->counterTaskHandle_;
	TaskHandle acqTaskHandle_ = GetData(device)->acqTaskHandle_;
	uint32_t resolution_ = GetData(device)->resolution;
	uint32_t numDOChannels_ = GetData(device)->numDOChannels;
	double scanRate_ = GetData(device)->scanRate;
	uint32_t binFactor_ = GetData(device)->binFactor;
	double zoom_ = GetData(device)->zoom;
	uInt32 numAIChannels_ = GetData(device)->acquisition.numAIChannels;
	*/

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
	TaskHandle scanWaveformTaskHandle_ = GetData(device)->scanWaveformTaskHandle_;
	TaskHandle lineClockTaskHandle_ = GetData(device)->lineClockTaskHandle_;
	TaskHandle counterTaskHandle_ = GetData(device)->counterTaskHandle_;
	TaskHandle acqTaskHandle_ = GetData(device)->acqTaskHandle_;
	uint32_t resolution_ = GetData(device)->resolution;
	uint32_t numDOChannels_ = GetData(device)->numDOChannels;
	double scanRate_ = GetData(device)->scanRate;
	uint32_t binFactor_ = GetData(device)->binFactor;
	double zoom_ = GetData(device)->zoom;
	uInt32 numAIChannels_ = GetData(device)->acquisition.numAIChannels;



	uint32_t elementsPerLine = X_UNDERSHOOT + resolution_ + X_RETRACE_LEN;
	uint32_t numScanLines = resolution_;
	uint32_t yLen = resolution_ + Y_RETRACE_LEN;
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
	uInt8 *lineClockPatterns = (uInt8*)malloc(sizeof(uInt8) *elementsPerFramePerChan * numDOChannels_);
	int err = GenerateGalvoWaveformFrame(resolution_, zoom_, xyWaveformFrame);
	if (err != 0)
		return OSc_Error_Waveform_Out_Of_Range;

	int32 numWritten = 0;
	int32 nierr = DAQmxWriteAnalogF64(scanWaveformTaskHandle_, totalElementsPerFramePerChan, FALSE, 10.0,
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
	err = GenerateLineClock(resolution_, numScanLines, lineClockPattern);
	if (err != 0)
		return OSc_Error_Waveform_Out_Of_Range;
	err = GenerateFLIMLineClock(resolution_, numScanLines, lineClockFLIM);
	if (err != 0)
		return OSc_Error_Waveform_Out_Of_Range;
	err = GenerateFLIMFrameClock(resolution_, numScanLines, frameClockFLIM);
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
	if (scanWaveformTaskHandle_)
	{
		free(xyWaveformFrame);
		DAQmxStopTask(scanWaveformTaskHandle_);
		DAQmxClearTask(scanWaveformTaskHandle_);
		scanWaveformTaskHandle_ = 0;
	}
	if (lineClockTaskHandle_)
	{
		free(lineClockPattern);
		DAQmxStopTask(lineClockTaskHandle_);
		DAQmxClearTask(lineClockTaskHandle_);
		lineClockTaskHandle_ = 0;
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
	TaskHandle scanWaveformTaskHandle_ = GetData(device)->scanWaveformTaskHandle_;
	TaskHandle counterTaskHandle_ = GetData(device)->counterTaskHandle_;
	TaskHandle acqTaskHandle_ = GetData(device)->acqTaskHandle_;
	uint32_t resolution_ = GetData(device)->resolution;
	uint32_t numDOChannels_ = GetData(device)->numDOChannels;
	double scanRate_ = GetData(device)->scanRate;
	uint32_t binFactor_ = GetData(device)->binFactor;
	double zoom_ = GetData(device)->zoom;
	uInt32 numAIChannels_ = GetData(device)->acquisition.numAIChannels;
	int32 nierr;

	nierr = DAQmxStartTask(acqTaskHandle_);
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Armed acquisition", true);

	nierr = DAQmxStartTask(counterTaskHandle_);
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Armed counter (line clock) generation", true);

	nierr = DAQmxStartTask(scanWaveformTaskHandle_);
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Armed scan waveform generation. Starting scan...", true);
	return OSc_Error_OK;

Error:
	if (scanWaveformTaskHandle_)
	{
		DAQmxStopTask(scanWaveformTaskHandle_);
		DAQmxClearTask(scanWaveformTaskHandle_);
		scanWaveformTaskHandle_ = 0;
	}

	if (counterTaskHandle_)
	{
		DAQmxStopTask(counterTaskHandle_);
		DAQmxClearTask(counterTaskHandle_);
		counterTaskHandle_ = 0;
	}

	if (acqTaskHandle_)
	{
		DAQmxStopTask(acqTaskHandle_);
		DAQmxClearTask(acqTaskHandle_);
		acqTaskHandle_ = 0;
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
	TaskHandle scanWaveformTaskHandle_ = GetData(device)->scanWaveformTaskHandle_;
	TaskHandle counterTaskHandle_ = GetData(device)->counterTaskHandle_;
	TaskHandle acqTaskHandle_ = GetData(device)->acqTaskHandle_;
	uint32_t resolution_ = GetData(device)->resolution;
	double scanRate_ = GetData(device)->scanRate;
	uint32_t binFactor_ = GetData(device)->binFactor;

	int32 nierr = DAQmxStopTask(acqTaskHandle_);
	if (nierr != 0)
	{
		//LogMessage("Error stopping acquisition task", true);
		goto Error;
	}
	//LogMessage("Stopped acquisition task", true);

	nierr = DAQmxStopTask(counterTaskHandle_);
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
	uint32_t xLen = resolution_ + X_RETRACE_LEN;
	// TODO: casting
	uint32_t waitScanToFinish = (5 + 1E-3 * (double)(xLen * Y_RETRACE_LEN * binFactor_ / scanRate_));
	//LogMessage("Wait " + boost::lexical_cast<std::string>(waitScanToFinish) +
		//" ms for scan to finish...", true);
	Sleep(waitScanToFinish);
	nierr = DAQmxStopTask(scanWaveformTaskHandle_);
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
	if (scanWaveformTaskHandle_)
	{
		DAQmxClearTask(scanWaveformTaskHandle_);
		scanWaveformTaskHandle_ = 0;
	}

	if (counterTaskHandle_)
	{
		DAQmxClearTask(counterTaskHandle_);
		counterTaskHandle_ = 0;
	}

	if (acqTaskHandle_)
	{
		DAQmxClearTask(acqTaskHandle_);
		acqTaskHandle_ = 0;
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
static OSc_Error ReadImage(OSc_Device *device)
{
	uint32_t resolution_ = GetData(device)->resolution;
	uint32_t elementsPerLine = X_UNDERSHOOT + resolution_ + X_RETRACE_LEN;
	uint32_t scanLines = resolution_;
	int32 elementsPerFramePerChan = elementsPerLine * scanLines;
	size_t nPixels = GetImageWidth(device) * GetImageHeight(device);
	

	//GetData(device)->ch1Buffer;
	/*
	delete[] ch1Buffer_;
	delete[] ch2Buffer_;
	delete[] ch3Buffer_;
	delete[] rawLineData_;
	delete[] avgLineData_;
	delete[] imageData_;
	*/
	/*
	GetData(device)->ch1Buffer = new uint16_t[nPixels];
	//ch2Buffer_ = new uint16_t[nPixels];
	//ch3Buffer_ = new uint16_t[nPixels];
	// raw data acquired by DAQ in each scan line
	rawLineData_ = new float64[numAIChannels_ * resolution_ * binFactor_];
	// averaged line data in original format
	avgLineData_ = new float64[numAIChannels_ * resolution_];
	// averaged final frame data (all channels)
	// format: CH1 resolution_ pixels | CH2 resolution pixels | CH1 resolution_ pixels | CH2 ...
	imageData_ = new uint16_t[numAIChannels_ * nPixels];
	*/


	// TODO
	GetData(device)->ch1Buffer = realloc(GetData(device)->ch1Buffer, 2 * nPixels);
	GetData(device)->oneFrameScanDone = false;

	OSc_Error err;
	if (OSc_Check_Error(err, StartScan(device))) {
		return err;
	}

	// Wait until one frame is scanned
	while (!GetData(device)->oneFrameScanDone)
		Sleep(50);

	if (OSc_Check_Error(err, StopScan(device))) {
		return err;
	}

	/*
	if (OSc_Check_Error(err, StartScan(device))) {
		return err;
	}
	*/
	Sleep(100);
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

	FinishAcquisition(device);
	return 0;
}


static OSc_Error RunAcquisitionLoop(OSc_Device *device, OSc_Acquisition *acq)
{
	GetData(device)->acquisition.acquisition = acq;
	DWORD id;
	GetData(device)->acquisition.thread =
		CreateThread(NULL, 0, AcquisitionLoop, device, 0, &id);
	return OSc_Error_OK;
}



static OSc_Error StopAcquisitionAndWait(OSc_Device *device, OSc_Acquisition *acq)
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
static OSc_Error IsAcquisitionRunning(OSc_Device *device, bool *isRunning)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	*isRunning = GetData(device)->acquisition.running;
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return OSc_Error_OK;
}


static OSc_Error WaitForAcquisitionToFinish(OSc_Device *device)
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
	TaskHandle scanWaveformTaskHandle_ = GetData(device)->scanWaveformTaskHandle_;
	TaskHandle lineClockTaskHandle_ = GetData(device)->lineClockTaskHandle_;
	TaskHandle counterTaskHandle_ = GetData(device)->counterTaskHandle_;
	TaskHandle acqTaskHandle_ = GetData(device)->acqTaskHandle_;
	uint32_t resolution_ = GetData(device)->resolution;
	uint32_t numDOChannels_ = GetData(device)->numDOChannels;
	double scanRate_ = GetData(device)->scanRate;
	uint32_t binFactor_ = GetData(device)->binFactor;
	double zoom_ = GetData(device)->zoom;
	uInt32 numAIChannels_ = GetData(device)->acquisition.numAIChannels;


	uint32_t elementsPerLine = X_UNDERSHOOT + resolution_ + X_RETRACE_LEN;
	uint32_t scanLines = resolution_;
	uint32_t yLen = scanLines + Y_RETRACE_LEN;
	int32 elementsPerFramePerChan = elementsPerLine * scanLines;
	int32 totalElementsPerFramePerChan = elementsPerLine * yLen;

	int32 nierr = DAQmxCfgSampClkTiming(scanWaveformTaskHandle_, "", 1E6*scanRate_ / binFactor_,
		DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, totalElementsPerFramePerChan); // reload
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Configured sample clock timing for scan waveform", true);

	nierr = DAQmxCfgSampClkTiming(acqTaskHandle_, "", 1E6*scanRate_,
		DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, resolution_ * binFactor_); // reload
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Configured sample clock timing for acquisition", true);

	// update counter timing-related parameters
	double effectiveScanPortion = (double)resolution_ / elementsPerLine;
	double lineFreq = (double)((1E6*scanRate_) / binFactor_ / elementsPerLine);  // unit: Hz
																				 // adjustment corresponding to galvo undershoot at each line
																				 // the delay (in second) between the command scan waveform and the actual scanner response
	double scanPhase = (double)(binFactor_ / (1E6*scanRate_) * X_UNDERSHOOT);

	nierr = DAQmxSetChanAttribute(counterTaskHandle_, "", DAQmx_CO_Pulse_Freq, lineFreq);
	nierr = DAQmxSetChanAttribute(counterTaskHandle_, "", DAQmx_CO_Pulse_Freq_InitialDelay, scanPhase);
	nierr = DAQmxSetChanAttribute(counterTaskHandle_, "", DAQmx_CO_Pulse_DutyCyc, effectiveScanPortion);
	nierr = DAQmxCfgImplicitTiming(counterTaskHandle_, DAQmx_Val_FiniteSamps, scanLines);
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Configured timing for counter generated line clock", true);

	return OSc_Error_OK;

Error:
	if (scanWaveformTaskHandle_)
	{
		DAQmxStopTask(scanWaveformTaskHandle_);
		DAQmxClearTask(scanWaveformTaskHandle_);
		scanWaveformTaskHandle_ = 0;
	}

	if (lineClockTaskHandle_)
	{
		DAQmxStopTask(lineClockTaskHandle_);
		DAQmxClearTask(lineClockTaskHandle_);
		lineClockTaskHandle_ = 0;
	}

	if (counterTaskHandle_)
	{
		DAQmxStopTask(counterTaskHandle_);
		DAQmxClearTask(counterTaskHandle_);
		counterTaskHandle_ = 0;
	}

	if (acqTaskHandle_)
	{
		DAQmxStopTask(acqTaskHandle_);
		DAQmxClearTask(acqTaskHandle_);
		acqTaskHandle_ = 0;
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

OSc_Error SnapImage(OSc_Device *device) {
	TaskHandle scanWaveformTaskHandle_ = GetData(device)->scanWaveformTaskHandle_;
	TaskHandle lineClockTaskHandle_ = GetData(device)->lineClockTaskHandle_;
	TaskHandle counterTaskHandle_ = GetData(device)->counterTaskHandle_;
	TaskHandle acqTaskHandle_ = GetData(device)->acqTaskHandle_;
	uint32_t resolution_ = GetData(device)->resolution;
	uint32_t numDOChannels_ = GetData(device)->numDOChannels;
	double scanRate_ = GetData(device)->scanRate;
	uint32_t binFactor_ = GetData(device)->binFactor;
	double zoom_ = GetData(device)->zoom;
	uInt32 numAIChannels_ = GetData(device)->acquisition.numAIChannels;

	//if (IsCapturing(device))
		//return OSc_Error_Acquisition_Running;
	bool isRunning = false;
	IsAcquisitionRunning(device, &isRunning);
	if(isRunning)
		return OSc_Error_Acquisition_Running;
	OSc_Error err;

	// if any of DAQ tasks are not initialized
	if (!scanWaveformTaskHandle_ || !lineClockTaskHandle_ || !acqTaskHandle_ || !counterTaskHandle_)
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
	if (GetData(device)->acqSettingsChanged && GetData(device)->isEveryNSamplesEventRegistered)
	{
		if (OSc_Check_Error(err, UnregisterLineAcqEvent(device))) {
			return err;
		}

		GetData(device)->isEveryNSamplesEventRegistered = false;
	}

	// Re-register event when resolution or binFactor has changed
	if (GetData(device)->acqSettingsChanged)
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
	if (OSc_Check_Error(err, ReadImage(device))) {
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
	TaskHandle scanWaveformTaskHandle_ = GetData(device)->scanWaveformTaskHandle_;
	TaskHandle lineClockTaskHandle_ = GetData(device)->lineClockTaskHandle_;
	TaskHandle counterTaskHandle_ = GetData(device)->counterTaskHandle_;
	TaskHandle acqTaskHandle_ = GetData(device)->acqTaskHandle_;
	uint32_t resolution_ = GetData(device)->resolution;
	uint32_t numDOChannels_ = GetData(device)->numDOChannels;
	double scanRate_ = GetData(device)->scanRate;
	uint32_t binFactor_ = GetData(device)->binFactor;
	double zoom_ = GetData(device)->zoom;
	uInt32 numAIChannels_ = GetData(device)->acquisition.numAIChannels;

	int32 nierr = DAQmxTaskControl(acqTaskHandle_, DAQmx_Val_Task_Commit);
	if (nierr != 0)
	{
		goto Error;
	}

	nierr = DAQmxTaskControl(counterTaskHandle_, DAQmx_Val_Task_Commit);
	if (nierr != 0)
	{
		goto Error;
	}

	nierr = DAQmxTaskControl(scanWaveformTaskHandle_, DAQmx_Val_Task_Commit);
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Committed DAQmx settings to hardware", true);

	return OSc_Error_OK;

Error:
	if (scanWaveformTaskHandle_)
	{
		DAQmxStopTask(scanWaveformTaskHandle_);
		DAQmxClearTask(scanWaveformTaskHandle_);
		scanWaveformTaskHandle_ = 0;
	}

	if (lineClockTaskHandle_)
	{
		DAQmxStopTask(lineClockTaskHandle_);
		DAQmxClearTask(lineClockTaskHandle_);
		lineClockTaskHandle_ = 0;
	}

	if (counterTaskHandle_)
	{
		DAQmxStopTask(counterTaskHandle_);
		DAQmxClearTask(counterTaskHandle_);
		counterTaskHandle_ = 0;
	}
	if (acqTaskHandle_)
	{
		DAQmxStopTask(acqTaskHandle_);
		DAQmxClearTask(acqTaskHandle_);
		acqTaskHandle_ = 0;
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
	TaskHandle scanWaveformTaskHandle_ = GetData(device)->scanWaveformTaskHandle_;
	TaskHandle lineClockTaskHandle_ = GetData(device)->lineClockTaskHandle_;
	TaskHandle counterTaskHandle_ = GetData(device)->counterTaskHandle_;
	TaskHandle acqTaskHandle_ = GetData(device)->acqTaskHandle_;
	uint32_t resolution_ = GetData(device)->resolution;
	uint32_t numDOChannels_ = GetData(device)->numDOChannels;
	double scanRate_ = GetData(device)->scanRate;
	uint32_t binFactor_ = GetData(device)->binFactor;
	double zoom_ = GetData(device)->zoom;
	uInt32 numAIChannels_ = GetData(device)->acquisition.numAIChannels;

	int32 nierr = DAQmxRegisterEveryNSamplesEvent(acqTaskHandle_, DAQmx_Val_Acquired_Into_Buffer,
		resolution_ * binFactor_, 0, NULL, NULL);
	if (nierr != 0)
	{
		goto Error;
	}
	//LogMessage("Ungistered line acquisition callback event", true);

	return OSc_Error_OK;

Error:
	if (acqTaskHandle_)
	{
		DAQmxStopTask(acqTaskHandle_);
		DAQmxClearTask(acqTaskHandle_);
		acqTaskHandle_ = 0;
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

// register DAQ line acquisition event
OSc_Error RegisterLineAcqEvent(OSc_Device *device)
{
	TaskHandle scanWaveformTaskHandle_ = GetData(device)->scanWaveformTaskHandle_;
	TaskHandle lineClockTaskHandle_ = GetData(device)->lineClockTaskHandle_;
	TaskHandle counterTaskHandle_ = GetData(device)->counterTaskHandle_;
	TaskHandle acqTaskHandle_ = GetData(device)->acqTaskHandle_;
	uint32_t resolution_ = GetData(device)->resolution;
	uint32_t numDOChannels_ = GetData(device)->numDOChannels;
	double scanRate_ = GetData(device)->scanRate;
	uint32_t binFactor_ = GetData(device)->binFactor;
	double zoom_ = GetData(device)->zoom;
	uInt32 numAIChannels_ = GetData(device)->acquisition.numAIChannels;

	// TODO
	/*
	// nSamples actually means nSamples per channel (refer to https://goo.gl/6zjMgB)
	int32 nierr = DAQmxRegisterEveryNSamplesEvent(acqTaskHandle_, DAQmx_Val_Acquired_Into_Buffer,
		resolution_ * binFactor_, 0, ReadLineCallbackWrapper, this);  // readimage
	if (nierr != 0)
	{
		goto Error;
	}
	LogMessage("Registered line acquisition callback event", true);
	*/
	return OSc_Error_OK;
/*
Error:
	if (acqTaskHandle_)
	{
		DAQmxStopTask(acqTaskHandle_);
		DAQmxClearTask(acqTaskHandle_);
		acqTaskHandle_ = 0;
	}
	/*
	int err;
	if (nierr != 0)
	{
		LogMessage("Failed registering EveryNSamplesEvent; task cleared");
		err = TranslateNIError(nierr);
	}
	else
	{
		err = DEVICE_ERR;
	}

	return OSc_Error_Unknown;
	*/
}

/*
// EveryNSamplesCallback()
// read from PMT line by line
// non-interlaced acquisition. 
// evary line data in format: Channel1 | Channel 2 | Channel 3 | ...
static OSc_Error ReadLineCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, OSc_Device *device)
{
	TaskHandle scanWaveformTaskHandle_ = GetData(device)->scanWaveformTaskHandle_;
	TaskHandle lineClockTaskHandle_ = GetData(device)->lineClockTaskHandle_;
	TaskHandle counterTaskHandle_ = GetData(device)->counterTaskHandle_;
	TaskHandle acqTaskHandle_ = GetData(device)->acqTaskHandle_;
	uint32_t resolution_ = GetData(device)->resolution;
	uint32_t numDOChannels_ = GetData(device)->numDOChannels;
	double scanRate_ = GetData(device)->scanRate;
	uint32_t binFactor_ = GetData(device)->binFactor;
	double zoom_ = GetData(device)->zoom;
	uInt32 numAIChannels_ = GetData(device)->acquisition.numAIChannels;
	//int32 totalRead_ = GetData(device);
	bool oneFrameScanDone_ = GetData(device)->oneFrameScanDone;
	//LogMessage("line acq started...", true);
	int32 readPerChan;
	int32_t prevPercentRead = -1;
	uInt32 totalSamplesPerLine = numAIChannels_ * resolution_ * binFactor_;
	int32 currLine = 1 + totalRead_ / numAIChannels_ / binFactor_ / resolution_;
	// rawLineData format with GroupByChannel (non-interlaced) is:
	// CH1 pixel 1 meas 1..binFactor | CH1 p2 m1..binFactor | ... | CH1 pN m1..binFactor || CH2 p1 m1..binFactor |...
	int32 nierr = DAQmxReadAnalogF64(taskHandle, -1, 10.0, DAQmx_Val_GroupByChannel,
		rawLineData_, totalSamplesPerLine, &readPerChan, NULL);
	if (nierr != 0)
	{
		goto Error;
	}

	if (readPerChan > 0)
	{
		//  append data line by line to the frame data array
		for (uint32_t i = 0; i < totalSamplesPerLine; i += binFactor_)
		{
			// pixel averaging
			avgLineData_[i / binFactor_] = rawLineData_[i] / binFactor_;
			for (unsigned j = 1; j < binFactor_; j++)
				avgLineData_[i / binFactor_] += (rawLineData_[i + j] / binFactor_);
			// convert processed line and append to output image frame
			imageData_[i / binFactor_ + totalRead_ / binFactor_] =
				(int16)abs(avgLineData_[i / binFactor_] / inputVoltageRange_ * 32768);
		}

		totalRead_ += (numAIChannels_ * readPerChan); // update total elements acquired
													  //int32 currLine = totalRead_ / numAIChannels_ / binFactor_ / resolution_;
		if (currLine % 128 == 0)
		{

		}
	}
	else
	{
		LogMessage("Callback received but no data read");
	}

	if (totalRead_ == resolution_ * totalSamplesPerLine)
	{
		oneFrameScanDone_ = true;
		totalRead_ = 0;
		LogMessage("End of scanning one frame", true);
	}

	return DEVICE_OK;

Error:
	if (taskHandle)
	{
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
		taskHandle = 0;
	}
	return DEVICE_ERR;
}
*/





