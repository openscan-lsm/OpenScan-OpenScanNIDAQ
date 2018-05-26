/* Device-specific scanner and/or detector implementations */
/* Interact with physical hardware (NI DAQ) */
/* and rely on specific device library (NIDAQmx) */

#include "OScNIDAQ.h"
#include "Waveform.h"
#include "strmap.h"
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
	//data->selectedDispChan_ = malloc(OSc_Total_Channel_Num * (OSc_MAX_STR_LEN + 1) * sizeof(char));
	data->channelMap_ = sm_new(32);
	//Assume portList[256][32];
	data->aiPorts_ = malloc(256 * (sizeof(char)*32));
	for (int i = 0; i < 256; i++) {
		data->aiPorts_[i] = malloc(32 * sizeof(char));
	}
	data->enabledAIPorts_ = malloc(sizeof(char) * 2048);
	
	// Assume 3 chanel at maximum, each 16 chars at most
	data->selectedDispChan_ = malloc(3 * sizeof(char*));
	for (int i = 0; i < 3; i++) {
		data->selectedDispChan_[i] = malloc(sizeof(char) * 16);
	}
	data->selectedDispChan_[0] = "Channel1";
	data->channelCount = 1;
	data->settingsChanged = true;
	data->timingSettingsChanged = true;
	data->waveformSettingsChanged = true;
	data->acqSettingsChanged = true;
	data->channelSettingsChanged = true;
	data->isEveryNSamplesEventRegistered = false;
	data->oneFrameScanDone = false;

	data->totalRead = 0;
	data->scanRate = 1.25;  // MHz
	data->resolution = 512;
	data->zoom = 1.0;
	data->magnification = 1.0;
	data->binFactor = 2;
	data->numLinesToBuffer = 8;
	data->inputVoltageRange = 10.0;
	data->channels = CHANNEL1;
	data->numAIChannels = 1;
	data->numDOChannels = 1;
	data->offsetXY[0] = 0.0;
	data->offsetXY[1] = 0.0;
	data->minVolts_ = -10.0;
	data->maxVolts_ = 10.0;
	
	InitializeCriticalSection(&(data->acquisition.mutex));
	data->acquisition.thread = NULL;
	InitializeConditionVariable(&(data->acquisition.acquisitionFinishCondition));
	data->acquisition.running = false;
	data->acquisition.armed = false;
	data->acquisition.started = false;
	data->acquisition.stopRequested = false;
	data->acquisition.acquisition = NULL;
}

// automatically detect deviceName using DAQmxGetSysDevNames()
OSc_Error NIDAQEnumerateInstances(OSc_Device ***devices, size_t *deviceCount)
{
	OSc_Return_If_Error(EnsureNIDAQInitialized());

	// get a comma - delimited list of all of the devices installed in the system
	char deviceNames[4096];
	int32 nierr = DAQmxGetSysDevNames(deviceNames, sizeof(deviceNames));
	if (nierr != 0)
	{
		return OSc_Error_Unknown;  //TODO
	}

	char deviceList[NUM_SLOTS_IN_CHASSIS][OSc_MAX_STR_LEN + 1];

	OSc_Error err;
	if (OSc_Check_Error(err, ParseDeviceNameList(deviceNames, deviceList, deviceCount)))
	{
		return err;
	}

	*devices = malloc(*deviceCount * sizeof(OSc_Device *));

	for (int i = 0; i < (int)(*deviceCount); ++i)
	{
		struct OScNIDAQPrivateData *data = calloc(1, sizeof(struct OScNIDAQPrivateData));
		// TODO - able to select which DAQ device to use in MM GUI
		// for now assume the DAQ is installed in the 1st available slot in the chassis
		strncpy(data->deviceName, deviceList[i], OSc_MAX_STR_LEN);

		OSc_Device *device;
		if (OSc_Check_Error(err, OSc_Device_Create(&device, &OpenScan_NIDAQ_Device_Impl, data)))
		{
			char msg[OSc_MAX_STR_LEN + 1] = "Failed to create device ";
			strcat(msg, data->deviceName);
			OSc_Log_Error(device, msg);
			return err;
		}

		PopulateDefaultParameters(GetData(device));

		(*devices)[i] = device;
	}
	
	return OSc_Error_OK;
}


OSc_Error GetAIPortsForDevice(char* devices, int* deviceCount, char** result) {
	char ports[4096];
	int32 nierr = DAQmxGetDevAIPhysicalChans(devices, ports, sizeof(ports));
	if (nierr != 0)
	{
		return OSc_Error_Unknown;  //TODO
	}

	// TODO: Max number of AI ports
	char portList[256][32];

	OSc_Error err;
	if (OSc_Check_Error(err, ParseAIPortList(ports, portList, deviceCount)))
	{
		return err;
	}

	// Return char** result from portlist[256][32]
	for (int i = 0; i < *deviceCount; i++) {
		for (int j = 0; j < 32; j++) {
			result[i][j] = portList[i][j];
			if (portList[i][j] == '\0')
				break;
		}
	}
	//result = &portList[0][0];
	return OSc_Error_OK;
}

OSc_Error GetVoltageRangeForDevice(OSc_Device* device, double* minVolts, double* maxVolts){
	//const int MAX_RANGES = 64;
	struct OScNIDAQPrivateData* debug = GetData(device);
	#define MAX_RANGES 64
	float64 ranges[2 * MAX_RANGES];
	for (int i = 0; i < MAX_RANGES; ++i)
	{
		ranges[2 * i] = 0.0;
		ranges[2 * i + 1] = 0.0;
	}

	int32 nierr = DAQmxGetDevAOVoltageRngs(GetData(device)->deviceName, ranges,
		sizeof(ranges) / sizeof(float64));
	if (nierr != 0)
	{
		//LogMessage("Error getting analog voltage ranges");
	}

	// Find the common min and max.
	*minVolts = ranges[0];
	*maxVolts = ranges[1];
	for (int i = 0; i < MAX_RANGES; ++i)
	{
		if (ranges[2 * i] == 0.0 && ranges[2 * i + 1] == 0.0)
			break;

		if (ranges[2 * i + 1] > *maxVolts)
		{
			*minVolts = ranges[2 * i];
			*maxVolts = ranges[2 * i + 1];
		}
	}


	return OSc_Error_OK;
}

OSc_Error GetEnabledAIPorts(OSc_Device *device) {
	//char portList[2048];
	char* portList;
	struct OScNIDAQPrivateData* debug = GetData(device);
	// Assume three channels at most
	int channelNum = GetData(device)->channelCount;
	for (int i = 0; i < channelNum; i++) {
		char mappedStr[255]; // Assume string len is less than 255 char
		sm_get(GetData(device)->channelMap_, GetData(device)->selectedDispChan_[i],  mappedStr, sizeof(mappedStr));
		// Append comma
		// port0, port1, port3...
		if (i == 0) {
			portList = malloc(strlen(mappedStr) * sizeof(char));
			strcpy(portList, mappedStr);
		}
		else {
			char* buffer = malloc((strlen(portList) + strlen(mappedStr) + 1) * sizeof(char));
			strcpy(buffer, portList);
			strcat(buffer, ",");
			strcat(buffer, mappedStr);
			portList = buffer;
		}
	
	}
	GetData(device)->enabledAIPorts_ = portList;
	return OSc_Error_OK;
}


// convert comma comma - delimited device list to a 2D string array
// each row contains the name of one device
static OSc_Error ParseDeviceNameList(char *names,
	char (*deviceNames)[OSc_MAX_STR_LEN + 1], size_t *deviceCount)
{
	const char s[3] = ", ";
	int count = 0;

	// token is a static pointer to the input string
	// input string will be modified between iterations
	for (char *token = strtok(names, s); token != NULL; token = strtok(NULL, s))
	{
		if (count < NUM_SLOTS_IN_CHASSIS)
		{
			strcpy(deviceNames[count], token);
			count++;
		}
		else
			return OSc_Error_Unknown;  //TODO
	}

	*deviceCount = (size_t)count;

	return OSc_Error_OK;
}

// convert comma comma - delimited device list to a 2D string array
// each row contains the name of one ai port
static OSc_Error ParseAIPortList(char *names,
	// assume there are maximum 256 port 
	char deviceNames[MAX_NUM_PORTS][32], size_t *deviceCount)
{
	const char s[3] = ", ";
	int count = 0;

	// token is a static pointer to the input string
	// input string will be modified between iterations
	for (char *token = strtok(names, s); token != NULL; token = strtok(NULL, s))
	{
		if (count < 256)
		{
			strcpy(deviceNames[count], token);
			count++;
		}
		else
			return OSc_Error_Unknown;  //TODO
	}

	*deviceCount = (size_t)count;

	return OSc_Error_OK;
}

OSc_Error MapDispChanToAIPorts(OSc_Device* device)
{
	struct OScNIDAQPrivateData* debug = GetData(device);
	char dispChannels[3][512] = {
		{"Channel1"},
		{"Channel2"},
		{"Channel3" }
	};

	int numDispChannels = 3;
	size_t numAIPorts = 1;
	GetAIPortsForDevice(GetData(device)->deviceName, &numAIPorts, GetData(device)->aiPorts_);
	//int numAIPorts = (int)sizeof(GetData(device)->aiPorts_) / sizeof(char*);
	// Count number of AI ports, assume AIports has 32 entries (TODO)
	/*
	int numAIPorts = 0;
	for (int i = 0; i < 32; i++) {
		if (GetData(device)->aiPorts_[i][0] != '0')
			numAIPorts++;
	}
	*/

	int numChannels = (numDispChannels > numAIPorts) ? numAIPorts : numDispChannels;
	struct OScNIDAQPrivateData* dData = GetData(device);
	for (int i = 0; i < numChannels; ++i)
	{
		sm_put(GetData(device)->channelMap_, dispChannels[i], GetData(device)->aiPorts_[i]);
		//GetData(device)->channelMap_[dispChannels.at(i)] = aiPorts_.at(i);
		// e.g., channelMap["Channel1"] = "PXI1Slot2/ai0";
	}

	return OSc_Error_OK;
}



// same to Initialize() in old OpenScan format
OSc_Error OpenDAQ(OSc_Device *device)
{
	OSc_Log_Debug(device, "Start initializing DAQ");
	OSc_Error err;
	if (OSc_Check_Error(err, MapDispChanToAIPorts(device))) {
		OSc_Log_Error(device, "Fail to init hash table");
	}
	struct OScNIDAQPrivateData* debug = GetData(device);
	// TODO: allow user to select these channels -- probably need a Hub structure

	char* deviceName = malloc(strlen(GetData(device)->deviceName));

	strcpy(deviceName, GetData(device)->deviceName);
	GetData(device)->aoChanList_ = malloc(sizeof(char) * 512);
	strcpy(GetData(device)->aoChanList_, strcat(deviceName, "/ao0:1"));

	strcpy(deviceName, GetData(device)->deviceName);
	GetData(device)->doChanList_ = malloc(sizeof(char) * 512);
	strcpy(GetData(device)->doChanList_, strcat(deviceName, "/port0/line5:7"));

	strcpy(deviceName, GetData(device)->deviceName);
	GetData(device)->coChanList_ = malloc(sizeof(char) * 512);
	strcpy(GetData(device)->aoChanList_, strcat(deviceName, "/ctr0"));

	strcpy(deviceName, GetData(device)->deviceName);
	GetData(device)->acqTrigPort_ = malloc(sizeof(char) * 512);
	char* noSlash = strcat(deviceName, "/PFI12");
	char* slash = "/";
	char* buffer = malloc((strlen(noSlash) + strlen(slash)) * sizeof(char));
	strcpy(buffer, slash);
	strcat(buffer, noSlash);
	strcpy(GetData(device)->acqTrigPort_,  buffer);
	OSc_Log_Debug(device, "Initializing NI DAQ...");
	OSc_Return_If_Error(InitDAQ(device));
	OSc_Return_If_Error(SetTriggers(device));

	++g_openDeviceCount;

	OSc_Log_Debug(device, "DAQ initialized");

	return OSc_Error_OK;
}


OSc_Error InitDAQ(OSc_Device *device)
{
	OSc_Return_If_Error(EnsureNIDAQInitialized());
	int32 nierr;

	// reset num of data read from PMTs
	// so that when restarting an imaging session from an unsuccessful acquisition
	// error won't occur due to mismatch between totalRead and resolution * totalSamplesPerLine
	GetData(device)->totalRead = 0; 

	// initialize scan waveform task
	if (!GetData(device)->scanWaveformTaskHandle_)
	{
		nierr = DAQmxCreateTask("", &GetData(device)->scanWaveformTaskHandle_);
		if (nierr != 0)
		{
			OSc_Log_Error(device, "Error creating scanWaveformTaskHandle: ");		
			char buf[1024];
			DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
			OSc_Log_Error(device, buf);
			return OSc_Error_Unknown_Enum_Value_Name;
		}
		OSc_Log_Debug(device, "Created scan waveform task");

		char aoTerminals[OSc_MAX_STR_LEN + 1];
		strcat(strcpy(aoTerminals, GetData(device)->deviceName), "/ao0:1");
		nierr = DAQmxCreateAOVoltageChan(GetData(device)->scanWaveformTaskHandle_, aoTerminals, "",
			-10.0, 10.0, DAQmx_Val_Volts, NULL);
		if (nierr != 0)
		{
			OSc_Log_Error(device, "Failed to create AO channel: ");
			char buf[1024];
			DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
			OSc_Log_Error(device, buf);
			goto Error;
		}

		OSc_Log_Debug(device, "Created AO voltage channels for X and Y scan waveforms");
	}

	// initialize line clock task
	if (!GetData(device)->lineClockTaskHandle_)
	{
		nierr = DAQmxCreateTask("", &GetData(device)->lineClockTaskHandle_);
		if (nierr != 0)
		{
			OSc_Log_Error(device, "Failed to create line clock task: ");
			char buf[1024];
			DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
			OSc_Log_Error(device, buf);
			return OSc_Error_Unknown_Enum_Value_Name;
		}
		OSc_Log_Debug(device, "Created line clock task");
		
		// wire p0.5 to /PXI1Slot2/PFI7 to trigger acquisition line by line
		// has to use port0 since it supports buffered opertation
		// p0.6 to generate line clock for FLIM
		// p0.7 to generate frame clock for FLIM
		//nierr = DAQmxCreateDOChan(lineClockTaskHandle_, "PXI1Slot2/port0/line5:6",
		//	"", DAQmx_Val_ChanForAllLines );

		char doTerminals[OSc_MAX_STR_LEN + 1];
		strcat(strcpy(doTerminals, GetData(device)->deviceName), "/port0/line5:7");
		nierr = DAQmxCreateDOChan(GetData(device)->lineClockTaskHandle_,  doTerminals,
			"", DAQmx_Val_ChanPerLine);
		if (nierr != 0)
		{
			OSc_Log_Error(device, "Failed to create DO channels");
			char buf[1024];
			DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
			OSc_Log_Error(device, buf);
			goto Error;
		}

		// get number of physical DO channels for image acquisition
		nierr = DAQmxGetReadNumChans(GetData(device)->lineClockTaskHandle_, &GetData(device)->numDOChannels);
		if (nierr != 0)
		{
			char buf[1024];
			DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
			OSc_Log_Error(device, buf);
			return OSc_Error_Unknown_Enum_Value_Name;
		}

		char msg[OSc_MAX_STR_LEN + 1];
		snprintf(msg, OSc_MAX_STR_LEN, "Created %d physical DO channels for line clocks and frame clock.", GetData(device)->numDOChannels);
		OSc_Log_Debug(device, msg);
	}

	// init counter
	if (!GetData(device)->counterTaskHandle_)
	{
		nierr = DAQmxCreateTask("", &GetData(device)->counterTaskHandle_);
		if (nierr != 0)
		{
			char buf[1024];
			DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
			OSc_Log_Error(device, buf);
			return OSc_Error_Unknown_Enum_Value_Name;
		}
		OSc_Log_Debug(device, "Created counter task.");

		// Create CO Channel.
		uint32_t elementsPerLine = X_UNDERSHOOT + GetData(device)->resolution + X_RETRACE_LEN;
		uint32_t scanLines = GetData(device)->resolution;
		double effectiveScanPortion = (double)GetData(device)->resolution / elementsPerLine;
		double lineFreq = (double)((1E6*GetData(device)->scanRate) / GetData(device)->binFactor / elementsPerLine);  // unit: Hz
		// adjustment corresponding to galvo undershoot at each line
	    // the delay (in second) between the command scan waveform and the actual scanner response
		double scanPhase = (double)(GetData(device)->binFactor / (1E6*GetData(device)->scanRate) * X_UNDERSHOOT);
		char counterTerminals[OSc_MAX_STR_LEN + 1];
		strcat(strcpy(counterTerminals, GetData(device)->deviceName), "/ctr0");
		nierr = DAQmxCreateCOPulseChanFreq(GetData(device)->counterTaskHandle_, counterTerminals,
			"", DAQmx_Val_Hz, DAQmx_Val_Low, scanPhase, lineFreq, effectiveScanPortion); // CTR0 OUT = PFI12
		if (nierr != 0)
		{
			goto Error;
		}

		// define how many lines to scan
		nierr = DAQmxCfgImplicitTiming(GetData(device)->counterTaskHandle_, DAQmx_Val_FiniteSamps, scanLines);
		if (nierr != 0)
		{
			char buf[1024];
			DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
			OSc_Log_Error(device, buf);
			goto Error;
		}
		OSc_Log_Debug(device, "Configured counter timing");

	}

	// initialize acquisition task
	if (!GetData(device)->acqTaskHandle_ || GetData(device)->channelSettingsChanged)
	{
		if (GetData(device)->acqTaskHandle_) // clear task first if reset is caused by channel settings
		{
			nierr = DAQmxStopTask(GetData(device)->acqTaskHandle_);
			nierr = DAQmxClearTask(GetData(device)->acqTaskHandle_);
			GetData(device)->acqTaskHandle_ = 0;
		}

		nierr = DAQmxCreateTask("", &GetData(device)->acqTaskHandle_);
		if (nierr != 0)
		{
			return OSc_Error_Unknown_Enum_Value_Name;
		}
		OSc_Log_Debug(device, "Created acquisition task");
		/*
		char aiTerminals[OSc_MAX_STR_LEN + 1];
		strcat(strcpy(aiTerminals, GetData(device)->deviceName), "/ai0:2");
		nierr = DAQmxCreateAIVoltageChan(GetData(device)->acqTaskHandle_, aiTerminals, "",
			DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL);
		if (nierr != 0)
		{
			char buf[1024];
			DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
			OSc_Log_Error(device, buf);
			goto Error;
		}
		OSc_Log_Debug(device, "Created AI voltage channels for image acquisition");
		// get number of physical AI channels for image acquisition
		// difference from GetNumberOfChannels() which indicates number of channels to display
		nierr = DAQmxGetReadNumChans(GetData(device)->acqTaskHandle_, &GetData(device)->numAIChannels);
		if (nierr != 0)
		{
			char buf[1024];
			DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
			OSc_Log_Error(device, buf);
			return OSc_Error_Unknown_Enum_Value_Name;
		}
		char msg[OSc_MAX_STR_LEN + 1];
		snprintf(msg, OSc_MAX_STR_LEN, "%d physical AI channels available.", GetData(device)->numAIChannels);
		OSc_Log_Debug(device, msg);
		*/

		OSc_Error err = ReconfigAIVoltageChannels(device);
		if (err != OSc_Error_OK)
		{
			OSc_Log_Error(device, "Error creating AI voltage channels");
			return err;
		}
	}

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
		
		OSc_Log_Error(device, "Failed initializing tasks; task cleared");
		err = OSc_Error_Unknown_Enum_Value_Name;
	}
	else
	{
		// TODO: Specify what exact error it is 
		err = OSc_Error_Unknown_Enum_Value_Name;
	}
	return err;

}

OSc_Error ReconfigAIVoltageChannels(OSc_Device* device)
{
	OSc_Error err = GetVoltageRangeForDevice(device, &GetData(device)->minVolts_, &GetData(device)->maxVolts_);
	char msg[OSc_MAX_STR_LEN + 1];
	snprintf(msg, OSc_MAX_STR_LEN, "Min Voltage: %6.2f; Max voltage: %6.2f", GetData(device)->minVolts_, GetData(device)->maxVolts_);
	OSc_Log_Debug(device, msg);

	// dynamically adjust ai ports according to which display channels are selected
	GetEnabledAIPorts(device);
	snprintf(msg, OSc_MAX_STR_LEN, "Enabling AI ports %s", GetData(device)->enabledAIPorts_);
	OSc_Log_Debug(device, msg);

	int32 nierr = DAQmxCreateAIVoltageChan(GetData(device)->acqTaskHandle_, GetData(device)->enabledAIPorts_, "",
		DAQmx_Val_Cfg_Default, GetData(device)->minVolts_, GetData(device)->maxVolts_, DAQmx_Val_Volts, NULL);
	if (nierr != 0)
	{
		goto Error;
	}
	OSc_Log_Debug(device, "Created AI voltage channels for image acquisition");

	// get number of physical AI channels for image acquisition
	// difference from GetNumberOfChannels() which indicates number of channels to display
	struct OScNIDAQPrivateData* debugData = GetData(device);

	nierr = DAQmxGetReadNumChans(GetData(device)->acqTaskHandle_, &GetData(device)->numAIChannels);
	if (nierr != 0)
	{
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		return nierr;
	}
	snprintf(msg, OSc_MAX_STR_LEN, "%d physical AI channels available.", GetData(device)->numAIChannels);
	OSc_Log_Debug(device, msg);

	return OSc_Error_OK;

Error:
	if (GetData(device)->acqTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->acqTaskHandle_);
		DAQmxClearTask(GetData(device)->acqTaskHandle_);
		GetData(device)->acqTaskHandle_ = 0;
	}

	if (nierr != 0)
	{
		err = OSc_Error_Unknown_Enum_Value_Name;
	}
	else
	{
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
static OSc_Error SetTriggers(OSc_Device *device)
{
	// Use AO StartTrigger to trigger the line clock.
	// This is an internal trigger signal.
	char aoStartTrigName[256];
	OSc_Error err;
	OSc_Return_If_Error(GetTerminalNameWithDevPrefix(GetData(device)->scanWaveformTaskHandle_,
		"ao/StartTrigger", aoStartTrigName));
	OSc_Log_Debug(device, "Get AO Start Trigger name to trigger line clock");

	// Configure counter
	// line clock generation is triggered by AO StartTrigger internally
	// and thus sync'ed to scan waveform generation
	int32 nierr = DAQmxCfgDigEdgeStartTrig(GetData(device)->counterTaskHandle_, aoStartTrigName, DAQmx_Val_Rising);
	if (nierr != 0)
	{
		OSc_Log_Error(device, "Error: cannot config counter trigger: ");
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}
	OSc_Log_Debug(device, "Configured digital edge start trigger for counter (line clock)");

	char acqTriggerSource[OSc_MAX_STR_LEN + 1] = "/";
	strcat(strcat(acqTriggerSource, GetData(device)->deviceName), "/PFI12");
	/*char msg[OSc_MAX_STR_LEN + 1];
	snprintf(msg, OSc_MAX_STR_LEN, "length of trigger souce %s is %d: ", acqTriggerSource, (int)strlen(acqTriggerSource) );
	OSc_Log_Debug(device, msg);*/

	nierr = DAQmxCfgDigEdgeStartTrig(GetData(device)->acqTaskHandle_, acqTriggerSource, DAQmx_Val_Rising);
	if (nierr != 0)
	{
		OSc_Log_Error(device, "Error: cannot config start trigger for acquisition");
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}
	OSc_Log_Debug(device, "Configured digital edge start trigger for image acquisition");

	nierr = DAQmxSetStartTrigRetriggerable(GetData(device)->acqTaskHandle_, 1);
	if (nierr != 0)
	{
		OSc_Log_Error(device, "Error: cannot set start trigger retriggable: ");
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
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
	
	err;
	if (nierr != 0)
	{
		OSc_Log_Error(device, "Failed setting triggers; task cleared");
		//err = TranslateNIError(nierr);
	}
	else
	{
		err = OSc_Error_Unknown;
	}
	
	return err;
}

static OSc_Error GetTerminalNameWithDevPrefix(TaskHandle taskHandle, const char terminalName[], char triggerName[])
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
	int err = GenerateGalvoWaveformFrame(GetData(device)->resolution, GetData(device)->zoom, 
		GetData(device)->offsetXY[0], GetData(device)->offsetXY[1], xyWaveformFrame);
	if (err != 0)
		return OSc_Error_Waveform_Out_Of_Range;

	int32 numWritten = 0;
	int32 nierr = DAQmxWriteAnalogF64(GetData(device)->scanWaveformTaskHandle_, totalElementsPerFramePerChan, FALSE, 10.0,
		DAQmx_Val_GroupByChannel, xyWaveformFrame, &numWritten, NULL);
	if (nierr != 0)
	{
		OSc_Log_Error(device, "Write scanwaveform error:");
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}
	if (numWritten != totalElementsPerFramePerChan)
	{
		OSc_Log_Error(device, "Failed to write complete scan waveform");
		goto Error;
	}
	OSc_Log_Debug(device, "One frame waveform written to DAQ memory");

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

	err;
	if (nierr != 0)
	{
		OSc_Log_Error(device, "Failed writing waveforms; task cleared");
		//err = TranslateNIError(nierr);
	}
	else
	{
		err = OSc_Error_Waveform_Out_Of_Range;
	}

	return err;

}


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
			char buf[1024];
			DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
			OSc_Log_Error(device, buf);
			goto Error;
		}
		OSc_Log_Debug(device, "Armed acquisition");
	}
	else {
		OSc_Log_Debug(device, "Dummy acquisition... scanner only.");
	}

	nierr = DAQmxStartTask(GetData(device)->counterTaskHandle_);
	if (nierr != 0)
	{
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}
	OSc_Log_Debug(device, "Armed counter (line clock) generation");

	nierr = DAQmxStartTask(GetData(device)->scanWaveformTaskHandle_);
	if (nierr != 0)
	{
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}
	OSc_Log_Debug(device, "Armed scan waveform generation. Starting scan...");
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
	
	int err;
	if (nierr != 0)
	{
		OSc_Log_Error(device, "Failed starting tasks; task cleared");
		//err = TranslateNIError(nierr);
	}
	else
	{
		err = OSc_Error_Unknown;
	}
	
	return err;
}


static OSc_Error StopScan(OSc_Device *device)
{
	int32 nierr;

	if (!GetData(device)->scannerOnly) {
		nierr = DAQmxStopTask(GetData(device)->acqTaskHandle_);
		if (nierr != 0)
		{
			char buf[1024], msg[OSc_MAX_STR_LEN + 1];
			DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
			snprintf(msg, OSc_MAX_STR_LEN, "Error stopping acquisition task: %d", (int)nierr);
			OSc_Log_Error(device, msg);
			OSc_Log_Error(device, buf);
			goto Error;
		}
		OSc_Log_Debug(device, "Stopped acquisition task");
	}
	else {
		OSc_Log_Debug(device, "Acquisition task skipped");
	}

	// When scanRate is low, it takes longer to finish generating scan waveform.
	// Since acquisition only takes a portion of the total scan time,
	// it may occur that waveform task is stopped right after acquisition is done
	// but the waveform generation is not done yet -- thus nierr 200010:
	// "Finite acquisition or generation has been stopped before the requested number
	// of samples were acquired or generated."
	// So need to wait some miliseconds till waveform generation is done before stop the task.
	uint32_t xLen = X_UNDERSHOOT + GetData(device)->resolution + X_RETRACE_LEN;
	uint32_t yLen = GetData(device)->resolution + Y_RETRACE_LEN;
	uint32_t yRetraceTime = (uint32_t)(1E-3 * (double)(xLen * Y_RETRACE_LEN * GetData(device)->binFactor / GetData(device)->scanRate));
	uint32_t estFrameTime = (uint32_t)(1E-3 * (double)(xLen * yLen * GetData(device)->binFactor / GetData(device)->scanRate));
	// TODO: casting
	uint32_t waitScanToFinish = GetData(device)->scannerOnly ? estFrameTime : yRetraceTime;  // wait longer if no real acquisition;
	char msg[OSc_MAX_STR_LEN + 1];
	snprintf(msg, OSc_MAX_STR_LEN, "Wait %d ms for scan to finish...", waitScanToFinish);
	OSc_Log_Debug(device, msg);
	Sleep(waitScanToFinish);

	nierr = DAQmxStopTask(GetData(device)->counterTaskHandle_);
	if (nierr != 0)
	{
		char buf[1024], msg[OSc_MAX_STR_LEN + 1];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		snprintf(msg, OSc_MAX_STR_LEN, "Error stopping counter task: %d", (int)nierr);
		OSc_Log_Error(device, msg);
		OSc_Log_Error(device, buf);
		goto Error;
	}
	OSc_Log_Debug(device, "Stopped counter (line clock) task");

	nierr = DAQmxStopTask(GetData(device)->scanWaveformTaskHandle_);
	if (nierr != 0)
	{
		char buf[1024], msg[OSc_MAX_STR_LEN + 1];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		snprintf(msg, OSc_MAX_STR_LEN, "Error stopping scan waveform task: %d", (int)nierr);
		OSc_Log_Error(device, msg);
		OSc_Log_Error(device, buf);
		goto Error;
	}
	OSc_Log_Debug(device, "Stopped scan waveform task");

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

	int err;
	if (nierr != 0)
	{
		OSc_Log_Error(device, "Failed stopping tasks; task cleared");
		//err = TranslateNIError(nierr);
	}
	else
	{
		err = OSc_Error_Unknown_Enum_Value_Name;
	}
	
	return err;

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
	//struct OScNIDAQPrivateData* debugData = GetData(device);
	//uint32_t resolution_ = GetData(device)->resolution;
	uint32_t elementsPerLine = X_UNDERSHOOT + GetData(device)->resolution + X_RETRACE_LEN;
	uint32_t scanLines = GetData(device)->resolution;
	int32 elementsPerFramePerChan = elementsPerLine * scanLines;
	size_t nPixels = GetImageWidth(device) * GetImageHeight(device);

	GetData(device)->imageData = realloc(GetData(device)->imageData,
		sizeof(uint16_t) * GetData(device)->numAIChannels * nPixels);
	GetData(device)->rawLineData = realloc(GetData(device)->rawLineData, 
		sizeof(float64) * GetData(device)->numAIChannels * GetData(device)->resolution * GetData(device)->binFactor);
	GetData(device)->avgLineData = realloc(GetData(device)->avgLineData,
		sizeof(float64) * GetData(device)->numAIChannels * GetData(device)->resolution);

	GetData(device)->ch1Buffer = realloc(GetData(device)->ch1Buffer, sizeof(uint16_t) * nPixels);
	GetData(device)->ch2Buffer = realloc(GetData(device)->ch2Buffer, sizeof(uint16_t) * nPixels);
	GetData(device)->ch3Buffer = realloc(GetData(device)->ch3Buffer, sizeof(uint16_t) * nPixels);

	GetData(device)->oneFrameScanDone = GetData(device)->scannerOnly;

	if (GetData(device)->scannerOnly) {
		// initialize channel buffers
		for (size_t i = 0; i < nPixels; ++i)
		{
			GetData(device)->ch1Buffer[i] = 0;
			GetData(device)->ch2Buffer[i] = 255;
			GetData(device)->ch3Buffer[i] = 32767;
		}
		for (size_t i = 0; i < GetData(device)->numAIChannels * nPixels; ++i)
		{
			GetData(device)->imageData[i] = 16383;
		}
	}

	uint32_t yLen = GetData(device)->resolution + Y_RETRACE_LEN;
	uint32_t estFrameTime = (uint32_t)(1E-3 * (double)(elementsPerLine * yLen * GetData(device)->binFactor / GetData(device)->scanRate));
	uint32_t totalWaitTime = 0;  // mSec
	OSc_Return_If_Error(StartScan(device));

	// Wait until one frame is scanned
	while (!GetData(device)->oneFrameScanDone) {
		Sleep(1);
		totalWaitTime += 1;
		if (totalWaitTime > 2 * estFrameTime)
		{
			OSc_Log_Error(device, "Error: Acquisition timeout!");
			break;
		}
	}
	char msg[OSc_MAX_STR_LEN + 1];
	snprintf(msg, OSc_MAX_STR_LEN, "Total wait time is %d ", totalWaitTime);
	OSc_Log_Debug(device, msg);

	OSc_Return_If_Error(StopScan(device));

	// SplitChannels
	// skik if set to scanner only mode
	if (!GetData(device)->scannerOnly)
	{
		OSc_Return_If_Error(SplitChannels(device));
	}

	switch (GetData(device)->channels)
	{
	case CHANNEL1:
		acq->frameCallback(acq, 0, GetData(device)->ch1Buffer, acq->data);
		break;
	case CHANNEL2:
		acq->frameCallback(acq, 0, GetData(device)->ch2Buffer, acq->data);
		break;
	case CHANNEL3:
		acq->frameCallback(acq, 0, GetData(device)->ch3Buffer, acq->data);
		break;
	
	case CHANNELS_1_AND_2:
		acq->frameCallback(acq, 0, GetData(device)->ch1Buffer, acq->data);
		acq->frameCallback(acq, 1, GetData(device)->ch2Buffer, acq->data);
		break;

	case CHANNELS1_2_3:
		acq->frameCallback(acq, 0, GetData(device)->ch1Buffer, acq->data);
		acq->frameCallback(acq, 1, GetData(device)->ch2Buffer, acq->data);
		acq->frameCallback(acq, 2, GetData(device)->ch3Buffer, acq->data);
		break;
	
	default:
		acq->frameCallback(acq, 0, GetData(device)->ch1Buffer, acq->data);
		acq->frameCallback(acq, 1, GetData(device)->ch2Buffer, acq->data);
		acq->frameCallback(acq, 2, GetData(device)->ch3Buffer, acq->data);
		break;
	}
	
	//Sleep(100);

	return OSc_Error_OK;
}

// split all-channel image buffer to separate channel buffers
// * works when DAQ acquires in GroupByChannel (non-interlaced) mode
static OSc_Error SplitChannels(OSc_Device *device)
{
	bool testImprov = true;
	// imageData_ if displayed as 2D image will have N channels on each row
	// data is stored line by line with N channels in a row per line
	uint32_t rawImageWidth = GetImageWidth(device) * GetData(device)->numAIChannels;
	uint32_t rawImageHeight = GetImageHeight(device);
	uint32_t xLength = GetImageWidth(device);
	uint32_t yLength = GetImageHeight(device);
	size_t nPixels = xLength * yLength;
	uint16_t* ch1Ptr;
	uint16_t* ch2Ptr;
	uint16_t* ch3Ptr;
	OSc_Error err = OSc_Error_OK;

	if (testImprov) {
		// Improvement: Use ptr to avoid hard copying
		switch (GetData(device)->channels)
		{
		case CHANNEL1:
			ch1Ptr = &(GetData(device)->imageData[0 * xLength]);
			memcpy(GetData(device)->ch1Buffer, ch1Ptr, nPixels * sizeof(uint16_t));
			break;
		case CHANNEL2:
			ch2Ptr = &(GetData(device)->imageData[0 * xLength]);
			memcpy(GetData(device)->ch2Buffer, ch2Ptr, nPixels * sizeof(uint16_t));
			break;
		case CHANNEL3:
			ch3Ptr = &(GetData(device)->imageData[0 * xLength]);
			memcpy(GetData(device)->ch3Buffer, ch3Ptr, nPixels * sizeof(uint16_t));
			break;

		case CHANNELS_1_AND_2:
			ch1Ptr = &(GetData(device)->imageData[0 * xLength]);
			ch2Ptr = &(GetData(device)->imageData[1 * xLength]);
			memcpy(GetData(device)->ch1Buffer, ch1Ptr, nPixels * sizeof(uint16_t));
			memcpy(GetData(device)->ch2Buffer, ch2Ptr, nPixels * sizeof(uint16_t));
			break;

		case CHANNELS1_2_3:
			ch1Ptr = &(GetData(device)->imageData[0 * xLength]);
			ch2Ptr = &(GetData(device)->imageData[1 * xLength]);
			ch3Ptr = &(GetData(device)->imageData[2 * xLength]);
			memcpy(GetData(device)->ch1Buffer, ch1Ptr, nPixels * sizeof(uint16_t));
			memcpy(GetData(device)->ch2Buffer, ch2Ptr, nPixels * sizeof(uint16_t));
			memcpy(GetData(device)->ch3Buffer, ch3Ptr, nPixels * sizeof(uint16_t));
			break;

		default:
			ch1Ptr = &(GetData(device)->imageData[0 * xLength]);
			ch2Ptr = &(GetData(device)->imageData[1 * xLength]);
			ch3Ptr = &(GetData(device)->imageData[2 * xLength]);
			memcpy(GetData(device)->ch1Buffer, ch1Ptr, nPixels * sizeof(uint16_t));
			memcpy(GetData(device)->ch2Buffer, ch2Ptr, nPixels * sizeof(uint16_t));
			memcpy(GetData(device)->ch3Buffer, ch3Ptr, nPixels * sizeof(uint16_t));
			break;
		}

	}



	else {
		uint16_t** imgBuffer;
		imgBuffer = malloc(sizeof(uint16_t*) * (GetData(device)->numAIChannels));
		for (int i = 0; i < GetData(device)->numAIChannels; i++) {
			imgBuffer[i] = calloc(nPixels, sizeof(uint16_t));
		}

		// convert big image buffer to separate channel buffers
		for (uint32_t chan = 0; chan < GetData(device)->numAIChannels; chan++)
			for (uint32_t currRow = 0; currRow < yLength; currRow++)
				for (uint32_t currCol = 0; currCol < xLength; currCol++) {
					imgBuffer[chan][currCol + currRow * xLength] =
						GetData(device)->imageData[currCol + chan*xLength + currRow*rawImageWidth];
				}


		switch (GetData(device)->channels)
		{
		case CHANNEL1:
			for (size_t i = 0; i < nPixels; i++)
				GetData(device)->ch1Buffer[i] = imgBuffer[0][i];
			break;
		case CHANNEL2:
			for (size_t i = 0; i < nPixels; i++)
				GetData(device)->ch2Buffer[i] = imgBuffer[0][i];
			break;
		case CHANNEL3:
			for (size_t i = 0; i < nPixels; i++)
				GetData(device)->ch3Buffer[i] = imgBuffer[0][i];
			break;

		case CHANNELS_1_AND_2:
			for (size_t i = 0; i < nPixels; i++) {
				GetData(device)->ch1Buffer[i] = imgBuffer[0][i];
				GetData(device)->ch2Buffer[i] = imgBuffer[1][i];
			}
			break;

		case CHANNELS1_2_3:
			for (size_t i = 0; i < nPixels; i++) {
				GetData(device)->ch1Buffer[i] = imgBuffer[0][i];
				GetData(device)->ch2Buffer[i] = imgBuffer[1][i];
				GetData(device)->ch3Buffer[i] = imgBuffer[2][i];
			}
			break;

		default:
			for (size_t i = 0; i < nPixels; i++) {
				GetData(device)->ch1Buffer[i] = imgBuffer[0][i];
				GetData(device)->ch2Buffer[i] = imgBuffer[1][i];
				GetData(device)->ch3Buffer[i] = imgBuffer[2][i];
			}
			break;
		}
		free(imgBuffer);
	}
	OSc_Log_Debug(device, "Finished reading one image and splitting data to channel buffers");
	return err;
}

// equal to SequenceThread::AcquireFrame()
static OSc_Error AcquireFrame(OSc_Device *device, OSc_Acquisition *acq)
{
	OSc_Log_Debug(device, "Reading image...");
	OSc_Return_If_Error(ReadImage(device, acq));
	OSc_Log_Debug(device, "Finished reading image");

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


// equal to SequenceThread::svc()
static DWORD WINAPI AcquisitionLoop(void *param)
{
	OSc_Device *device = (OSc_Device *)param;
	OSc_Acquisition *acq = GetData(device)->acquisition.acquisition;

	int totalFrames;
	if (acq->numberOfFrames == INT32_MAX)
		totalFrames = INT32_MAX;
	else 
		totalFrames = acq->numberOfFrames;

	for (int frame = 0; frame < totalFrames; ++frame)
	{
		bool stopRequested;
		EnterCriticalSection(&(GetData(device)->acquisition.mutex));
		stopRequested = GetData(device)->acquisition.stopRequested;
		LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
		if (stopRequested)
			break;

		char msg[OSc_MAX_STR_LEN + 1];
		snprintf(msg, OSc_MAX_STR_LEN, "Sequence acquiring frame # %d", frame);
		OSc_Log_Debug(device, msg);

		OSc_Error err;
		if (OSc_Check_Error(err, AcquireFrame(device, acq)))
		{
			char msg[OSc_MAX_STR_LEN + 1];
			snprintf(msg, OSc_MAX_STR_LEN, "Error during sequence acquisition: %d", (int)err);
			OSc_Log_Error(device, msg);
			FinishAcquisition(device);
			return 0;
		}
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

static OSc_Error ReconfigTiming(OSc_Device *device)
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
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}
	OSc_Log_Debug(device, "Configured sample clock timing for scan waveform");

	// by default, acquire data for one scan line each time
	nierr = DAQmxCfgSampClkTiming(GetData(device)->acqTaskHandle_, "", 1E6*GetData(device)->scanRate,
		DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, GetData(device)->resolution * GetData(device)->binFactor); // reload
	if (nierr != 0)
	{
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}
	OSc_Log_Debug(device, "Configured sample clock timing for acquistion");

	// manually increase acquisition buffer size to avoid input buffer overflow
	nierr = DAQmxCfgInputBuffer(GetData(device)->acqTaskHandle_, GetData(device)->numLinesToBuffer *
		GetData(device)->resolution * GetData(device)->binFactor * GetData(device)->numAIChannels);
	if (nierr != 0)
	{
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}
	char msg[OSc_MAX_STR_LEN + 1];
	snprintf(msg, OSc_MAX_STR_LEN, "Change acquisition buffer to size of %d scan lines", GetData(device)->numLinesToBuffer);
	OSc_Log_Debug(device, msg);

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
	OSc_Log_Debug(device, "Configured timing for counter generated line clock");

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
		OSc_Log_Error(device, "Error configuring timing; task cleared");
		//err = TranslateNIError(nierr);
		err = OSc_Error_Unknown;
	}
	else
	{
		err = OSc_Error_Unknown;
	}
	return err;

}


// DAQmx Commit the settings into hardware 
// This allows for very efficient restarts
static OSc_Error CommitTasks(OSc_Device *device)
{
	int32 nierr = DAQmxTaskControl(GetData(device)->acqTaskHandle_, DAQmx_Val_Task_Commit);
	if (nierr != 0)
	{
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}

	nierr = DAQmxTaskControl(GetData(device)->counterTaskHandle_, DAQmx_Val_Task_Commit);
	if (nierr != 0)
	{
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}

	nierr = DAQmxTaskControl(GetData(device)->scanWaveformTaskHandle_, DAQmx_Val_Task_Commit);
	if (nierr != 0)
	{
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}
	OSc_Log_Debug(device, "Committed DAQmx settings to hardware");

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
		OSc_Log_Error(device, "Error committing tasks; task cleared");
		err = OSc_Error_Unknown;
	}
	else
	{
		err = OSc_Error_Unknown;
	}
	return err;
}


// Unregister DAQ line acquisition event
static OSc_Error UnregisterLineAcqEvent(OSc_Device *device)
{
	int32 nierr = DAQmxRegisterEveryNSamplesEvent(GetData(device)->acqTaskHandle_, DAQmx_Val_Acquired_Into_Buffer,
		GetData(device)->resolution * GetData(device)->binFactor, 0, NULL, NULL);
	if (nierr != 0)
	{
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}
	OSc_Log_Debug(device, "Ungistered line acquisition callback event");

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
		OSc_Log_Error(device, "Error unregistering event; task cleared");
		err = OSc_Error_Unknown;
	}
	else
	{
		err = OSc_Error_Unknown;
	}
	return err;
}


// register DAQ line acquisition event
static OSc_Error RegisterLineAcqEvent(OSc_Device *device)
{
	// nSamples actually means nSamples per channel (refer to https://goo.gl/6zjMgB)
	int32 nierr = DAQmxRegisterEveryNSamplesEvent(GetData(device)->acqTaskHandle_, DAQmx_Val_Acquired_Into_Buffer,
		GetData(device)->resolution * GetData(device)->binFactor, 0, ReadLineCallback, device);  // readimage
	if (nierr != 0)
	{
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}
	char msg[OSc_MAX_STR_LEN + 1] = "Registered line acquisition callback event";
	OSc_Log_Debug(device, msg);
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
		OSc_Log_Error(device, "Failed registering EveryNSamplesEvent; task cleared");
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
static OSc_Error ReadLineCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData)
{
	OSc_Device *device= (OSc_Device*)(callbackData);

	int32 readPerChan;
	int32_t prevPercentRead = -1;
	uInt32 totalSamplesPerLine = GetData(device)->numAIChannels * GetData(device)->resolution * GetData(device)->binFactor;
	int32 currLine = 1 + GetData(device)->totalRead / GetData(device)->numAIChannels / GetData(device)->binFactor / GetData(device)->resolution;
	// rawLineData format with GroupByChannel (non-interlaced) is:
	// CH1 pixel 1 meas 1..binFactor | CH1 p2 m1..binFactor | ... | CH1 pN m1..binFactor || CH2 p1 m1..binFactor |...
	int32 nierr = DAQmxReadAnalogF64(GetData(device)->acqTaskHandle_, -1, 10.0, DAQmx_Val_GroupByChannel,
		GetData(device)->rawLineData, totalSamplesPerLine, &readPerChan, NULL);
	if (nierr != 0)
	{
		char msg[OSc_MAX_STR_LEN + 1];
		snprintf(msg, OSc_MAX_STR_LEN, "Reading line failed after line %d", currLine);
		OSc_Log_Debug(device, msg);
		char buf[1024];
		DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
		OSc_Log_Error(device, buf);
		goto Error;
	}

	if (readPerChan > 0)
	{
		//struct OScNIDAQPrivateData* debugData = GetData(device);
		//  append data line by line to the frame data array
		for (uint32_t i = 0; i < totalSamplesPerLine; i += GetData(device)->binFactor)
		{
			// pixel averaging
			GetData(device)->avgLineData[i / GetData(device)->binFactor] = GetData(device)->rawLineData[i] / GetData(device)->binFactor;
			for (unsigned j = 1; j < (unsigned)GetData(device)->binFactor; j++)
				GetData(device)->avgLineData[i / GetData(device)->binFactor] += (GetData(device)->rawLineData[i + j] / GetData(device)->binFactor);
			// convert processed line and append to output image frame
			GetData(device)->imageData[i / GetData(device)->binFactor + GetData(device)->totalRead / GetData(device)->binFactor] =
				(int16)abs(GetData(device)->avgLineData[i / GetData(device)->binFactor] / GetData(device)->inputVoltageRange * 32768);
		}

		GetData(device)->totalRead += (GetData(device)->numAIChannels * readPerChan); // update total elements acquired

		if (currLine % 128 == 0)
		{
			char msg[OSc_MAX_STR_LEN + 1];
			snprintf(msg, OSc_MAX_STR_LEN, "Read %d lines", currLine);
			OSc_Log_Debug(device, msg);
		}
	}
	else
	{
		OSc_Log_Error(device, "Callback received but no data read");
	}

	if (GetData(device)->totalRead == GetData(device)->resolution * totalSamplesPerLine)
	{
		GetData(device)->oneFrameScanDone = true;
		GetData(device)->totalRead = 0;
		OSc_Log_Debug(device, "End of scanning one frame");
	}

	return OSc_Error_OK;

Error:
	if (GetData(device)->acqTaskHandle_)
	{
		DAQmxStopTask(GetData(device)->acqTaskHandle_);
		DAQmxClearTask(GetData(device)->acqTaskHandle_);
		GetData(device)->acqTaskHandle_ = 0;
	}
	return OSc_Error_Unknown;
}


// Update DAQ configurations when settings change
OSc_Error ReconfigDAQ(OSc_Device * device)
{
	// if any of DAQ tasks are not initialized
	if (!GetData(device)->scanWaveformTaskHandle_ || !GetData(device)->lineClockTaskHandle_ ||
		!GetData(device)->acqTaskHandle_ || !GetData(device)->counterTaskHandle_ || GetData(device)->channelSettingsChanged)
	{
		OSc_Log_Debug(device, "Re-initializing NI DAQ...");
		OSc_Return_If_Error(InitDAQ(device));
		GetData(device)->channelSettingsChanged = false;

		OSc_Return_If_Error(SetTriggers(device));
		OSc_Log_Debug(device, "DAQ re-initialized.");

		// all the timing, waveforms, etc. have to reset as well
		// as new tasks, although bear the same names as previously failed/cleared ones,
		// have different memory address (pointers) and their relationship to
		// timing, triggers, etc. need to be re-established.
		GetData(device)->timingSettingsChanged = true;
		GetData(device)->waveformSettingsChanged = true;
		GetData(device)->acqSettingsChanged = true;
	}

	if (GetData(device)->timingSettingsChanged)
	{
		OSc_Log_Debug(device, "Reconfiguring timing...");
		OSc_Return_If_Error(ReconfigTiming(device));
		GetData(device)->timingSettingsChanged = false;
		GetData(device)->settingsChanged = true;
	}

	if (GetData(device)->waveformSettingsChanged)
	{
		OSc_Log_Debug(device, "Writing scan waveform and line clock pattern to DAQ...");
		OSc_Return_If_Error(WriteWaveforms(device));
		GetData(device)->waveformSettingsChanged = false;
		GetData(device)->settingsChanged = true;
	}

	// first check if existing EveryNSamplesEvent needs to be unregistered
	// to allow new event to get registered when acqSettings has changed since previous scan
	if (GetData(device)->acqSettingsChanged && GetData(device)->isEveryNSamplesEventRegistered && !GetData(device)->scannerOnly)
	{
		OSc_Return_If_Error(UnregisterLineAcqEvent(device));
		GetData(device)->isEveryNSamplesEventRegistered = false;
	}

	// Re-register event when resolution or binFactor has changed
	if (GetData(device)->acqSettingsChanged && !GetData(device)->scannerOnly)
	{
		OSc_Return_If_Error(RegisterLineAcqEvent(device));
		GetData(device)->acqSettingsChanged = false;
		GetData(device)->settingsChanged = true;
		GetData(device)->isEveryNSamplesEventRegistered = true;
	}

	/*
	// Re-create channel mapping when channel setting has changed
	if (GetData(device)->channelSettingsChanged) {
		OSc_Log_Debug(device, "Channel setting changed. Reconfiguring channel mapping...");
		//OSc_Return_If_Error(MapDispChanToAIPorts(device));
		OSc_Return_If_Error(ReconfigAIVoltageChannels(device));
		GetData(device)->channelSettingsChanged = false;
		GetData(device)->settingsChanged = true;

	}
	*/

	// commit tasks whenever settings have changed
	if (GetData(device)->settingsChanged)
	{
		OSc_Return_If_Error(CommitTasks(device));
		GetData(device)->settingsChanged = false;
	}

	return OSc_Error_OK;
}
