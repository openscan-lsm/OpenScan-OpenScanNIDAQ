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


// Must be called immediately after failed DAQmx function
void LogNiError(OScDev_Device *device, int32 nierr, const char *when)
{
	char buf[1024];
	strncpy(buf, "DAQmx error while ", sizeof(buf) - 1);
	strncat(buf, when, sizeof(buf) - strlen(buf) - 1);
	strncat(buf, "; extended error info follows", sizeof(buf) - strlen(buf) - 1);
	OScDev_Log_Error(device, buf);

	DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
	OScDev_Log_Error(device, buf);
}


static inline uint16_t DoubleToFixed16(double d, int intBits)
{
	int fracBits = 16 - intBits;
	return (uint16_t)round(d * (1 << fracBits));
}


static void PopulateDefaultParameters(struct OScNIDAQPrivateData *data)
{
	data->detectorOnly = false;
	data->scannerOnly = false;
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

	memset(&data->clockConfig, 0, sizeof(struct ClockConfig));
	memset(&data->scannerConfig, 0, sizeof(struct ScannerConfig));
	memset(&data->detectorConfig, 0, sizeof(struct DetectorConfig));

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

// convert comma comma - delimited device list to a 2D string array
// each row contains the name of one ai port
static OScDev_Error ParseAIPortList(char *names,
	// assume there are maximum 256 port 
	char deviceNames[MAX_NUM_PORTS][32], int *deviceCount)
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
			return OScDev_Error_Unknown;  //TODO
	}

	*deviceCount = (size_t)count;

	return OScDev_OK;
}



// automatically detect deviceName using DAQmxGetSysDevNames()
OScDev_Error NIDAQEnumerateInstances(OScDev_Device ***devices, size_t *deviceCount)
{
	OScDev_Error err;

	// get a comma - delimited list of all of the devices installed in the system
	char deviceNames[4096];
	int32 nierr = DAQmxGetSysDevNames(deviceNames, sizeof(deviceNames));
	if (nierr != 0)
	{
		return OScDev_Error_Unknown;  //TODO
	}

	char deviceList[NUM_SLOTS_IN_CHASSIS][OScDev_MAX_STR_LEN + 1];

	if (OScDev_CHECK(err, ParseDeviceNameList(deviceNames, deviceList, deviceCount)))
		return err;

	*devices = malloc(*deviceCount * sizeof(OScDev_Device *));

	for (int i = 0; i < (int)(*deviceCount); ++i)
	{
		struct OScNIDAQPrivateData *data = calloc(1, sizeof(struct OScNIDAQPrivateData));
		strncpy(data->deviceName, deviceList[i], OScDev_MAX_STR_LEN);

		OScDev_Device *device;
		if (OScDev_CHECK(err, OScDev_Device_Create(&device, &OpenScan_NIDAQ_Device_Impl, data)))
		{
			char msg[OScDev_MAX_STR_LEN + 1] = "Failed to create device ";
			strcat(msg, data->deviceName);
			OScDev_Log_Error(device, msg);
			return err;
		}

		PopulateDefaultParameters(GetData(device));

		(*devices)[i] = device;
	}
	
	return OScDev_OK;
}


OScDev_Error GetAIPortsForDevice(char* devices, int* deviceCount, char** result) {
	char ports[4096];
	int32 nierr = DAQmxGetDevAIPhysicalChans(devices, ports, sizeof(ports));
	if (nierr != 0)
	{
		return OScDev_Error_Unknown;  //TODO
	}

	// TODO: Max number of AI ports
	char portList[256][32];

	OScDev_Error err;
	if (OScDev_CHECK(err, ParseAIPortList(ports, portList, deviceCount)))
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
	return OScDev_OK;
}


OScDev_Error GetVoltageRangeForDevice(OScDev_Device* device, double* minVolts, double* maxVolts){
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
		OScDev_Log_Error(device, "Error getting analog voltage ranges");
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

	return OScDev_OK;
}


OScDev_Error GetEnabledAIPorts(OScDev_Device *device) {
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
	return OScDev_OK;
}


// convert comma comma - delimited device list to a 2D string array
// each row contains the name of one device
static OScDev_Error ParseDeviceNameList(char *names,
	char (*deviceNames)[OScDev_MAX_STR_LEN + 1], size_t *deviceCount)
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
			return OScDev_Error_Unknown;  //TODO
	}

	*deviceCount = (size_t)count;

	return OScDev_OK;
}


OScDev_Error MapDispChanToAIPorts(OScDev_Device* device)
{
	struct OScNIDAQPrivateData* debug = GetData(device);
	char dispChannels[3][512] = {
		{"Channel1"},
		{"Channel2"},
		{"Channel3"}
	};

	int numDispChannels = 3;
	int numAIPorts = 1;
	GetAIPortsForDevice(GetData(device)->deviceName, &numAIPorts, GetData(device)->aiPorts_);

	int numChannels = (numDispChannels > numAIPorts) ? numAIPorts : numDispChannels;
	struct OScNIDAQPrivateData* dData = GetData(device);
	for (int i = 0; i < numChannels; ++i)
	{
		sm_put(GetData(device)->channelMap_, dispChannels[i], GetData(device)->aiPorts_[i]);
	}

	return OScDev_OK;
}


// same to Initialize() in old OpenScan format
OScDev_Error OpenDAQ(OScDev_Device *device)
{
	OScDev_Log_Debug(device, "Start initializing DAQ");
	OScDev_Error err;
	if (OScDev_CHECK(err, MapDispChanToAIPorts(device))) {
		OScDev_Log_Error(device, "Fail to init hash table");
	}
	struct OScNIDAQPrivateData* debug = GetData(device);
	// TODO: allow user to select these channels -- probably need a Hub structure

	// TODO BUG The string manipulation below has buffer overflows and also
	// leaks memory.

	char* deviceName = malloc(strlen(GetData(device)->deviceName));

	strcpy(deviceName, GetData(device)->deviceName);
	GetData(device)->aoChanList_ = malloc(sizeof(char) * 512);
	strcpy(GetData(device)->aoChanList_, strcat(deviceName, "/ao0:1"));

	strcpy(deviceName, GetData(device)->deviceName);
	GetData(device)->doChanList_ = malloc(sizeof(char) * 512);
	strcpy(GetData(device)->doChanList_, strcat(deviceName, "/port0/line5:7"));

	strcpy(deviceName, GetData(device)->deviceName);
	GetData(device)->coChanList_ = malloc(sizeof(char) * 512);
	strcpy(GetData(device)->coChanList_, strcat(deviceName, "/ctr0"));

	strcpy(deviceName, GetData(device)->deviceName);
	GetData(device)->acqTrigPort_ = malloc(sizeof(char) * 512);
	char* noSlash = strcat(deviceName, "/PFI12");
	char* slash = "/";
	char* buffer = malloc((strlen(noSlash) + strlen(slash)) * sizeof(char));
	strcpy(buffer, slash);
	strcat(buffer, noSlash);
	strcpy(GetData(device)->acqTrigPort_,  buffer);

	return OScDev_OK;
}


OScDev_Error CloseDAQ(OScDev_Device *device)
{
	//TODO
	//StopAcquisitionAndWait(device, acq);

	return OScDev_OK;
}


static OScDev_Error GetTerminalNameWithDevPrefix(TaskHandle taskHandle, const char terminalName[], char triggerName[])
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
	return OScDev_OK;
}


// DAQ version; start all tasks
// Arm acquisition task first. Then make sure the (digital) line clock output 
// is armed before the (analog) waveform output. 
// This will ensure both tasks will start at the same time.
static OScDev_Error StartScan(OScDev_Device *device)
{
	OScDev_Error err;
	if (!GetData(device)->scannerOnly) {
		if (OScDev_CHECK(err, StartDetector(device, &GetData(device)->detectorConfig)))
			return err;
	}		
	else
		OScDev_Log_Debug(device, "DAQ not used as detector");

	if (OScDev_CHECK(err, StartClock(device, &GetData(device)->clockConfig)))
		return err;

	if (OScDev_CHECK(err, StartScanner(device, &GetData(device)->scannerConfig)))
		return err;

	return OScDev_OK;
}


static OScDev_Error WaitScanToFinish(OScDev_Device *device)
{
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
	char msg[OScDev_MAX_STR_LEN + 1];
	snprintf(msg, OScDev_MAX_STR_LEN, "Wait %d ms for scan to finish...", waitScanToFinish);
	OScDev_Log_Debug(device, msg);
	Sleep(waitScanToFinish);

	return OScDev_OK;
}



// stop running tasks
// need to stop detector first, then clock and scanner
static OScDev_Error StopScan(OScDev_Device *device)
{
	OScDev_Error err, lastErr = 0;

	// Stopping a task may return an error if it failed, so make sure to stop
	// all tasks even if we get errors.

	if (!GetData(device)->scannerOnly) {
		err = StopDetector(device, &GetData(device)->detectorConfig);
		if (err)
			lastErr = err;
	}

	// When scanRate is low, it takes longer to finish generating scan waveform.
	// Since acquisition only takes a portion of the total scan time,
	// it may occur that waveform task is stopped right after acquisition is done
	// but the waveform generation is not done yet -- thus nierr 200010:
	// "Finite acquisition or generation has been stopped before the requested number
	// of samples were acquired or generated."
	// So need to wait some miliseconds till waveform generation is done before stop the task.
	if (OScDev_CHECK(err, WaitScanToFinish(device)))
		return err;

	err = StopClock(device, &GetData(device)->clockConfig);
	if (err)
		lastErr = err;

	err = StopScanner(device, &GetData(device)->scannerConfig);
	if (err)
		lastErr = err;

	return lastErr;
}


static unsigned GetImageWidth(OScDev_Device *device) {
	return  GetData(device)->resolution;
}

static unsigned GetImageHeight(OScDev_Device *device) {
	return  GetData(device)->resolution;
}

// DAQ version; acquire from multiple channels
static OScDev_Error ReadImage(OScDev_Device *device, OScDev_Acquisition *acq)
{
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

	GetData(device)->oneFrameScanDone = false;

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
	uint32_t estFrameTimeMs = (uint32_t)(1E-3 * (double)(elementsPerLine * yLen * GetData(device)->binFactor / GetData(device)->scanRate));
	uint32_t totalWaitTimeMs = 0;

	OScDev_Error err;
	if (OScDev_CHECK(err, StartScan(device)))
		return err;

	// Wait for scan to complete
	int32 nierr = DAQmxWaitUntilTaskDone(GetData(device)->scannerConfig.aoTask,
		2 * estFrameTimeMs * 1e-3);
	if (nierr)
		LogNiError(device, nierr, "waiting for scanner task to finish");

	if (OScDev_CHECK(err, StopScan(device)))
		return err;

	// Wait for data
	if (!GetData(device)->scannerOnly)
	{
		while (!GetData(device)->oneFrameScanDone) {
			Sleep(1);
			totalWaitTimeMs += 1;
			if (totalWaitTimeMs > 2 * estFrameTimeMs)
			{
				OScDev_Log_Error(device, "Error: Acquisition timeout!");
				break;
			}
		}
		char msg[OScDev_MAX_STR_LEN + 1];
		snprintf(msg, OScDev_MAX_STR_LEN, "Total wait time is %d ", totalWaitTimeMs);
		OScDev_Log_Debug(device, msg);
	}

	// SplitChannels
	// skik if set to scanner only mode
	if (!GetData(device)->scannerOnly)
	{
		if (OScDev_CHECK(err, SplitChannels(device)))
			return err;

		bool shouldContinue;
		switch (GetData(device)->channels)
		{
		case CHANNEL1:
			shouldContinue = OScDev_Acquisition_CallFrameCallback(acq, 0, GetData(device)->ch1Buffer);
			break;
		case CHANNEL2:
			shouldContinue = OScDev_Acquisition_CallFrameCallback(acq, 0, GetData(device)->ch2Buffer);
			break;
		case CHANNEL3:
			shouldContinue = OScDev_Acquisition_CallFrameCallback(acq, 0, GetData(device)->ch3Buffer);
			break;

		case CHANNELS_1_AND_2:
			shouldContinue = OScDev_Acquisition_CallFrameCallback(acq, 0, GetData(device)->ch1Buffer) &&
				OScDev_Acquisition_CallFrameCallback(acq, 1, GetData(device)->ch2Buffer);
			break;

		case CHANNELS_1_AND_3:
			shouldContinue = OScDev_Acquisition_CallFrameCallback(acq, 0, GetData(device)->ch1Buffer) &&
				OScDev_Acquisition_CallFrameCallback(acq, 1, GetData(device)->ch3Buffer);
			break;

		case CHANNELS1_2_3:
		default: // TODO Should this really be the default?
			shouldContinue = OScDev_Acquisition_CallFrameCallback(acq, 0, GetData(device)->ch1Buffer) &&
				OScDev_Acquisition_CallFrameCallback(acq, 1, GetData(device)->ch2Buffer) &&
				OScDev_Acquisition_CallFrameCallback(acq, 2, GetData(device)->ch3Buffer);
			break;
		}

		if (!shouldContinue) {
			// TODO We should halt acquisition
		}
	}

	return OScDev_OK;
}

// split all-channel image buffer to separate channel buffers
// * works when DAQ acquires in GroupByChannel (non-interlaced) mode
static OScDev_Error SplitChannels(OScDev_Device *device)
{
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
	OScDev_Error err = OScDev_OK;

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
		for (uint32_t i = 0; i < yLength; i++) {
			memcpy(&(GetData(device)->ch1Buffer[i*xLength]), &ch1Ptr[i*rawImageWidth], xLength * sizeof(uint16_t));
			memcpy(&(GetData(device)->ch2Buffer[i*xLength]), &ch2Ptr[i*rawImageWidth], xLength * sizeof(uint16_t));
		}
		break;

	case CHANNELS_1_AND_3:
		ch1Ptr = &(GetData(device)->imageData[0 * xLength]);
		ch3Ptr = &(GetData(device)->imageData[1 * xLength]);
		for (uint32_t i = 0; i < yLength; i++) {
			memcpy(&(GetData(device)->ch1Buffer[i*xLength]), &ch1Ptr[i*rawImageWidth], xLength * sizeof(uint16_t));
			memcpy(&(GetData(device)->ch3Buffer[i*xLength]), &ch3Ptr[i*rawImageWidth], xLength * sizeof(uint16_t));
		}
		break;

	case CHANNELS1_2_3:
	default: // TODO Should this be the default?
		ch1Ptr = &(GetData(device)->imageData[0 * xLength]);
		ch2Ptr = &(GetData(device)->imageData[1 * xLength]);
		ch3Ptr = &(GetData(device)->imageData[2 * xLength]);
		for (uint32_t i = 0; i < yLength; i++) {
			memcpy(&(GetData(device)->ch1Buffer[i*xLength]), &ch1Ptr[i*rawImageWidth], xLength * sizeof(uint16_t));
			memcpy(&(GetData(device)->ch2Buffer[i*xLength]), &ch2Ptr[i*rawImageWidth], xLength * sizeof(uint16_t));
			memcpy(&(GetData(device)->ch3Buffer[i*xLength]), &ch3Ptr[i*rawImageWidth], xLength * sizeof(uint16_t));
		}
		break;
	}

	OScDev_Log_Debug(device, "Finished reading one image and splitting data to channel buffers");
	return err;
}

// equal to SequenceThread::AcquireFrame()
static OScDev_Error AcquireFrame(OScDev_Device *device, OScDev_Acquisition *acq)
{
	OScDev_Error err;
	OScDev_Log_Debug(device, "Reading image...");
	if (OScDev_CHECK(err, ReadImage(device, acq)))
		return err;
	OScDev_Log_Debug(device, "Finished reading image");

	return OScDev_OK;
}


static void FinishAcquisition(OScDev_Device *device)
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
	OScDev_Device *device = (OScDev_Device *)param;
	OScDev_Acquisition *acq = GetData(device)->acquisition.acquisition;

	uint32_t totalFrames;
	OScDev_Error err;
	if (OScDev_CHECK(err, OScDev_Acquisition_GetNumberOfFrames(acq, &totalFrames)))
		return 0;

	for (uint32_t frame = 0; frame < totalFrames; ++frame)
	{
		bool stopRequested;
		EnterCriticalSection(&(GetData(device)->acquisition.mutex));
		stopRequested = GetData(device)->acquisition.stopRequested;
		LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
		if (stopRequested)
			break;

		char msg[OScDev_MAX_STR_LEN + 1];
		snprintf(msg, OScDev_MAX_STR_LEN, "Sequence acquiring frame # %d", frame);
		OScDev_Log_Debug(device, msg);

		OScDev_Error err;
		if (OScDev_CHECK(err, AcquireFrame(device, acq)))
		{
			char msg[OScDev_MAX_STR_LEN + 1];
			snprintf(msg, OScDev_MAX_STR_LEN, "Error during sequence acquisition: %d", (int)err);
			OScDev_Log_Error(device, msg);
			FinishAcquisition(device);
			return 0;
		}
	}

	FinishAcquisition(device);
	return 0;
}


OScDev_Error RunAcquisitionLoop(OScDev_Device *device)
{
	DWORD id;
	GetData(device)->acquisition.thread =
		CreateThread(NULL, 0, AcquisitionLoop, device, 0, &id);
	return OScDev_OK;
}



OScDev_Error StopAcquisitionAndWait(OScDev_Device *device)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (!GetData(device)->acquisition.running)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			return OScDev_OK;
		}

		GetData(device)->acquisition.stopRequested = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return WaitForAcquisitionToFinish(device);
}


OScDev_Error IsAcquisitionRunning(OScDev_Device *device, bool *isRunning)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	*isRunning = GetData(device)->acquisition.running;
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return OScDev_OK;
}


OScDev_Error WaitForAcquisitionToFinish(OScDev_Device *device)
{
	OScDev_Error err = OScDev_OK;
	CONDITION_VARIABLE *cv = &(GetData(device)->acquisition.acquisitionFinishCondition);

	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	while (GetData(device)->acquisition.running)
	{
		SleepConditionVariableCS(cv, &(GetData(device)->acquisition.mutex), INFINITE);
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return err;
}


OScDev_Error ReconfigDAQ(OScDev_Device *device)
{
	OScDev_Error err;

	err = SetUpClock(device, &GetData(device)->clockConfig);
	if (err)
		return err;
	err = SetUpScanner(device, &GetData(device)->scannerConfig);
	if (err)
		return err;
	if (!GetData(device)->scannerOnly)
	{
		err = SetUpDetector(device, &GetData(device)->detectorConfig);
		if (err)
			return err;
	}

	return OScDev_OK;
}