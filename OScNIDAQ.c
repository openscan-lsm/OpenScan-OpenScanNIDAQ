/* Device-specific scanner and/or detector implementations */
/* Interact with physical hardware (NI DAQ) */
/* and rely on specific device library (NIDAQmx) */

#include "OScNIDAQ.h"
#include "Waveform.h"
#include "strmap/strmap.h"
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


char* ErrorCodeDomain()
{
	static char* domainName = NULL;
	if (domainName == NULL) {
		domainName = "NI DAQmx";
		OScDev_Error_RegisterCodeDomain(domainName, OScDev_ErrorCodeFormat_I32);
	}
	return domainName;
}


// Must be called immediately after failed DAQmx function
OScDev_RichError *CreateDAQmxError(int32 nierr)
{

	char buf[1024];
	DAQmxGetExtendedErrorInfo(buf, sizeof(buf));

	if (nierr > 0)
		OScDev_Log_Warning(NULL, buf);

	if (nierr >= 0)
		return OScDev_RichError_OK;

	return OScDev_Error_CreateWithCode(ErrorCodeDomain(), nierr, buf);
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
	data->framePixelsFilled = 0;

	data->lineDelay = 50;
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
OScDev_RichError *EnumerateInstances(OScDev_PtrArray **devices, OScDev_DeviceImpl *impl)
{
	OScDev_RichError *err;

	// get a comma - delimited list of all of the devices installed in the system
	char deviceNames[4096];
	err = CreateDAQmxError(DAQmxGetSysDevNames(deviceNames, sizeof(deviceNames)));
	if (err)
		return err;

	char deviceList[NUM_SLOTS_IN_CHASSIS][OScDev_MAX_STR_LEN + 1];

	size_t deviceCount;
	err = ParseDeviceNameList(deviceNames, deviceList, &deviceCount);
	if (err)
		return err;

	*devices = OScDev_PtrArray_Create();

	for (size_t i = 0; i < deviceCount; ++i)
	{
		struct OScNIDAQPrivateData *data = calloc(1, sizeof(struct OScNIDAQPrivateData));
		strncpy(data->deviceName, deviceList[i], OScDev_MAX_STR_LEN);

		OScDev_Device *device;
		err = OScDev_Error_AsRichError(OScDev_Device_Create(&device, impl, data));
		if (err)
		{
			char msg[OScDev_MAX_STR_LEN + 1] = "Failed to create device ";
			strcat(msg, data->deviceName);
			err = OScDev_Error_Wrap(err, msg);
			return err;
		}

		PopulateDefaultParameters(GetData(device));

		OScDev_PtrArray_Append(*devices, device);
	}
	
	return OScDev_RichError_OK;
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
	char* portList = malloc(1);
	struct OScNIDAQPrivateData* debug = GetData(device);
	// Assume three channels at most
	int channelNum = GetData(device)->channelCount;
	for (int i = 0; i < channelNum; i++) {
		char mappedStr[255]; // Assume string len is less than 255 char
		sm_get(GetData(device)->channelMap_, GetData(device)->selectedDispChan_[i],  mappedStr, sizeof(mappedStr));
		// Append comma
		// port0, port1, port3...
		if (i == 0) {
			portList = realloc(portList, strlen(mappedStr) * sizeof(char));
			strcpy(portList, mappedStr);
		}
		else {
			char* buffer = realloc(portList, (strlen(portList) + strlen(mappedStr) + 1) * sizeof(char));
			strcpy(buffer, portList);
			strcat(buffer, ",");
			strcat(buffer, mappedStr);
			portList = buffer;
		}
	
	}
	GetData(device)->enabledAIPorts_ = portList;
	free(portList);
	return OScDev_OK;
}


// convert comma comma - delimited device list to a 2D string array
// each row contains the name of one device
static OScDev_RichError *ParseDeviceNameList(char *names,
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
			return OScDev_Error_Create("Error Unknown");
	}

	*deviceCount = (size_t)count;

	return OScDev_RichError_OK;
}


OScDev_RichError *MapDispChanToAIPorts(OScDev_Device* device)
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

	return OScDev_RichError_OK;
}


// same to Initialize() in old OpenScan format
OScDev_RichError *OpenDAQ(OScDev_Device *device)
{
	OScDev_Log_Debug(device, "Start initializing DAQ");
	OScDev_RichError *err;
	err = MapDispChanToAIPorts(device);
	if (err)
		return OScDev_Error_Wrap(err, "Fail to init hash table");

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

	// ???
	return OScDev_RichError_OK;
}


OScDev_RichError *CloseDAQ(OScDev_Device *device)
{
	//TODO
	//StopAcquisitionAndWait(device, acq);

	return OScDev_RichError_OK;
}


static OScDev_RichError *GetTerminalNameWithDevPrefix(TaskHandle taskHandle, const char terminalName[], char triggerName[])
{
	int32	error = 0;
	char	device[256];
	int32	productCategory;
	uInt32	numDevices, i = 1;
	OScDev_RichError *err;

	err = CreateDAQmxError(DAQmxGetTaskNumDevices(taskHandle, &numDevices));
	if (err)
		return err;
	while (i <= numDevices) {
		err = CreateDAQmxError(DAQmxGetNthTaskDevice(taskHandle, i++, device, 256));
		if (err)
			return err;
		err = CreateDAQmxError(DAQmxGetDevProductCategory(device, &productCategory));
		if (err)
			return err;
		if (productCategory != DAQmx_Val_CSeriesModule && productCategory != DAQmx_Val_SCXIModule) {
			*triggerName++ = '/';
			strcat(strcat(strcpy(triggerName, device), "/"), terminalName);
			break;
		}
	}
	return OScDev_RichError_OK;
}


// DAQ version; start all tasks
// Arm acquisition task first. Then make sure the (digital) line clock output 
// is armed before the (analog) waveform output. 
// This will ensure both tasks will start at the same time.
static OScDev_RichError *StartScan(OScDev_Device *device)
{
	OScDev_RichError *err;
	if (!GetData(device)->scannerOnly) {
		err = StartDetector(device, &GetData(device)->detectorConfig);
		if (err)
			return err;
	}		
	else
		OScDev_Log_Debug(device, "DAQ not used as detector");

	err = StartClock(device, &GetData(device)->clockConfig);
	if (err)
		return err;

	err = StartScanner(device, &GetData(device)->scannerConfig);
	if (err)
		return err;

	return OScDev_RichError_OK;
}


static OScDev_RichError *WaitScanToFinish(OScDev_Device *device, OScDev_Acquisition *acq)
{
	double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
	uint32_t xOffset, yOffset, width, height;
	OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);

	// When scanRate is low, it takes longer to finish generating scan waveform.
	// Since acquisition only takes a portion of the total scan time,
	// it may occur that waveform task is stopped right after acquisition is done
	// but the waveform generation is not done yet -- thus nierr 200010:
	// "Finite acquisition or generation has been stopped before the requested number
	// of samples were acquired or generated."
	// So need to wait some miliseconds till waveform generation is done before stop the task.
	uint32_t xLen = GetData(device)->lineDelay + width + X_RETRACE_LEN;
	uint32_t yLen = height + Y_RETRACE_LEN;
	uint32_t yRetraceTime = (uint32_t)(1e3 * xLen * Y_RETRACE_LEN * GetData(device)->binFactor / pixelRateHz);
	uint32_t estFrameTime = (uint32_t)(1e3 * xLen * yLen * GetData(device)->binFactor / pixelRateHz);
	// TODO: casting
	uint32_t waitScanToFinish = GetData(device)->scannerOnly ? estFrameTime : yRetraceTime;  // wait longer if no real acquisition;
	char msg[OScDev_MAX_STR_LEN + 1];
	snprintf(msg, OScDev_MAX_STR_LEN, "Wait %d ms for scan to finish...", waitScanToFinish);
	OScDev_Log_Debug(device, msg);
	Sleep(waitScanToFinish);

	return OScDev_RichError_OK;
}



// stop running tasks
// need to stop detector first, then clock and scanner
static OScDev_RichError *StopScan(OScDev_Device *device, OScDev_Acquisition *acq)
{
	OScDev_RichError *err, *lastErr = OScDev_RichError_OK;

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
	err = WaitScanToFinish(device, acq);
	if (err)
		return err;

	err = StopClock(device, &GetData(device)->clockConfig);
	if (err)
		lastErr = err;

	err = StopScanner(device, &GetData(device)->scannerConfig);
	if (err)
		lastErr = err;

	return lastErr;
}


// DAQ version; acquire from multiple channels
static OScDev_RichError *ReadImage(OScDev_Device *device, OScDev_Acquisition *acq)
{
	double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
	uint32_t xOffset, yOffset, width, height;
	OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);

	uint32_t elementsPerLine = GetData(device)->lineDelay + width + X_RETRACE_LEN;
	uint32_t scanLines = height;
	int32 elementsPerFramePerChan = elementsPerLine * scanLines;
	size_t nPixels = width * height;

	GetData(device)->oneFrameScanDone = false;
	GetData(device)->framePixelsFilled = 0;

	uint32_t yLen = height + Y_RETRACE_LEN;
	uint32_t estFrameTimeMs = (uint32_t)(1e3 * elementsPerLine * yLen * GetData(device)->binFactor / pixelRateHz);
	uint32_t totalWaitTimeMs = 0;

	OScDev_RichError *err;
	err = StartScan(device);
	if (err)
		return err;

	// Wait for scan to complete
	err = CreateDAQmxError(DAQmxWaitUntilTaskDone(GetData(device)->scannerConfig.aoTask,
		2 * estFrameTimeMs * 1e-3));
	if (err) {
		err = OScDev_Error_Wrap(err, "Failed to wait for scanner task to finish");
		return err;
	}

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

	err = StopScan(device, acq);
	if (err)
		return err;

	// SplitChannels
	// skik if set to scanner only mode
	if (!GetData(device)->scannerOnly)
	{
		bool shouldContinue;
		for (uint32_t ch = 0; ch < GetData(device)->numAIChannels; ++ch)
		{
			shouldContinue = OScDev_Acquisition_CallFrameCallback(acq,
				ch, GetData(device)->frameBuffers[ch]);
			if (!shouldContinue)
			{
				// TODO Stop acquisition
			}
		}
	}

	return OScDev_RichError_OK;
}


static OScDev_RichError *AcquireFrame(OScDev_Device *device, OScDev_Acquisition *acq)
{
	OScDev_RichError *err;
	OScDev_Log_Debug(device, "Reading image...");
	err = ReadImage(device, acq);
	if (err)
		return err;
	OScDev_Log_Debug(device, "Finished reading image");

	return OScDev_RichError_OK;
}


static DWORD WINAPI AcquisitionLoop(void *param)
{
	OScDev_Device *device = (OScDev_Device *)param;
	OScDev_Acquisition *acq = GetData(device)->acquisition.acquisition;

	uint32_t totalFrames = OScDev_Acquisition_GetNumberOfFrames(acq);

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

		OScDev_RichError *err;
		err = AcquireFrame(device, acq);
		if (err)
		{
			err = OScDev_Error_Wrap(err, "Error during sequence acquisition");
			char msg[OScDev_MAX_STR_LEN + 1];
			OScDev_Error_FormatRecursive(err, msg, sizeof(msg));
			OScDev_Log_Error(device, msg);
			break;
		}
	}

	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	GetData(device)->acquisition.running = false;
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	CONDITION_VARIABLE *cv = &(GetData(device)->acquisition.acquisitionFinishCondition);
	WakeAllConditionVariable(cv);

	return 0;
}


OScDev_RichError *RunAcquisitionLoop(OScDev_Device *device)
{
	DWORD id;
	GetData(device)->acquisition.thread =
		CreateThread(NULL, 0, AcquisitionLoop, device, 0, &id);
	return OScDev_RichError_OK;
}


OScDev_RichError *StopAcquisitionAndWait(OScDev_Device *device)
{
	CRITICAL_SECTION *mutex = &GetData(device)->acquisition.mutex;
	CONDITION_VARIABLE *cv = &(GetData(device)->acquisition.acquisitionFinishCondition);

	EnterCriticalSection(mutex);
	if (GetData(device)->acquisition.started) {
		GetData(device)->acquisition.stopRequested = true;
	}
	else { // Armed but not started
		GetData(device)->acquisition.running = false;
	}

	while (GetData(device)->acquisition.running)
	{
		SleepConditionVariableCS(cv, mutex, INFINITE);
	}
	LeaveCriticalSection(mutex);

	return OScDev_RichError_OK;
}


OScDev_RichError *IsAcquisitionRunning(OScDev_Device *device, bool *isRunning)
{
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	*isRunning = GetData(device)->acquisition.running;
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
	return OScDev_RichError_OK;
}


OScDev_RichError *WaitForAcquisitionToFinish(OScDev_Device *device)
{
	CRITICAL_SECTION *mutex = &GetData(device)->acquisition.mutex;
	CONDITION_VARIABLE *cv = &(GetData(device)->acquisition.acquisitionFinishCondition);

	EnterCriticalSection(mutex);
	while (GetData(device)->acquisition.running)
	{
		SleepConditionVariableCS(cv, mutex, INFINITE);
	}
	LeaveCriticalSection(mutex);

	return OScDev_RichError_OK;
}


OScDev_RichError *ReconfigDAQ(OScDev_Device *device, OScDev_Acquisition *acq)
{
	double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
	uint32_t resolution = OScDev_Acquisition_GetResolution(acq);
	double zoomFactor = OScDev_Acquisition_GetZoomFactor(acq);
	uint32_t xOffset, yOffset, width, height;
	OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);
	if (pixelRateHz != GetData(device)->configuredPixelRateHz) {
		GetData(device)->clockConfig.mustReconfigureTiming = true;
		GetData(device)->scannerConfig.mustReconfigureTiming = true;
		GetData(device)->detectorConfig.mustReconfigureTiming = true;
	}
	if (resolution != GetData(device)->configuredResolution) {
		GetData(device)->scannerConfig.mustReconfigureTiming = true;
		GetData(device)->scannerConfig.mustRewriteOutput = true;
	}
	if (zoomFactor != GetData(device)->configuredZoomFactor) {
		GetData(device)->clockConfig.mustRewriteOutput = true;
		GetData(device)->scannerConfig.mustRewriteOutput = true;
	}
	if (xOffset != GetData(device)->configuredXOffset ||
		yOffset != GetData(device)->configuredYOffset) {
		GetData(device)->scannerConfig.mustRewriteOutput = true;
	}
	if (width != GetData(device)->configuredRasterWidth ||
		height != GetData(device)->configuredRasterHeight) {
		GetData(device)->clockConfig.mustReconfigureTiming = true;
		GetData(device)->scannerConfig.mustReconfigureTiming = true;
		GetData(device)->detectorConfig.mustReconfigureTiming = true;
		GetData(device)->clockConfig.mustRewriteOutput = true;
		GetData(device)->scannerConfig.mustRewriteOutput = true;
		GetData(device)->detectorConfig.mustReconfigureCallback = true;
	}

	// Note that additional setting of 'mustReconfigure' flags occurs in settings

	OScDev_RichError *err;

	err = SetUpClock(device, &GetData(device)->clockConfig, acq);
	if (err)
		return err;
	err = SetUpScanner(device, &GetData(device)->scannerConfig, acq);
	if (err)
		return err;
	if (!GetData(device)->scannerOnly)
	{
		err = SetUpDetector(device, &GetData(device)->detectorConfig, acq);
		if (err)
			return err;
	}

	pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
	resolution = OScDev_Acquisition_GetResolution(acq);
	zoomFactor = OScDev_Acquisition_GetZoomFactor(acq);
	OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);
	GetData(device)->configuredPixelRateHz = pixelRateHz;
	GetData(device)->configuredResolution = resolution;
	GetData(device)->configuredZoomFactor = zoomFactor;
	GetData(device)->configuredXOffset = xOffset;
	GetData(device)->configuredYOffset = yOffset;
	GetData(device)->configuredRasterWidth = width;
	GetData(device)->configuredRasterHeight = height;

	return OScDev_RichError_OK;
}