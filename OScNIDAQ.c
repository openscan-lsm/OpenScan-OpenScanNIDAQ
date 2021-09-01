
#include "OScNIDAQ.h"
#include "Waveform.h"

#include <Windows.h>

#include <math.h>
#include <stdio.h>
#include <string.h>


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


// Fill in non-zero defaults only
static void InitializePrivateData(struct OScNIDAQPrivateData *data)
{
	data->lineDelay = 50;
	data->numLinesToBuffer = 8;
	data->inputVoltageRange = 10.0;
	data->minVolts_ = -10.0;
	data->maxVolts_ = 10.0;

	data->channelEnabled[0] = true;
	
	InitializeCriticalSection(&(data->acquisition.mutex));
	InitializeConditionVariable(&(data->acquisition.acquisitionFinishCondition));
}


OScDev_RichError *EnumerateInstances(OScDev_PtrArray **devices, OScDev_DeviceImpl *impl)
{
	OScDev_RichError *err;

	// get a comma-delimited list of all of the devices installed in the system
	char deviceNames[4096];
	err = CreateDAQmxError(DAQmxGetSysDevNames(deviceNames, sizeof(deviceNames)));
	if (err)
		return err;

	char deviceList[MAX_NUM_DEVICES][OScDev_MAX_STR_LEN + 1];

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

		InitializePrivateData(GetData(device));

		OScDev_PtrArray_Append(*devices, device);
	}
	
	return OScDev_RichError_OK;
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


OScDev_RichError *EnumerateAIPhysChans(OScDev_Device *device)
{
	char *buf = malloc(1024);
	int32 nierr = DAQmxGetDevAIPhysicalChans(GetData(device)->deviceName, buf, 1024);
	if (nierr < 0)
		return CreateDAQmxError(nierr);
	if (strlen(buf) == 0)
		return OScDev_Error_Create("Device has no AI physical channels");

	buf = realloc(buf, strlen(buf) + 1); // Shrink-wrap
	GetData(device)->aiPhysChans = buf;
	return OScDev_RichError_OK;
}


int GetNumberOfEnabledChannels(OScDev_Device *device)
{
	int ret = 0;
	for (int i = 0; i < MAX_PHYSICAL_CHANS; ++i) {
		if (GetData(device)->channelEnabled[i]) {
			++ret;
		}
	}
	return ret;
}


void GetEnabledChannels(OScDev_Device *device, char *buf, size_t bufsiz)
{
	if (bufsiz == 0)
		return;
	buf[0] = '\0';
	char *p = buf;
	char *bufend = buf + bufsiz;

	for (int i = 0; i < MAX_PHYSICAL_CHANS; ++i) {
		if (GetData(device)->channelEnabled[i]) {
			char chan[64];
			GetAIPhysChan(device, i, chan, sizeof(chan));
			if (p != buf)
				p += snprintf(p, bufend - p, "%s", ", ");
			p += snprintf(p, bufend - p, "%s", chan);
		}
	}
}


int GetNumberOfAIPhysChans(OScDev_Device *device)
{
	for (int i = 0; i < MAX_PHYSICAL_CHANS; ++i) {
		char buf[64];
		GetAIPhysChan(device, i, buf, sizeof(buf));
		if (strlen(buf) == 0) {
			return i;
		}
	}
	return MAX_PHYSICAL_CHANS;
}


// Return the index-th physical channel, or empty string if no such channel
void GetAIPhysChan(OScDev_Device *device, int index, char *buf, size_t bufsiz)
{
	if (bufsiz == 0)
		return;
	buf[0] = '\0';

	// Make p point to start of the channel at desired index
	char *p = GetData(device)->aiPhysChans;
	for (int i = 0; i < index; ++i) {
		char *pp = strchr(p, ',');
		if (!pp)
			return;
		while (*++pp == ' ')
			;
		p = pp;
	}

	// Now make q point to beyond the end of the channel
	char *q = strchr(p, ',');
	if (!q)
		q = strchr(p, '\0');

	size_t siz = q - p + 1;
	if (siz > bufsiz)
		siz = bufsiz;
	snprintf(buf, siz, "%s", p);
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
		if (count < MAX_NUM_DEVICES)
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


OScDev_RichError *OpenDAQ(OScDev_Device *device)
{
	OScDev_Log_Debug(device, "Start initializing DAQ");

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
	uint32_t yRetraceTime = (uint32_t)(1e3 * xLen * Y_RETRACE_LEN / pixelRateHz);
	uint32_t estFrameTime = (uint32_t)(1e3 * xLen * yLen / pixelRateHz);
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
	uint32_t estFrameTimeMs = (uint32_t)(1e3 * elementsPerLine * yLen / pixelRateHz);
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

	if (!GetData(device)->scannerOnly)
	{
		int nChans = GetNumberOfEnabledChannels(device);
		for (int ch = 0; ch < nChans; ++ch)
		{
			bool shouldContinue = OScDev_Acquisition_CallFrameCallback(acq,
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