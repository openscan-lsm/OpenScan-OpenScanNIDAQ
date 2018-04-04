/* Generic operation on NI DAQ properties */
/* that are not tied to specific DAQ devices or NIDAQmx functions */

#include "OScNIDAQDevice.h"
#include "OScNIDAQDevicePrivate.h"
#include "OpenScanLibPrivate.h"
#include "OScNIDAQ.h"
#include <NIDAQmx.h>

#include <string.h>

#include <Windows.h>


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
		OSc_Return_If_Error(NIDAQEnumerateInstances(&g_devices, &g_deviceCount));
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
	OSc_Return_If_Error(NIDAQ_PrepareSettings(device));
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