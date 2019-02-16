/* Generic operation on NI DAQ properties */
/* that are not tied to specific DAQ devices or NIDAQmx functions */

#include "OScNIDAQDevice.h"
#include "OScNIDAQDevicePrivate.h"
#include "OScNIDAQ.h"
#include <NIDAQmx.h>

#include <string.h>

#include <Windows.h>


static OScDev_Device **g_devices;
static size_t g_deviceCount;

static OScDev_Error NIDAQGetModelName(const char **name)
{
	*name = "OpenScan-NIDAQ";
	return OScDev_OK;
}

static OScDev_Error NIDAQGetInstances(OScDev_Device ***devices, size_t *count)
{
	OScDev_Error err;
	if (!g_devices && OScDev_CHECK(err, NIDAQEnumerateInstances(&g_devices, &g_deviceCount)))
		return err;
	*devices = g_devices;
	*count = g_deviceCount;
	return OScDev_OK;
}


static OScDev_Error NIDAQReleaseInstance(OScDev_Device *device)
{
	free(GetData(device));
	return OScDev_OK;
}


static OScDev_Error NIDAQGetName(OScDev_Device *device, char *name)
{
	strncpy(name, GetData(device)->deviceName, OScDev_MAX_STR_LEN);
	return OScDev_OK;
}



static OScDev_Error NIDAQOpen(OScDev_Device *device)
{
	int32 nierr = DAQmxResetDevice(GetData(device)->deviceName); // TODO wrong function
	if (nierr)
	{
		char msg[OScDev_MAX_STR_LEN + 1] = "Cannot reset NI DAQ card ";
		strcat(msg, GetData(device)->deviceName);
		OScDev_Log_Error(device, msg);
		return OScDev_Error_Unknown; // TODO Detailed info
	}

	return OpenDAQ(device);
}


static OScDev_Error NIDAQClose(OScDev_Device *device)
{
	StopAcquisitionAndWait(device);
	OScDev_Error err = CloseDAQ(device);
	return err;
}


static OScDev_Error NIDAQHasClock(OScDev_Device *device, bool *hasClock)
{
	*hasClock = true;
	return OScDev_OK;
}


static OScDev_Error NIDAQHasScanner(OScDev_Device *device, bool *hasScanner)
{
	*hasScanner = true;
	return OScDev_OK;
}


static OScDev_Error NIDAQHasDetector(OScDev_Device *device, bool *hasDetector)
{
	*hasDetector = true;
	return OScDev_OK;
}


static OScDev_Error NIDAQGetSettings(OScDev_Device *device, OScDev_Setting ***settings, size_t *count)
{
	OScDev_Error err;
	if OScDev_CHECK(err, NIDAQ_PrepareSettings(device))
		return err;
	*settings = GetData(device)->settings;
	*count = GetData(device)->settingCount;
	return OScDev_OK;

}

static OScDev_Error NIDAQGetAllowedResolutions(OScDev_Device *device, size_t **widths, size_t **heights, size_t *count)
{
	static size_t resolutions[] = { 256, 512, 1024, 2048 };
	*widths = *heights = resolutions;
	*count = sizeof(resolutions) / sizeof(size_t);
	return OScDev_OK;
}

static OScDev_Error NIDAQGetResolution(OScDev_Device *device, size_t *width, size_t *height)
{
	*width = *height = GetData(device)->resolution;
	return OScDev_OK;
}


static OScDev_Error NIDAQSetResolution(OScDev_Device *device, size_t width, size_t height)
{
	if (width == GetData(device)->resolution)
		return OScDev_OK;
	GetData(device)->resolution = (uint32_t)width;

	GetData(device)->clockConfig.mustReconfigureTiming = true;
	GetData(device)->scannerConfig.mustReconfigureTiming = true;
	GetData(device)->detectorConfig.mustReconfigureTiming = true;
	GetData(device)->clockConfig.mustRewriteOutput = true;
	GetData(device)->scannerConfig.mustRewriteOutput = true;
	GetData(device)->detectorConfig.mustReconfigureCallback = true;

	// reflect the change to magnification as well
	GetData(device)->magnification =
		(double)width / OSc_DEFAULT_RESOLUTION * GetData(device)->zoom / OSc_DEFAULT_ZOOM;

	return OScDev_OK;
}


static OScDev_Error NIDAQGetMagnification(OScDev_Device *device, double *magnification)
{
	*magnification = GetData(device)->magnification;
	return OScDev_OK;
}

// probably incorrect and no use as it doesn't reflect the change of resolution or zoom
static OScDev_Error NIDAQSetMagnification(OScDev_Device *device)
{
	size_t resolution = GetData(device)->resolution;
	double zoom = GetData(device)->zoom;
	GetData(device)->magnification = 
		(double)(resolution / OSc_DEFAULT_RESOLUTION) * (zoom / OSc_DEFAULT_ZOOM);
	return OScDev_OK;
}


static OScDev_Error NIDAQGetImageSize(OScDev_Device *device, uint32_t *width, uint32_t *height)
{
	*width = GetData(device)->resolution;
	*height = GetData(device)->resolution;
	return OScDev_OK;
}

// Same as OpenScanDAQ::GetNumberOfChannels()
static OScDev_Error NIDAQGetNumberOfChannels(OScDev_Device *device, uint32_t *nChannels)
{
	*nChannels = (GetData(device)->channels) == CHANNELS1_2_3 ? 3 :
		((GetData(device)->channels) == CHANNELS_1_AND_2 || (GetData(device)->channels) == CHANNELS_1_AND_3) ? 2 : 1;
	return OScDev_OK;
}

static OScDev_Error NIDAQGetBytesPerSample(OScDev_Device *device, uint32_t *bytesPerSample)
{
	*bytesPerSample = 2;
	return OScDev_OK;
}


// equal to SequenceThread::Start()
static OScDev_Error NIDAQArm(OScDev_Device *device, OScDev_Acquisition *acq)
{
	bool useClock, useScanner, useDetector;
	OScDev_Acquisition_IsClockRequested(acq, &useClock);
	OScDev_Acquisition_IsScannerRequested(acq, &useScanner);
	OScDev_Acquisition_IsDetectorRequested(acq, &useDetector);

	// assume scanner is always enabled
	if (!useClock || !useScanner)
		return OScDev_Error_Unsupported_Operation;

	enum OScDev_TriggerSource clockStartTriggerSource;
	OScDev_Acquisition_GetClockStartTriggerSource(acq, &clockStartTriggerSource);
	if (clockStartTriggerSource != OScDev_TriggerSource_Software)
		return OScDev_Error_Unsupported_Operation;

	enum OScDev_ClockSource clockSource;
	OScDev_Acquisition_GetClockSource(acq, &clockSource);
	if (clockSource != OScDev_ClockSource_Internal)
		return OScDev_Error_Unsupported_Operation;
	// what if we use external line clock to trigger acquisition?

	if (useDetector)
	{
		// arm scanner, detector, and clock
		GetData(device)->scannerOnly = false;
	}
	else
	{
		// arm scanner and clock
		GetData(device)->scannerOnly = true;
	}

	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (GetData(device)->acquisition.running &&
			GetData(device)->acquisition.armed)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			if (GetData(device)->acquisition.started)
				return OScDev_Error_Acquisition_Running;
			else
				return OScDev_OK;
		}

		GetData(device)->acquisition.acquisition = acq;

		GetData(device)->acquisition.stopRequested = false;
		GetData(device)->acquisition.running = true;
		GetData(device)->acquisition.armed = false;
		GetData(device)->acquisition.started = false;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	OScDev_Error err;
	if (OScDev_CHECK(err, ReconfigDAQ(device)))
		return err;

	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		GetData(device)->acquisition.armed = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	return OScDev_OK;
}


static OScDev_Error NIDAQStart(OScDev_Device *device)
{
	// start scanner
	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		if (!GetData(device)->acquisition.running ||
			!GetData(device)->acquisition.armed)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			return OScDev_Error_Not_Armed;
		}
		if (GetData(device)->acquisition.started)
		{
			LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
			return OScDev_Error_Acquisition_Running;
		}

		GetData(device)->acquisition.started = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	// We don't yet support running detector as trigger source

	return RunAcquisitionLoop(device);

}


static OScDev_Error NIDAQStop(OScDev_Device *device)
{
	return StopAcquisitionAndWait(device);
}


static OScDev_Error NIDAQIsRunning(OScDev_Device *device, bool *isRunning)
{
	return IsAcquisitionRunning(device, isRunning);
}


static OScDev_Error NIDAQWait(OScDev_Device *device)
{
	return WaitForAcquisitionToFinish(device);
}


struct OScDev_DeviceImpl OpenScan_NIDAQ_Device_Impl = {
	.GetModelName = NIDAQGetModelName,
	.GetInstances = NIDAQGetInstances,
	.ReleaseInstance = NIDAQReleaseInstance,
	.GetName = NIDAQGetName,
	.Open = NIDAQOpen,
	.Close = NIDAQClose,
	.HasClock = NIDAQHasClock,
	.HasScanner = NIDAQHasScanner,
	.HasDetector = NIDAQHasDetector,
	.GetSettings = NIDAQGetSettings,
	.GetAllowedResolutions = NIDAQGetAllowedResolutions,
	.GetResolution = NIDAQGetResolution,
	.SetResolution = NIDAQSetResolution,
	.GetMagnification = NIDAQGetMagnification,
	.SetMagnification = NIDAQSetMagnification,
	.GetImageSize = NIDAQGetImageSize,
	.GetNumberOfChannels = NIDAQGetNumberOfChannels,
	.GetBytesPerSample = NIDAQGetBytesPerSample,
	.Arm = NIDAQArm,
	.Start = NIDAQStart,
	.Stop = NIDAQStop,
	.IsRunning = NIDAQIsRunning,
	.Wait = NIDAQWait,
};


static OScDev_Error GetDeviceImpls(struct OScDev_DeviceImpl **impls, size_t *implCount)
{
	if (*implCount < 1)
		return OScDev_OK;

	impls[0] = &OpenScan_NIDAQ_Device_Impl;
	*implCount = 1;
	return OScDev_OK;
}


OScDev_MODULE_IMPL = {
	.displayName = "OpenScan NI-DAQ",
	.GetDeviceImpls = GetDeviceImpls,
};