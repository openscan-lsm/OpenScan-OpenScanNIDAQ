/* Generic operation on NI DAQ properties */
/* that are not tied to specific DAQ devices or NIDAQmx functions */

#include "OScNIDAQDevicePrivate.h"
#include "OScNIDAQ.h"
#include <NIDAQmx.h>

#include <string.h>

#include <Windows.h>

// Forward declaration
static OScDev_DeviceImpl DeviceImpl;


static OScDev_Error NIDAQGetModelName(const char **name)
{
	*name = "OpenScan-NIDAQ";
	return OScDev_OK;
}


static OScDev_Error NIDAQEnumerateInstances(OScDev_PtrArray **devices)
{
	OScDev_RichError* err = EnumerateInstances(devices, &DeviceImpl);
	return OScDev_Error_ReturnAsCode(err);
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
		OScDev_RichError* err = CreateDAQmxError(nierr);

		char msg[OScDev_MAX_STR_LEN + 1] = "Cannot reset NI DAQ card ";
		strcat(msg, GetData(device)->deviceName);
		return OScDev_Error_ReturnAsCode(OScDev_Error_Wrap(err, msg));
	}

	return OScDev_Error_ReturnAsCode(OpenDAQ(device));
}


static OScDev_Error NIDAQClose(OScDev_Device *device)
{
	StopAcquisitionAndWait(device);
	return OScDev_Error_ReturnAsCode(CloseDAQ(device));
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


static OScDev_Error NIDAQGetPixelRates(OScDev_Device *device, OScDev_NumRange **pixelRatesHz)
{
	static const double ratesMHz[] = {
		0.0500,
		0.1000,
		0.1250,
		0.2000,
		0.2500,
		0.4000,
		0.5000,
		0.6250,
		1.0000,
		1.2500,
		0.0 // End mark
	};
	*pixelRatesHz = OScDev_NumRange_CreateDiscrete();
	for (size_t i = 0; ratesMHz[i] != 0.0; ++i) {
		OScDev_NumRange_AppendDiscrete(*pixelRatesHz, 1e6 * ratesMHz[i]);
	}
	return OScDev_OK;
}


static OScDev_Error NIDAQGetResolutions(OScDev_Device *device, OScDev_NumRange **resolutions)
{
	*resolutions = OScDev_NumRange_CreateDiscrete();
	OScDev_NumRange_AppendDiscrete(*resolutions, 256);
	OScDev_NumRange_AppendDiscrete(*resolutions, 512);
	OScDev_NumRange_AppendDiscrete(*resolutions, 1024);
	OScDev_NumRange_AppendDiscrete(*resolutions, 2048);
	return OScDev_OK;
}


static OScDev_Error NIDAQGetZoomFactors(OScDev_Device *device, OScDev_NumRange **zooms)
{
	*zooms = OScDev_NumRange_CreateContinuous(0.2, 20.0);
	return OScDev_OK;
}


static OScDev_Error NIDAQIsROIScanSupported(OScDev_Device *device, bool *supported)
{
	*supported = true;
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


static OScDev_Error NIDAQArm(OScDev_Device *device, OScDev_Acquisition *acq)
{
	bool useClock, useScanner, useDetector;
	OScDev_Acquisition_IsClockRequested(acq, &useClock);
	OScDev_Acquisition_IsScannerRequested(acq, &useScanner);
	OScDev_Acquisition_IsDetectorRequested(acq, &useDetector);

	// assume scanner is always enabled
	if (!useClock || !useScanner)
		return OScDev_Error_ReturnAsCode(OScDev_Error_Create("Unsupported Operation"));

	OScDev_TriggerSource clockStartTriggerSource;
	OScDev_Acquisition_GetClockStartTriggerSource(acq, &clockStartTriggerSource);
	if (clockStartTriggerSource != OScDev_TriggerSource_Software)
		return OScDev_Error_ReturnAsCode(OScDev_Error_Create("Unsupported Operation"));

	OScDev_ClockSource clockSource;
	OScDev_Acquisition_GetClockSource(acq, &clockSource);
	if (clockSource != OScDev_ClockSource_Internal)
		return OScDev_Error_ReturnAsCode(OScDev_Error_Create("Unsupported Operation"));
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

	OScDev_RichError *err;
	CRITICAL_SECTION *mutex = &GetData(device)->acquisition.mutex;
	EnterCriticalSection(mutex);
	{
		if (GetData(device)->acquisition.running)
		{
			// TODO Error should be "already armed"
			LeaveCriticalSection(mutex);
			return OScDev_Error_ReturnAsCode(OScDev_Error_Create("Device already armed"));
		}

		GetData(device)->acquisition.acquisition = acq;

		GetData(device)->acquisition.stopRequested = false;
		GetData(device)->acquisition.running = true;
		GetData(device)->acquisition.armed = false;
		GetData(device)->acquisition.started = false;
	}
	LeaveCriticalSection(mutex);

	err = ReconfigDAQ(device, acq);
	if (err)
		goto error;

	EnterCriticalSection(&(GetData(device)->acquisition.mutex));
	{
		GetData(device)->acquisition.armed = true;
	}
	LeaveCriticalSection(&(GetData(device)->acquisition.mutex));

	return OScDev_OK;

error:
	EnterCriticalSection(mutex);
	{
		GetData(device)->acquisition.running = false;
		GetData(device)->acquisition.acquisition = NULL;
	}
	LeaveCriticalSection(mutex);
	return OScDev_Error_ReturnAsCode(err);
}


static OScDev_Error NIDAQStart(OScDev_Device *device)
{
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

	return OScDev_Error_ReturnAsCode(RunAcquisitionLoop(device));
}


static OScDev_Error NIDAQStop(OScDev_Device *device)
{
	return OScDev_Error_ReturnAsCode(StopAcquisitionAndWait(device));
}


static OScDev_Error NIDAQIsRunning(OScDev_Device *device, bool *isRunning)
{
	return OScDev_Error_ReturnAsCode(IsAcquisitionRunning(device, isRunning));
}


static OScDev_Error NIDAQWait(OScDev_Device *device)
{
	return OScDev_Error_ReturnAsCode(WaitForAcquisitionToFinish(device));
}


static OScDev_DeviceImpl DeviceImpl = {
	.GetModelName = NIDAQGetModelName,
	.EnumerateInstances = NIDAQEnumerateInstances,
	.ReleaseInstance = NIDAQReleaseInstance,
	.GetName = NIDAQGetName,
	.Open = NIDAQOpen,
	.Close = NIDAQClose,
	.HasClock = NIDAQHasClock,
	.HasScanner = NIDAQHasScanner,
	.HasDetector = NIDAQHasDetector,
	.MakeSettings = NIDAQMakeSettings,
	.GetPixelRates = NIDAQGetPixelRates,
	.GetResolutions = NIDAQGetResolutions,
	.GetZoomFactors = NIDAQGetZoomFactors,
	.IsROIScanSupported = NIDAQIsROIScanSupported,
	.GetNumberOfChannels = NIDAQGetNumberOfChannels,
	.GetBytesPerSample = NIDAQGetBytesPerSample,
	.Arm = NIDAQArm,
	.Start = NIDAQStart,
	.Stop = NIDAQStop,
	.IsRunning = NIDAQIsRunning,
	.Wait = NIDAQWait,
};


static OScDev_Error GetDeviceImpls(OScDev_PtrArray **impls)
{
	*impls = OScDev_PtrArray_Create();
	OScDev_PtrArray_Append(*impls, &DeviceImpl);
	return OScDev_OK;
}


OScDev_MODULE_IMPL = {
	.displayName = "OpenScan NI-DAQ",
	.GetDeviceImpls = GetDeviceImpls,
	.supportsRichErrors = true,
};