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


OSc_Error EnumerateInstances(OSc_Device ***devices, size_t *deviceCount)
{
	OSc_Return_If_Error(EnsureNIDAQInitialized());

	// The first FPGA board on the system always has the RIO Resource Name
	// "RIO0" (as far as I know). For now, only support this one.

	struct OScNIDAQPrivateData *data = calloc(1, sizeof(struct OScNIDAQPrivateData));
	strncpy(data->rioResourceName, "RIO0", OSc_MAX_STR_LEN);

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
	OSc_Return_If_Error(EnsureNIDAQInitialized());

	//TODO
	
	++g_openDeviceCount;

	return OSc_Error_OK;
}

// same to Shutdown() in old OpenScan format
OSc_Error CloseDAQ(OSc_Device *device)
{
	//TODO

	--g_openDeviceCount;
	if (g_openDeviceCount == 0)
		OSc_Return_If_Error(DeinitializeNiFpga());

	return OSc_Error_OK;
}


OSc_Error StartDAQ(OSc_Device *device)
{
	return OSc_Error_OK;
}


static OSc_Error SendParameters(OSc_Device *device)
{
	OSc_Return_If_Error(StartDAQ(device));

		return OSc_Error_OK;
}


static OSc_Error SetScanRate(OSc_Device *device, double scanRate)
{
	return OSc_Error_OK;
}


static OSc_Error SetResolution(OSc_Device *device, uint32_t resolution)
{
	return OSc_Error_OK;
}


static OSc_Error InitScan(OSc_Device *device)
{
	return OSc_Error_OK;
}


OSc_Error SetScanParameters(OSc_Device *device)
{
	return OSc_Error_OK;
}


static OSc_Error WriteWaveforms(OSc_Device *device, uint16_t *firstX, uint16_t *firstY)
{
	return OSc_Error_OK;
}


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


static OSc_Error StartScan(OSc_Device *device)
{
	return OSc_Error_OK;
}


static OSc_Error StopScan(OSc_Device *device)
{
	return OSc_Error_OK;
}


static OSc_Error ReadImage(OSc_Device *device, OSc_Acquisition *acq, bool discard)
{
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