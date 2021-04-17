#pragma once

#include "OScNIDAQDevicePrivate.h"

#define NUM_SLOTS_IN_CHASSIS 4
#define MAX_NUM_PORTS 256


OScDev_Error NIDAQEnumerateInstances(OScDev_PtrArray **devices);
OScDev_RichError *ParseDeviceNameList(char *names,
	char(*deviceNames)[OScDev_MAX_STR_LEN + 1], size_t *deviceCount);
OScDev_RichError *OpenDAQ(OScDev_Device *device);
OScDev_RichError *CloseDAQ(OScDev_Device *device);
OScDev_RichError *GetTerminalNameWithDevPrefix(TaskHandle taskHandle, const char terminalName[], char triggerName[]);
OScDev_RichError *ReconfigDAQ(OScDev_Device *device, OScDev_Acquisition *acq);
OScDev_RichError *RunAcquisitionLoop(OScDev_Device *device);
OScDev_RichError *StopAcquisitionAndWait(OScDev_Device *device);
OScDev_RichError *IsAcquisitionRunning(OScDev_Device *device, bool *isRunning);
OScDev_RichError *WaitForAcquisitionToFinish(OScDev_Device *device);
