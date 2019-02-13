#pragma once

#include "OScNIDAQDevicePrivate.h"

#define NUM_SLOTS_IN_CHASSIS 4
#define MAX_NUM_PORTS 256


OScDev_Error NIDAQEnumerateInstances(OScDev_Device ***devices, size_t *deviceCount);
OScDev_Error ParseDeviceNameList(char *names,
	char(*deviceNames)[OScDev_MAX_STR_LEN + 1], size_t *deviceCount);
OScDev_Error OpenDAQ(OScDev_Device *device);
OScDev_Error CloseDAQ(OScDev_Device *device);
OScDev_Error InitDAQ(OScDev_Device *device);
OScDev_Error GetTerminalNameWithDevPrefix(TaskHandle taskHandle, const char terminalName[], char triggerName[]);
OScDev_Error WriteWaveforms(OScDev_Device *device);
OScDev_Error ReconfigTiming(OScDev_Device *device);
OScDev_Error UnregisterLineAcqEvent(OScDev_Device *device);
OScDev_Error RegisterLineAcqEvent(OScDev_Device *device);
OScDev_Error CommitTasks(OScDev_Device *device);
OScDev_Error ReadLineCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData);
OScDev_Error SplitChannels(OScDev_Device *device);
OScDev_Error ReconfigDAQ(OScDev_Device *device);
OScDev_Error ReconfigAIVoltageChannels(OScDev_Device* device);
OScDev_Error RunAcquisitionLoop(OScDev_Device *device);
OScDev_Error StopAcquisitionAndWait(OScDev_Device *device);
OScDev_Error IsAcquisitionRunning(OScDev_Device *device, bool *isRunning);
OScDev_Error WaitForAcquisitionToFinish(OScDev_Device *device);
