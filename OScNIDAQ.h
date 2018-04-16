#pragma once

#include "OScNIDAQDevicePrivate.h"

#define NUM_SLOTS_IN_CHASSIS 4

OSc_Error NIDAQEnumerateInstances(OSc_Device ***devices, size_t *deviceCount);
OSc_Error ParseDeviceNameList(char * names, 
	char deviceNames[NUM_SLOTS_IN_CHASSIS][OSc_MAX_STR_LEN + 1], size_t * deviceCount);
OSc_Error OpenDAQ(OSc_Device *device);
OSc_Error CloseDAQ(OSc_Device *device);
OSc_Error InitDAQ(OSc_Device *device);
OSc_Error GetTerminalNameWithDevPrefix(TaskHandle taskHandle, const char terminalName[], char triggerName[]);
OSc_Error WriteWaveforms(OSc_Device *device);
OSc_Error SetTriggers(OSc_Device *device);
OSc_Error ReconfigTiming(OSc_Device *device);
OSc_Error UnregisterLineAcqEvent(OSc_Device *device);
OSc_Error RegisterLineAcqEvent(OSc_Device *device);
OSc_Error CommitTasks(OSc_Device *device);
OSc_Error ReadLineCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void* callbackData);
OSc_Error SplitChannels(OSc_Device *device);
OSc_Error ReconfigDAQ(OSc_Device *device);
// TODO: naming conflicts with OScNIFPGA
OSc_Error RunAcquisitionLoop(OSc_Device *device, OSc_Acquisition *acq);
OSc_Error StopAcquisitionAndWait(OSc_Device *device, OSc_Acquisition *acq);
OSc_Error IsAcquisitionRunning(OSc_Device *device, bool *isRunning);
OSc_Error WaitForAcquisitionToFinish(OSc_Device *device);
