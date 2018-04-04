#pragma once

#include "OScNIDAQDevicePrivate.h"

#include "OpenScanDeviceImpl.h"

struct OSc_Device_Impl OpenScan_NIDAQ_Device_Impl;

OSc_Error EnumerateInstances(OSc_Device ***devices, size_t *deviceCount);
OSc_Error OpenDAQ(OSc_Device *device);
OSc_Error CloseDAQ(OSc_Device *device);
OSc_Error StartDAQ(OSc_Device *device);
OSc_Error SetScanParameters(OSc_Device *device);
OSc_Error ReloadWaveform(OSc_Device *device);
OSc_Error RunAcquisitionLoop(OSc_Device *device, OSc_Acquisition *acq);
OSc_Error StopAcquisitionAndWait(OSc_Device *device, OSc_Acquisition *acq);
OSc_Error IsAcquisitionRunning(OSc_Device *device, bool *isRunning);
OSc_Error WaitForAcquisitionToFinish(OSc_Device *device);
OSc_Error GetTerminalNameWithDevPrefix(TaskHandle taskHandle, const char terminalName[], char triggerName[]);
OSc_Error SetTriggers(OSc_Device *device);
OSc_Error ReconfigTiming(OSc_Device *device);
OSc_Error UnregisterLineAcqEvent(OSc_Device *device);
OSc_Error RegisterLineAcqEvent(OSc_Device *device);
OSc_Error CommitTasks(OSc_Device *device);


