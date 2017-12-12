#pragma once

#include "OScNIDAQDevicePrivate.h"

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