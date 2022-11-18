#pragma once

#include "OScNIDAQDevicePrivate.h"

OScDev_Error NIDAQEnumerateInstances(OScDev_PtrArray **devices);
OScDev_RichError *OpenDAQ(OScDev_Device *device);
OScDev_RichError *CloseDAQ(OScDev_Device *device);
OScDev_RichError *ReconfigDAQ(OScDev_Device *device, OScDev_Acquisition *acq);
OScDev_RichError *RunAcquisitionLoop(OScDev_Device *device);
OScDev_RichError *StopAcquisitionAndWait(OScDev_Device *device);
OScDev_RichError *IsAcquisitionRunning(OScDev_Device *device, bool *isRunning);
OScDev_RichError *WaitForAcquisitionToFinish(OScDev_Device *device);
