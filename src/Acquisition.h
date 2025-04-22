#pragma once

#include <OpenScanDeviceLib.h>

OScDev_RichError *StartAcquisitionLoop(OScDev_Device *device);
OScDev_RichError *StopAcquisitionAndWait(OScDev_Device *device);
OScDev_RichError *IsAcquisitionRunning(OScDev_Device *device, bool *isRunning);
OScDev_RichError *WaitForAcquisitionToFinish(OScDev_Device *device);
