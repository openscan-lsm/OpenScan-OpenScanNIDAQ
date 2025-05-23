#pragma once

#include <OpenScanDeviceLib.h>

OScDev_RichError *ArmAcquisition(OScDev_Device *device,
                                 OScDev_Acquisition *acq, bool scannerOnly);
OScDev_RichError *StartAcquisition(OScDev_Device *device);
OScDev_RichError *StopAcquisitionAndWait(OScDev_Device *device);
OScDev_RichError *IsAcquisitionRunning(OScDev_Device *device, bool *isRunning);
OScDev_RichError *WaitForAcquisitionToFinish(OScDev_Device *device);
