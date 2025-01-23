#pragma once

#include <OpenScanDeviceLib.h>

OScDev_Error NIDAQMakeSettings(OScDev_Device *device,
                               OScDev_PtrArray **settings);
