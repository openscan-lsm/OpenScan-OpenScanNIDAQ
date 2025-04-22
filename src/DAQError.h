#pragma once

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>

// Must be called immediately after failed DAQmx function
OScDev_RichError *CreateDAQmxError(int32 nierr);
