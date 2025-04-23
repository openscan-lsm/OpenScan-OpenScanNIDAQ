#include "OpenScanDevice.h"

#include <OpenScanDeviceLib.h>

OScDev_Error NIDAQGetDeviceImpls(OScDev_PtrArray **impls) {
    *impls = OScDev_PtrArray_Create();
    OScDev_PtrArray_Append(*impls, &NIDAQDeviceImpl);
    return OScDev_OK;
}

OScDev_MODULE_IMPL = {
    .displayName = "OpenScan NI-DAQ",
    .GetDeviceImpls = NIDAQGetDeviceImpls,
    .supportsRichErrors = true,
};
