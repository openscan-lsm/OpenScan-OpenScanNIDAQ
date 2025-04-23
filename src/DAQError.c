#include "DAQError.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>

#include <stddef.h>

static char *ErrorCodeDomain() {
    static char *domainName = NULL;
    if (domainName == NULL) {
        domainName = "NI DAQmx";
        OScDev_Error_RegisterCodeDomain(domainName,
                                        OScDev_ErrorCodeFormat_I32);
    }
    return domainName;
}

// Must be called immediately after failed DAQmx function
OScDev_RichError *CreateDAQmxError(int32 nierr) {
    if (nierr == 0)
        return OScDev_RichError_OK;

    char buf[1024];
    DAQmxGetExtendedErrorInfo(buf, sizeof(buf));

    if (nierr > 0) {
        OScDev_Log_Warning(NULL, buf);
        return OScDev_RichError_OK;
    }

    return OScDev_Error_CreateWithCode(ErrorCodeDomain(), nierr, buf);
}
