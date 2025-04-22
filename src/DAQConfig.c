#include "DAQConfig.h"

#include "Clock.h"
#include "Detector.h"
#include "DeviceImplData.h"
#include "Scanner.h"
#include "Waveform.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>
#include <ss8str.h>

#include <limits.h>
#include <stdint.h>
#include <stdlib.h>

// Return the index-th physical channel, or empty string if no such channel
static bool GetAIPhysChan(OScDev_Device *device, int index, ss8str *chan) {
    if (index < 0) {
        if (chan)
            ss8_clear(chan);
        return false;
    }

    ss8str chans;
    ss8_init_copy(&chans, &GetImplData(device)->aiPhysChans);

    size_t p = 0;
    bool notFound = false;
    for (int i = 0; i < index; ++i) {
        size_t q = ss8_find_ch(&chans, p, ',');
        if (q == SIZE_MAX) {
            notFound = true;
            break;
        }
        p = q + 1;
    }

    if (chan) {
        if (notFound) {
            ss8_clear(chan);
        } else {
            size_t q = ss8_find_ch(&chans, p, ',');
            ss8_copy_substr(chan, &chans, p, q - p);
            ss8_strip_ch(chan, ' ');
        }
    }

    ss8_destroy(&chans);
    return !notFound;
}

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

void SetWaveformParamsFromDevice(OScDev_Device *device,
                                 struct WaveformParams *parameters,
                                 OScDev_Acquisition *acq) {
    parameters->resolution = OScDev_Acquisition_GetResolution(acq);
    parameters->zoom = OScDev_Acquisition_GetZoomFactor(acq);
    OScDev_Acquisition_GetROI(acq, &parameters->xOffset, &parameters->yOffset,
                              &parameters->width, &parameters->height);
    parameters->undershoot = GetImplData(device)->lineDelay;
    parameters->galvoOffsetX = GetImplData(device)->offsetXY[0];
    parameters->galvoOffsetY = GetImplData(device)->offsetXY[1];
    parameters->xPark = GetImplData(device)->xPark;
    parameters->yPark = GetImplData(device)->yPark;
    parameters->prevXParkVoltage = GetImplData(device)->prevXParkVoltage;
    parameters->prevYParkVoltage = GetImplData(device)->prevYParkVoltage;
}

OScDev_RichError *EnumerateInstances(OScDev_PtrArray **devices,
                                     OScDev_DeviceImpl *impl) {
    OScDev_RichError *err = OScDev_RichError_OK;

    ss8str names; // Comma-separated device names
    ss8_init(&names);
    ss8_set_len(&names, 4096);
    err = CreateDAQmxError(DAQmxGetSysDevNames(ss8_mutable_cstr(&names),
                                               (uInt32)ss8_len(&names)));
    if (err)
        goto fail1;
    ss8_set_len_to_cstrlen(&names);

    *devices = OScDev_PtrArray_Create();

    ss8str name;
    ss8_init(&name);
    size_t p = 0;
    for (;;) {
        size_t q = ss8_find_ch(&names, p, ',');
        ss8_copy_substr(&name, &names, p, q - p);
        ss8_strip_ch(&name, ' ');

        struct DeviceImplData *data = calloc(1, sizeof(struct DeviceImplData));
        InitializeImplData(data);
        ss8_copy(&data->deviceName, &name);

        OScDev_Device *device;
        err = OScDev_Error_AsRichError(
            OScDev_Device_Create(&device, impl, data));
        if (err) {
            ss8str msg;
            ss8_init_copy_cstr(&msg, "Failed to create device ");
            ss8_cat(&msg, &name);
            err = OScDev_Error_Wrap(err, ss8_cstr(&msg));
            ss8_destroy(&msg);
            goto fail2;
        }

        OScDev_PtrArray_Append(*devices, device);

        if (q == SIZE_MAX)
            break;
        p = q + 1;
    }

    goto succeed;

fail2:
    // TODO We have no way to destroy the already-created devices. Leaking.
    OScDev_PtrArray_Destroy(*devices);
succeed:
    ss8_destroy(&name);
fail1:
    ss8_destroy(&names);
    return err;
}

OScDev_RichError *EnumerateAIPhysChans(OScDev_Device *device) {
    ss8str *dest = &GetImplData(device)->aiPhysChans;
    ss8_set_len(dest, 1024);
    ss8_set_front(dest, '\0');
    int32 nierr = DAQmxGetDevAIPhysicalChans(
        ss8_cstr(&GetImplData(device)->deviceName), ss8_mutable_cstr(dest),
        (uInt32)ss8_len(dest));
    ss8_set_len_to_cstrlen(dest);
    ss8_shrink_to_fit(dest);
    if (nierr < 0)
        return CreateDAQmxError(nierr);
    if (ss8_is_empty(dest))
        return OScDev_Error_Create("Device has no AI physical channels");
    return OScDev_RichError_OK;
}

int GetNumberOfEnabledChannels(OScDev_Device *device) {
    int ret = 0;
    for (int i = 0; i < MAX_PHYSICAL_CHANS; ++i) {
        if (GetImplData(device)->channelEnabled[i]) {
            ++ret;
        }
    }
    return ret;
}

void GetEnabledChannels(OScDev_Device *device, ss8str *chans) {
    ss8str chan;
    ss8_init(&chan);

    for (int i = 0; i < MAX_PHYSICAL_CHANS; ++i) {
        if (GetImplData(device)->channelEnabled[i]) {
            GetAIPhysChan(device, i, &chan);
            if (!ss8_is_empty(chans))
                ss8_cat_cstr(chans, ", ");
            ss8_cat(chans, &chan);
        }
    }

    ss8_destroy(&chan);
}

int GetNumberOfAIPhysChans(OScDev_Device *device) {
    for (int i = 0; i < MAX_PHYSICAL_CHANS; ++i) {
        if (!GetAIPhysChan(device, i, NULL))
            return i;
    }
    return MAX_PHYSICAL_CHANS;
}

OScDev_RichError *ReconfigDAQ(OScDev_Device *device) {
    OScDev_Acquisition *acq = GetImplData(device)->acquisition.acquisition;
    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    uint32_t resolution = OScDev_Acquisition_GetResolution(acq);
    double zoomFactor = OScDev_Acquisition_GetZoomFactor(acq);
    uint32_t xOffset, yOffset, width, height;
    OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);
    if (pixelRateHz != GetImplData(device)->configuredPixelRateHz) {
        GetImplData(device)->clockConfig.mustReconfigureTiming = true;
        GetImplData(device)->scannerConfig.mustReconfigureTiming = true;
        GetImplData(device)->detectorConfig.mustReconfigureTiming = true;
    }
    if (resolution != GetImplData(device)->configuredResolution) {
        GetImplData(device)->scannerConfig.mustReconfigureTiming = true;
        GetImplData(device)->scannerConfig.mustRewriteOutput = true;
    }
    if (zoomFactor != GetImplData(device)->configuredZoomFactor) {
        GetImplData(device)->clockConfig.mustRewriteOutput = true;
        GetImplData(device)->scannerConfig.mustRewriteOutput = true;
    }
    if (xOffset != GetImplData(device)->configuredXOffset ||
        yOffset != GetImplData(device)->configuredYOffset) {
        GetImplData(device)->scannerConfig.mustRewriteOutput = true;
    }
    if (width != GetImplData(device)->configuredRasterWidth ||
        height != GetImplData(device)->configuredRasterHeight) {
        GetImplData(device)->clockConfig.mustReconfigureTiming = true;
        GetImplData(device)->scannerConfig.mustReconfigureTiming = true;
        GetImplData(device)->detectorConfig.mustReconfigureTiming = true;
        GetImplData(device)->clockConfig.mustRewriteOutput = true;
        GetImplData(device)->scannerConfig.mustRewriteOutput = true;
        GetImplData(device)->detectorConfig.mustReconfigureCallback = true;
    }

    // Note that additional setting of 'mustReconfigure' flags occurs in
    // settings

    OScDev_RichError *err;

    err = SetUpClock(device, &GetImplData(device)->clockConfig, acq);
    if (err)
        return err;
    if (!GetImplData(device)->scannerOnly) {
        err = SetUpDetector(device, &GetImplData(device)->detectorConfig, acq);
        if (err)
            return err;
    }

    pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    resolution = OScDev_Acquisition_GetResolution(acq);
    zoomFactor = OScDev_Acquisition_GetZoomFactor(acq);
    OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);
    GetImplData(device)->configuredPixelRateHz = pixelRateHz;
    GetImplData(device)->configuredResolution = resolution;
    GetImplData(device)->configuredZoomFactor = zoomFactor;
    GetImplData(device)->configuredXOffset = xOffset;
    GetImplData(device)->configuredYOffset = yOffset;
    GetImplData(device)->configuredRasterWidth = width;
    GetImplData(device)->configuredRasterHeight = height;

    return OScDev_RichError_OK;
}
