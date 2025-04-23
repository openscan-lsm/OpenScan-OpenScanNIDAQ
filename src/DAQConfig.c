#include "DAQConfig.h"

#include "Clock.h"
#include "DAQError.h"
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
