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
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

static const double MIN_AO_RATE_HZ = 100.0e3;    // keep galvo waveform smooth
static const double TARGET_AO_RATE_HZ = 200.0e3; // galvo BW ~kHz; higher
                                                 // wastes memory/CPU

// Choose aoRate = timebaseHz / t, where t (AO period in ticks) divides the
// pixel period in ticks. Among candidates within [MIN_AO_RATE_HZ, aoMaxHz],
// pick the frequency closest to TARGET_AO_RATE_HZ; tie -> higher frequency.
// Returns false when no valid AO rate exists for this pixel rate.
bool ComputeAORateHz(double pixelRateHz, double timebaseHz, double aoMaxHz,
                     double *aoRateHz) {
    long pixelPeriodTicks = lround(timebaseHz / pixelRateHz);
    bool found = false;
    double bestFreq = 0.0, bestDist = 0.0;
    for (long t = 1; t <= pixelPeriodTicks; ++t) {
        if (pixelPeriodTicks % t != 0)
            continue; // K = pixelPeriodTicks / t must be integer
        double f = timebaseHz / (double)t;
        if (f < MIN_AO_RATE_HZ || f > aoMaxHz)
            continue;
        double d = fabs(f - TARGET_AO_RATE_HZ);
        if (!found || d < bestDist || (d == bestDist && f > bestFreq)) {
            found = true;
            bestDist = d;
            bestFreq = f;
        }
    }
    if (found)
        *aoRateHz = bestFreq;
    return found;
}

// Populate GetImplData(device)->sampClkTimebaseHz and ->aoMaxRateHz on first
// call; no-op afterwards (guarded by ->timingCapsQueried).
OScDev_RichError *EnsureTimingCapsQueried(OScDev_Device *device) {
    if (GetImplData(device)->timingCapsQueried)
        return OScDev_RichError_OK;

    const char *dev = ss8_cstr(&GetImplData(device)->deviceName);
    OScDev_RichError *err;

    // AO max rate: plain device attribute, no task needed.
    float64 aoMax;
    err = CreateDAQmxError(DAQmxGetDevAOMaxRate(dev, &aoMax));
    if (err)
        return OScDev_Error_Wrap(err,
                                 "Failed to query AO maximum sample rate");

    // Timebase: throwaway committed AO task.
    ss8str aoChan;
    ss8_init_copy(&aoChan, &GetImplData(device)->deviceName);
    ss8_cat_cstr(&aoChan, "/ao0");

    TaskHandle task = 0;
    float64 timebase = 0.0;
    err = CreateDAQmxError(DAQmxCreateTask("", &task));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to create timing-query task");
        goto cleanup;
    }
    err = CreateDAQmxError(DAQmxCreateAOVoltageChan(
        task, ss8_cstr(&aoChan), "", -10.0, 10.0, DAQmx_Val_Volts, NULL));
    if (err) {
        err = OScDev_Error_Wrap(
            err, "Failed to create ao channel for timing query");
        goto cleanup;
    }
    err = CreateDAQmxError(DAQmxCfgSampClkTiming(
        task, "", 100000.0, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1000));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to configure timing-query task");
        goto cleanup;
    }
    err = CreateDAQmxError(DAQmxTaskControl(task, DAQmx_Val_Task_Commit));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to commit timing-query task");
        goto cleanup;
    }
    err = CreateDAQmxError(DAQmxGetSampClkTimebaseRate(task, &timebase));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to query samp clk timebase rate");
        goto cleanup;
    }

    GetImplData(device)->aoMaxRateHz = aoMax;
    GetImplData(device)->sampClkTimebaseHz = timebase;
    GetImplData(device)->timingCapsQueried = true;

cleanup:
    if (task)
        DAQmxClearTask(task);
    ss8_destroy(&aoChan);
    return err;
}

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
    parameters->pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    parameters->aoRateHz = GetImplData(device)->aoRateHz;
    parameters->undershootUs = GetImplData(device)->undershootUs;
    parameters->scanPhaseUs = GetImplData(device)->scanPhaseUs;
    parameters->retraceScaleUsPerVolt =
        GetImplData(device)->retraceScaleUsPerVolt;
    for (int i = 0; i < 4; ++i)
        parameters->xformMatrix[i] = GetImplData(device)->xformMatrix[i];
    parameters->xformOffsetX = GetImplData(device)->xformOffsetX;
    parameters->xformOffsetY = GetImplData(device)->xformOffsetY;
    parameters->xPark = GetImplData(device)->xPark;
    parameters->yPark = GetImplData(device)->yPark;
    parameters->prevXParkVoltage = GetImplData(device)->prevXParkVoltage;
    parameters->prevYParkVoltage = GetImplData(device)->prevYParkVoltage;
    parameters->laserBlankingSupported = LaserBlankingSupported(device);
    parameters->laserManualOn = GetImplData(device)->laserManualOn;
    parameters->laserOnVoltage = GetImplData(device)->laserOnVoltage;
    parameters->laserOffVoltage = GetImplData(device)->laserOffVoltage;
    parameters->laserOnLeadUs = GetImplData(device)->laserOnLeadUs;
    parameters->laserOnLagUs = GetImplData(device)->laserOnLagUs;
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

OScDev_RichError *EnumerateAOPhysChans(OScDev_Device *device) {
    ss8str *dest = &GetImplData(device)->aoPhysChans;
    ss8_set_len(dest, 1024);
    ss8_set_front(dest, '\0');
    int32 nierr = DAQmxGetDevAOPhysicalChans(
        ss8_cstr(&GetImplData(device)->deviceName), ss8_mutable_cstr(dest),
        (uInt32)ss8_len(dest));
    ss8_set_len_to_cstrlen(dest);
    ss8_shrink_to_fit(dest);
    if (nierr < 0)
        return CreateDAQmxError(nierr);
    if (ss8_is_empty(dest))
        return OScDev_Error_Create("Device has no AO physical channels");
    return OScDev_RichError_OK;
}

int GetNumberOfAOPhysChans(OScDev_Device *device) {
    const ss8str *chans = &GetImplData(device)->aoPhysChans;
    if (ss8_is_empty(chans))
        return 0;
    int count = 1;
    for (size_t p = ss8_find_ch(chans, 0, ','); p != SIZE_MAX;
         p = ss8_find_ch(chans, p + 1, ','))
        ++count;
    return count;
}

bool LaserBlankingSupported(OScDev_Device *device) {
    return GetNumberOfAOPhysChans(device) >= 3;
}

int GetNumberOfScannerAOChannels(OScDev_Device *device) {
    return LaserBlankingSupported(device) ? 3 : 2;
}
