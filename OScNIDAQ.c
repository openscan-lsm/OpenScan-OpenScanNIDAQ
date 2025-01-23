#include "OScNIDAQ.h"

#include "OScNIDAQPrivateData.h"
#include "ParkUnpark.h"
#include "Waveform.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>
#include <ss8str.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <Windows.h>

static bool GetAIPhysChan(OScDev_Device *device, int index, ss8str *chan);

// Must be called immediately after failed DAQmx function
void LogNiError(OScDev_Device *device, int32 nierr, const char *when) {
    ss8str msg;
    ss8_init_copy_cstr(&msg, "DAQmx error while ");
    ss8_cat_cstr(&msg, when);
    ss8_cat_cstr(&msg, "; extended error info follows");
    OScDev_Log_Error(device, ss8_cstr(&msg));

    ss8_set_len(&msg, 1024);
    DAQmxGetExtendedErrorInfo(ss8_mutable_cstr(&msg), (uInt32)ss8_len(&msg));
    ss8_set_len_to_cstrlen(&msg);
    OScDev_Log_Error(device, ss8_cstr(&msg));
    ss8_destroy(&msg);
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

// Fill in non-zero defaults only
static void InitializePrivateData(struct OScNIDAQPrivateData *data) {
    ss8_init(&data->deviceName);
    data->lineDelay = 50;
    data->numLinesToBuffer = 8;
    data->inputVoltageRange = 10.0;
    data->minVolts_ = -10.0;
    data->maxVolts_ = 10.0;
    ss8_init(&data->aiPhysChans);
    data->channelEnabled[0] = true;

    InitializeCriticalSection(&(data->acquisition.mutex));
    InitializeConditionVariable(
        &(data->acquisition.acquisitionFinishCondition));
}

void SetWaveformParamsFromDevice(OScDev_Device *device,
                                 struct WaveformParams *parameters,
                                 OScDev_Acquisition *acq) {
    parameters->resolution = OScDev_Acquisition_GetResolution(acq);
    parameters->zoom = OScDev_Acquisition_GetZoomFactor(acq);
    OScDev_Acquisition_GetROI(acq, &parameters->xOffset, &parameters->yOffset,
                              &parameters->width, &parameters->height);
    parameters->undershoot = GetData(device)->lineDelay;
    parameters->galvoOffsetX = GetData(device)->offsetXY[0];
    parameters->galvoOffsetY = GetData(device)->offsetXY[1];
    parameters->xPark = GetData(device)->xPark;
    parameters->yPark = GetData(device)->yPark;
    parameters->prevXParkVoltage = GetData(device)->prevXParkVoltage;
    parameters->prevYParkVoltage = GetData(device)->prevYParkVoltage;
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

        struct OScNIDAQPrivateData *data =
            calloc(1, sizeof(struct OScNIDAQPrivateData));
        InitializePrivateData(data);
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
    ss8str *dest = &GetData(device)->aiPhysChans;
    ss8_set_len(dest, 1024);
    ss8_set_front(dest, '\0');
    int32 nierr = DAQmxGetDevAIPhysicalChans(
        ss8_cstr(&GetData(device)->deviceName), ss8_mutable_cstr(dest),
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
        if (GetData(device)->channelEnabled[i]) {
            ++ret;
        }
    }
    return ret;
}

void GetEnabledChannels(OScDev_Device *device, ss8str *chans) {
    ss8str chan;
    ss8_init(&chan);

    for (int i = 0; i < MAX_PHYSICAL_CHANS; ++i) {
        if (GetData(device)->channelEnabled[i]) {
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

// Return the index-th physical channel, or empty string if no such channel
static bool GetAIPhysChan(OScDev_Device *device, int index, ss8str *chan) {
    if (index < 0) {
        if (chan)
            ss8_clear(chan);
        return false;
    }

    ss8str chans;
    ss8_init_copy(&chans, &GetData(device)->aiPhysChans);

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

// DAQ version; start all tasks
// Arm acquisition task first. Then make sure the (digital) line clock output
// is armed before the (analog) waveform output.
// This will ensure both tasks will start at the same time.
static OScDev_RichError *StartScan(OScDev_Device *device) {
    OScDev_RichError *err;
    if (!GetData(device)->scannerOnly) {
        err = StartDetector(device, &GetData(device)->detectorConfig);
        if (err)
            return err;
    } else
        OScDev_Log_Debug(device, "DAQ not used as detector");

    err = StartClock(device, &GetData(device)->clockConfig);
    if (err)
        return err;

    err = StartScanner(device, &GetData(device)->scannerConfig);
    if (err)
        return err;

    return OScDev_RichError_OK;
}

static OScDev_RichError *WaitScanToFinish(OScDev_Device *device,
                                          OScDev_Acquisition *acq) {
    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    struct WaveformParams parameters;
    SetWaveformParamsFromDevice(device, &parameters, acq);

    // When scanRate is low, it takes longer to finish generating scan
    // waveform. Since acquisition only takes a portion of the total scan time,
    // it may occur that waveform task is stopped right after acquisition is
    // done but the waveform generation is not done yet -- thus nierr 200010:
    // "Finite acquisition or generation has been stopped before the requested
    // number of samples were acquired or generated." So need to wait some
    // miliseconds till waveform generation is done before stop the task.
    double yRetraceTime =
        GetScannerWaveformSizeAfterLastPixel(&parameters) / pixelRateHz;
    double estFrameTime = GetScannerWaveformSize(&parameters) / pixelRateHz;
    double secondsToWait =
        GetData(device)->scannerOnly ? estFrameTime : yRetraceTime;
    char msg[OScDev_MAX_STR_LEN + 1];
    snprintf(msg, OScDev_MAX_STR_LEN, "Wait %f s for scan to finish...",
             secondsToWait);
    OScDev_Log_Debug(device, msg);
    Sleep((DWORD)ceil(secondsToWait * 1000));

    return OScDev_RichError_OK;
}

// stop running tasks
// need to stop detector first, then clock and scanner
static OScDev_RichError *StopScan(OScDev_Device *device,
                                  OScDev_Acquisition *acq) {
    OScDev_RichError *err, *lastErr = OScDev_RichError_OK;

    // Stopping a task may return an error if it failed, so make sure to stop
    // all tasks even if we get errors.

    if (!GetData(device)->scannerOnly) {
        err = StopDetector(device, &GetData(device)->detectorConfig);
        if (err)
            lastErr = err;
    }

    // When scanRate is low, it takes longer to finish generating scan
    // waveform. Since acquisition only takes a portion of the total scan time,
    // it may occur that waveform task is stopped right after acquisition is
    // done but the waveform generation is not done yet -- thus nierr 200010:
    // "Finite acquisition or generation has been stopped before the requested
    // number of samples were acquired or generated." So need to wait some
    // miliseconds till waveform generation is done before stop the task.
    err = WaitScanToFinish(device, acq);
    if (err)
        return err;

    err = StopClock(device, &GetData(device)->clockConfig);
    if (err)
        lastErr = err;

    err = StopScanner(device, &GetData(device)->scannerConfig);
    if (err)
        lastErr = err;

    return lastErr;
}

// DAQ version; acquire from multiple channels
static OScDev_RichError *ReadImage(OScDev_Device *device,
                                   OScDev_Acquisition *acq) {
    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);
    GetData(device)->oneFrameScanDone = false;
    GetData(device)->framePixelsFilled = 0;

    uint32_t totalElementsPerFramePerChan = GetScannerWaveformSize(&params);
    uint32_t estFrameTimeMs =
        (uint32_t)(1e3 * totalElementsPerFramePerChan / pixelRateHz);
    uint32_t totalWaitTimeMs = 0;

    OScDev_RichError *err;
    err = StartScan(device);
    if (err)
        return err;

    // Wait for scan to complete
    err = CreateDAQmxError(DAQmxWaitUntilTaskDone(
        GetData(device)->scannerConfig.aoTask, 2 * estFrameTimeMs * 1e-3));
    if (err) {
        err = OScDev_Error_Wrap(err,
                                "Failed to wait for scanner task to finish");
        return err;
    }

    // Wait for data
    if (!GetData(device)->scannerOnly) {
        while (!GetData(device)->oneFrameScanDone) {
            Sleep(1);
            totalWaitTimeMs += 1;
            if (totalWaitTimeMs > 2 * estFrameTimeMs) {
                OScDev_Log_Error(device, "Error: Acquisition timeout!");
                break;
            }
        }
        char msg[OScDev_MAX_STR_LEN + 1];
        snprintf(msg, OScDev_MAX_STR_LEN, "Total wait time is %d ",
                 totalWaitTimeMs);
        OScDev_Log_Debug(device, msg);
    }

    err = StopScan(device, acq);
    if (err)
        return err;

    if (!GetData(device)->scannerOnly) {
        int nChans = GetNumberOfEnabledChannels(device);
        for (int ch = 0; ch < nChans; ++ch) {
            bool shouldContinue = OScDev_Acquisition_CallFrameCallback(
                acq, ch, GetData(device)->frameBuffers[ch]);
            if (!shouldContinue) {
                // TODO Stop acquisition
            }
        }
    }

    return OScDev_RichError_OK;
}

static OScDev_RichError *AcquireFrame(OScDev_Device *device,
                                      OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    OScDev_Log_Debug(device, "Reading image...");
    err = ReadImage(device, acq);
    if (err)
        return err;
    OScDev_Log_Debug(device, "Finished reading image");

    return OScDev_RichError_OK;
}

static DWORD WINAPI AcquisitionLoop(void *param) {
    OScDev_Device *device = (OScDev_Device *)param;
    OScDev_Acquisition *acq = GetData(device)->acquisition.acquisition;

    uint32_t totalFrames = OScDev_Acquisition_GetNumberOfFrames(acq);

    // insert from parking to start here
    CreateScannerTask(device, &GetData(device)->scannerConfig);
    ConfigureUnparkTiming(device, &GetData(device)->scannerConfig, acq);
    WriteUnparkOutput(device, &GetData(device)->scannerConfig, acq);
    GenerateUnparkOutput(device, &GetData(device)->scannerConfig, acq);

    // prepare raster waveform
    SetUpScanner(device, &GetData(device)->scannerConfig, acq);

    for (uint32_t frame = 0; frame < totalFrames; ++frame) {
        bool stopRequested;
        EnterCriticalSection(&(GetData(device)->acquisition.mutex));
        stopRequested = GetData(device)->acquisition.stopRequested;
        LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
        if (stopRequested)
            break;

        char msg[OScDev_MAX_STR_LEN + 1];
        snprintf(msg, OScDev_MAX_STR_LEN, "Sequence acquiring frame # %d",
                 frame);
        OScDev_Log_Debug(device, msg);

        OScDev_RichError *err;
        err = AcquireFrame(device, acq);
        if (err) {
            err = OScDev_Error_Wrap(err, "Error during sequence acquisition");
            char msg[OScDev_MAX_STR_LEN + 1];
            OScDev_Error_FormatRecursive(err, msg, sizeof(msg));
            OScDev_Log_Error(device, msg);
            break;
        }
    }

    // insert from start to parking here
    ConfigureParkTiming(device, &GetData(device)->scannerConfig, acq);
    WriteParkOutput(device, &GetData(device)->scannerConfig, acq);
    GenerateParkOutput(device, &GetData(device)->scannerConfig, acq);

    EnterCriticalSection(&(GetData(device)->acquisition.mutex));
    GetData(device)->acquisition.running = false;
    LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
    CONDITION_VARIABLE *cv =
        &(GetData(device)->acquisition.acquisitionFinishCondition);
    WakeAllConditionVariable(cv);

    return 0;
}

OScDev_RichError *RunAcquisitionLoop(OScDev_Device *device) {
    DWORD id;
    GetData(device)->acquisition.thread =
        CreateThread(NULL, 0, AcquisitionLoop, device, 0, &id);
    return OScDev_RichError_OK;
}

OScDev_RichError *StopAcquisitionAndWait(OScDev_Device *device) {
    CRITICAL_SECTION *mutex = &GetData(device)->acquisition.mutex;
    CONDITION_VARIABLE *cv =
        &(GetData(device)->acquisition.acquisitionFinishCondition);

    EnterCriticalSection(mutex);
    if (GetData(device)->acquisition.started) {
        GetData(device)->acquisition.stopRequested = true;
    } else { // Armed but not started
        GetData(device)->acquisition.running = false;
    }

    while (GetData(device)->acquisition.running) {
        SleepConditionVariableCS(cv, mutex, INFINITE);
    }
    LeaveCriticalSection(mutex);

    return OScDev_RichError_OK;
}

OScDev_RichError *IsAcquisitionRunning(OScDev_Device *device,
                                       bool *isRunning) {
    EnterCriticalSection(&(GetData(device)->acquisition.mutex));
    *isRunning = GetData(device)->acquisition.running;
    LeaveCriticalSection(&(GetData(device)->acquisition.mutex));
    return OScDev_RichError_OK;
}

OScDev_RichError *WaitForAcquisitionToFinish(OScDev_Device *device) {
    CRITICAL_SECTION *mutex = &GetData(device)->acquisition.mutex;
    CONDITION_VARIABLE *cv =
        &(GetData(device)->acquisition.acquisitionFinishCondition);

    EnterCriticalSection(mutex);
    while (GetData(device)->acquisition.running) {
        SleepConditionVariableCS(cv, mutex, INFINITE);
    }
    LeaveCriticalSection(mutex);

    return OScDev_RichError_OK;
}

OScDev_RichError *ReconfigDAQ(OScDev_Device *device, OScDev_Acquisition *acq) {
    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    uint32_t resolution = OScDev_Acquisition_GetResolution(acq);
    double zoomFactor = OScDev_Acquisition_GetZoomFactor(acq);
    uint32_t xOffset, yOffset, width, height;
    OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);
    if (pixelRateHz != GetData(device)->configuredPixelRateHz) {
        GetData(device)->clockConfig.mustReconfigureTiming = true;
        GetData(device)->scannerConfig.mustReconfigureTiming = true;
        GetData(device)->detectorConfig.mustReconfigureTiming = true;
    }
    if (resolution != GetData(device)->configuredResolution) {
        GetData(device)->scannerConfig.mustReconfigureTiming = true;
        GetData(device)->scannerConfig.mustRewriteOutput = true;
    }
    if (zoomFactor != GetData(device)->configuredZoomFactor) {
        GetData(device)->clockConfig.mustRewriteOutput = true;
        GetData(device)->scannerConfig.mustRewriteOutput = true;
    }
    if (xOffset != GetData(device)->configuredXOffset ||
        yOffset != GetData(device)->configuredYOffset) {
        GetData(device)->scannerConfig.mustRewriteOutput = true;
    }
    if (width != GetData(device)->configuredRasterWidth ||
        height != GetData(device)->configuredRasterHeight) {
        GetData(device)->clockConfig.mustReconfigureTiming = true;
        GetData(device)->scannerConfig.mustReconfigureTiming = true;
        GetData(device)->detectorConfig.mustReconfigureTiming = true;
        GetData(device)->clockConfig.mustRewriteOutput = true;
        GetData(device)->scannerConfig.mustRewriteOutput = true;
        GetData(device)->detectorConfig.mustReconfigureCallback = true;
    }

    // Note that additional setting of 'mustReconfigure' flags occurs in
    // settings

    OScDev_RichError *err;

    err = SetUpClock(device, &GetData(device)->clockConfig, acq);
    if (err)
        return err;
    if (!GetData(device)->scannerOnly) {
        err = SetUpDetector(device, &GetData(device)->detectorConfig, acq);
        if (err)
            return err;
    }

    pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    resolution = OScDev_Acquisition_GetResolution(acq);
    zoomFactor = OScDev_Acquisition_GetZoomFactor(acq);
    OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);
    GetData(device)->configuredPixelRateHz = pixelRateHz;
    GetData(device)->configuredResolution = resolution;
    GetData(device)->configuredZoomFactor = zoomFactor;
    GetData(device)->configuredXOffset = xOffset;
    GetData(device)->configuredYOffset = yOffset;
    GetData(device)->configuredRasterWidth = width;
    GetData(device)->configuredRasterHeight = height;

    return OScDev_RichError_OK;
}
