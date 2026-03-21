#include "Acquisition.h"

#include "Clock.h"
#include "DAQConfig.h"
#include "DAQError.h"
#include "Detector.h"
#include "DeviceImplData.h"
#include "ParkUnpark.h"
#include "Scanner.h"
#include "Waveform.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>

#include <stdint.h>
#include <stdio.h>

#include <Windows.h>

static OScDev_RichError *SetUpDAQ(OScDev_Device *device) {
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

static OScDev_RichError *StartScan(OScDev_Device *device) {
    OScDev_RichError *err;
    if (!GetImplData(device)->scannerOnly) {
        err = StartDetector(&GetImplData(device)->detectorConfig);
        if (err)
            return err;
    } else
        OScDev_Log_Debug(device, "DAQ not used as detector");

    err = StartClock(&GetImplData(device)->clockConfig);
    if (err)
        return err;

    err = StartScanner(&GetImplData(device)->scannerConfig);
    if (err)
        return err;

    return OScDev_RichError_OK;
}

static OScDev_RichError *StopScan(OScDev_Device *device) {
    OScDev_RichError *err, *lastErr = OScDev_RichError_OK;

    if (!GetImplData(device)->scannerOnly) {
        err = StopDetector(&GetImplData(device)->detectorConfig);
        if (err)
            lastErr = err;
    }

    err = StopClock(&GetImplData(device)->clockConfig);
    if (err)
        lastErr = err;

    err = StopScanner(&GetImplData(device)->scannerConfig);
    if (err)
        lastErr = err;

    return lastErr;
}

static DWORD WINAPI AcquisitionLoop(void *param) {
    OScDev_Device *device = (OScDev_Device *)param;
    OScDev_Acquisition *acq = GetImplData(device)->acquisition.acquisition;

    uint32_t totalFrames = OScDev_Acquisition_GetNumberOfFrames(acq);

    CreateScannerTask(device, &GetImplData(device)->scannerConfig);
    ConfigureUnparkTiming(device, &GetImplData(device)->scannerConfig, acq);
    WriteUnparkOutput(device, &GetImplData(device)->scannerConfig, acq);
    GenerateUnparkOutput(device, &GetImplData(device)->scannerConfig, acq);

    SetUpScanner(device, &GetImplData(device)->scannerConfig, acq);

    GetImplData(device)->oneFrameScanDone = false;
    GetImplData(device)->framePixelsFilled = 0;
    GetImplData(device)->activeWriteBuffer = 0;

    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);
    uint32_t totalElementsPerFramePerChan = GetScannerWaveformSize(&params);
    uint32_t estFrameTimeMs =
        (uint32_t)(1e3 * totalElementsPerFramePerChan / pixelRateHz);

    OScDev_RichError *err;
    err = StartScan(device);
    if (err) {
        char msg[OScDev_MAX_STR_LEN + 1];
        OScDev_Error_FormatRecursive(err, msg, sizeof(msg));
        OScDev_Log_Error(device, msg);
        StopScan(device);
        goto finish;
    }

    for (uint32_t frame = 0; frame < totalFrames; ++frame) {
        bool stopRequested;
        EnterCriticalSection(&(GetImplData(device)->acquisition.mutex));
        stopRequested = GetImplData(device)->acquisition.stopRequested;
        LeaveCriticalSection(&(GetImplData(device)->acquisition.mutex));
        if (stopRequested)
            break;

        char msg[OScDev_MAX_STR_LEN + 1];
        snprintf(msg, OScDev_MAX_STR_LEN, "Acquiring frame # %u", frame);
        OScDev_Log_Debug(device, msg);

        if (!GetImplData(device)->scannerOnly) {
            int rb = 0;

            EnterCriticalSection(&GetImplData(device)->frameMutex);
            while (!GetImplData(device)->oneFrameScanDone) {
                if (!SleepConditionVariableCS(&GetImplData(device)->frameReady,
                                              &GetImplData(device)->frameMutex,
                                              2 * estFrameTimeMs)) {
                    break; // timeout
                }
                if (GetImplData(device)->acquisition.stopRequested)
                    break;
            }

            bool gotFrame = GetImplData(device)->oneFrameScanDone;
            if (gotFrame) {
                rb = GetImplData(device)->completedReadBuffer;
                GetImplData(device)->oneFrameScanDone = false;
            }
            LeaveCriticalSection(&GetImplData(device)->frameMutex);

            if (!gotFrame) {
                if (!GetImplData(device)->acquisition.stopRequested)
                    OScDev_Log_Error(device, "Error: Acquisition timeout!");
                break;
            }

            int nChans = GetNumberOfEnabledChannels(device);
            for (int ch = 0; ch < nChans; ++ch) {
                OScDev_Acquisition_CallFrameCallback(
                    acq, ch, GetImplData(device)->frameBuffers[rb][ch]);
            }
        } else {
            Sleep(estFrameTimeMs);
        }
    }

    StopScan(device);

    ConfigureParkTiming(device, &GetImplData(device)->scannerConfig, acq);
    WriteParkOutput(device, &GetImplData(device)->scannerConfig, acq);
    GenerateParkOutput(device, &GetImplData(device)->scannerConfig, acq);

finish:
    EnterCriticalSection(&(GetImplData(device)->acquisition.mutex));
    GetImplData(device)->acquisition.running = false;
    LeaveCriticalSection(&(GetImplData(device)->acquisition.mutex));
    CONDITION_VARIABLE *cv =
        &(GetImplData(device)->acquisition.acquisitionFinishCondition);
    WakeAllConditionVariable(cv);

    return 0;
}

OScDev_RichError *ArmAcquisition(OScDev_Device *device,
                                 OScDev_Acquisition *acq, bool scannerOnly) {
    CRITICAL_SECTION *mutex = &GetImplData(device)->acquisition.mutex;

    OScDev_RichError *err = OScDev_RichError_OK;
    EnterCriticalSection(mutex);
    {
        if (GetImplData(device)->acquisition.running) {
            err = OScDev_Error_Create("Acquisition already armed or running");
        } else {
            GetImplData(device)->acquisition.stopRequested = false;
            GetImplData(device)->acquisition.running = true;
            GetImplData(device)->acquisition.armed = false;
            GetImplData(device)->acquisition.started = false;
        }
    }
    LeaveCriticalSection(mutex);
    if (err)
        return err;

    GetImplData(device)->acquisition.acquisition = acq;
    GetImplData(device)->scannerOnly = scannerOnly;
    err = SetUpDAQ(device);
    if (err) {
        GetImplData(device)->acquisition.acquisition = NULL;
        EnterCriticalSection(mutex);
        { GetImplData(device)->acquisition.running = false; }
        LeaveCriticalSection(mutex);
        return err;
    }

    EnterCriticalSection(mutex);
    { GetImplData(device)->acquisition.armed = true; }
    LeaveCriticalSection(mutex);
    return OScDev_RichError_OK;
}

OScDev_RichError *StartAcquisition(OScDev_Device *device) {
    OScDev_RichError *err = OScDev_RichError_OK;
    EnterCriticalSection(&GetImplData(device)->acquisition.mutex);
    {
        if (!GetImplData(device)->acquisition.running ||
            !GetImplData(device)->acquisition.armed) {
            err = OScDev_Error_Create(
                "Cannot start acquisition without first arming");
        } else if (GetImplData(device)->acquisition.started) {
            err = OScDev_Error_Create(
                "Cannot start acquisition because acquisition already running");
        } else {
            GetImplData(device)->acquisition.started = true;
        }
    }
    LeaveCriticalSection(&GetImplData(device)->acquisition.mutex);
    if (err)
        return err;

    DWORD id;
    GetImplData(device)->acquisition.thread =
        CreateThread(NULL, 0, AcquisitionLoop, device, 0, &id);
    return OScDev_RichError_OK;
}

OScDev_RichError *StopAcquisitionAndWait(OScDev_Device *device) {
    CRITICAL_SECTION *mutex = &GetImplData(device)->acquisition.mutex;
    CONDITION_VARIABLE *cv =
        &(GetImplData(device)->acquisition.acquisitionFinishCondition);

    EnterCriticalSection(mutex);
    if (GetImplData(device)->acquisition.started) {
        GetImplData(device)->acquisition.stopRequested = true;
        WakeConditionVariable(&GetImplData(device)->frameReady);
    } else { // Armed but not started
        GetImplData(device)->acquisition.running = false;
    }

    while (GetImplData(device)->acquisition.running) {
        SleepConditionVariableCS(cv, mutex, INFINITE);
    }
    LeaveCriticalSection(mutex);

    return OScDev_RichError_OK;
}

OScDev_RichError *IsAcquisitionRunning(OScDev_Device *device,
                                       bool *isRunning) {
    EnterCriticalSection(&(GetImplData(device)->acquisition.mutex));
    *isRunning = GetImplData(device)->acquisition.running;
    LeaveCriticalSection(&(GetImplData(device)->acquisition.mutex));
    return OScDev_RichError_OK;
}

OScDev_RichError *WaitForAcquisitionToFinish(OScDev_Device *device) {
    CRITICAL_SECTION *mutex = &GetImplData(device)->acquisition.mutex;
    CONDITION_VARIABLE *cv =
        &(GetImplData(device)->acquisition.acquisitionFinishCondition);

    EnterCriticalSection(mutex);
    while (GetImplData(device)->acquisition.running) {
        SleepConditionVariableCS(cv, mutex, INFINITE);
    }
    LeaveCriticalSection(mutex);

    return OScDev_RichError_OK;
}
