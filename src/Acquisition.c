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
    uint32_t totalFrames = OScDev_Acquisition_GetNumberOfFrames(acq);
    if (totalFrames != GetImplData(device)->configuredTotalFrames) {
        GetImplData(device)->clockConfig.mustReconfigureTiming = true;
        GetImplData(device)->scannerConfig.mustReconfigureTiming = true;
    }
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

    if (!GetImplData(device)->spiralScanEnabled) {
        err = SetUpClock(device, &GetImplData(device)->clockConfig, acq);
        if (err)
            return err;
    }
    if (!GetImplData(device)->scannerOnly &&
        !GetImplData(device)->spiralScanEnabled) {
        err = SetUpDetector(device, &GetImplData(device)->detectorConfig, acq);
        if (err)
            return err;
    }

    pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    resolution = OScDev_Acquisition_GetResolution(acq);
    zoomFactor = OScDev_Acquisition_GetZoomFactor(acq);
    OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);
    GetImplData(device)->configuredTotalFrames = totalFrames;
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
    if (!GetImplData(device)->scannerOnly &&
        !GetImplData(device)->spiralScanEnabled) {
        err = StartDetector(&GetImplData(device)->detectorConfig);
        if (err)
            return err;
    } else
        OScDev_Log_Debug(device, "DAQ not used as detector");

    if (!GetImplData(device)->spiralScanEnabled) {
        err = StartClock(&GetImplData(device)->clockConfig);
        if (err)
            goto stop_detector;
    }

    err = StartScanner(&GetImplData(device)->scannerConfig);
    if (err)
        goto stop_clock;

    return OScDev_RichError_OK;

stop_clock:
    if (!GetImplData(device)->spiralScanEnabled)
        OScDev_Error_Destroy(StopClock(&GetImplData(device)->clockConfig));
stop_detector:
    if (!GetImplData(device)->scannerOnly &&
        !GetImplData(device)->spiralScanEnabled)
        OScDev_Error_Destroy(
            StopDetector(&GetImplData(device)->detectorConfig));
    return err;
}

static OScDev_RichError *StopScan(OScDev_Device *device) {
    OScDev_RichError *err, *lastErr = OScDev_RichError_OK;

    if (!GetImplData(device)->scannerOnly &&
        !GetImplData(device)->spiralScanEnabled) {
        err = StopDetector(&GetImplData(device)->detectorConfig);
        if (err)
            lastErr = err;
    }

    if (!GetImplData(device)->spiralScanEnabled) {
        err = StopClock(&GetImplData(device)->clockConfig);
        if (err)
            lastErr = err;
    }

    err = StopScanner(&GetImplData(device)->scannerConfig);
    if (err)
        lastErr = err;

    return lastErr;
}

static void LogRichError(OScDev_Device *device, OScDev_RichError *err) {
    char msg[OScDev_MAX_STR_LEN + 1];
    OScDev_Error_FormatRecursive(err, msg, sizeof(msg));
    OScDev_Log_Error(device, msg);
}

static DWORD WINAPI AcquisitionLoop(void *param) {
    OScDev_Device *device = (OScDev_Device *)param;
    OScDev_Acquisition *acq = GetImplData(device)->acquisition.acquisition;

    uint32_t totalFrames = OScDev_Acquisition_GetNumberOfFrames(acq);
    OScDev_RichError *err;

    err = CreateScannerTask(device, &GetImplData(device)->scannerConfig);
    if (err) {
        LogRichError(device, err);
        goto finish;
    }

    err = ConfigureUnparkTiming(device, &GetImplData(device)->scannerConfig,
                                acq);
    if (err) {
        LogRichError(device, err);
        goto finish;
    }

    err = WriteUnparkOutput(device, &GetImplData(device)->scannerConfig, acq);
    if (err) {
        LogRichError(device, err);
        goto finish;
    }

    err =
        GenerateUnparkOutput(device, &GetImplData(device)->scannerConfig, acq);
    if (err) {
        LogRichError(device, err);
        goto finish;
    }

    err = SetUpScanner(device, &GetImplData(device)->scannerConfig, acq);
    if (err) {
        LogRichError(device, err);
        goto park;
    }

    GetImplData(device)->readBufferState = READ_BUFFER_IDLE;
    GetImplData(device)->framePixelsFilled = 0;
    GetImplData(device)->rawDataSize = 0;
    GetImplData(device)->activeWriteBuffer = 0;

    uint32_t estFrameTimeMs;
    if (GetImplData(device)->spiralScanEnabled) {
        struct SpiralWaveformParams spiralParams;
        SetSpiralWaveformParamsFromDevice(device, &spiralParams, acq);
        uint32_t totalElementsPerFramePerChan =
            GetSpiralWaveformSize(&spiralParams);
        estFrameTimeMs = (uint32_t)(1e3 * totalElementsPerFramePerChan /
                                    SPIRAL_SAMPLE_RATE_HZ);
    } else {
        double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
        struct WaveformParams params;
        SetWaveformParamsFromDevice(device, &params, acq);
        uint32_t totalElementsPerFramePerChan =
            GetScannerWaveformSize(&params);
        estFrameTimeMs =
            (uint32_t)(1e3 * totalElementsPerFramePerChan / pixelRateHz);
    }

    err = StartScan(device);
    if (err) {
        LogRichError(device, err);
        goto park;
    }

    for (uint32_t frame = 0; totalFrames >= INT32_MAX || frame < totalFrames;
         ++frame) {
        bool stopRequested;
        EnterCriticalSection(&(GetImplData(device)->acquisition.mutex));
        stopRequested = GetImplData(device)->acquisition.stopRequested;
        LeaveCriticalSection(&(GetImplData(device)->acquisition.mutex));
        if (stopRequested)
            break;

        char msg[OScDev_MAX_STR_LEN + 1];
        snprintf(msg, OScDev_MAX_STR_LEN, "Acquiring frame # %u", frame);
        OScDev_Log_Debug(device, msg);

        if (GetImplData(device)->scannerOnly) {
            Sleep(estFrameTimeMs);
        } else if (GetImplData(device)->spiralScanEnabled) {
            Sleep(estFrameTimeMs);
            // Emit dummy (blank) images in case spiral scan is run with
            // detector(s) enabled.
            uint32_t xOff, yOff, w, h;
            OScDev_Acquisition_GetROI(acq, &xOff, &yOff, &w, &h);
            uint16_t *zeroBuffer =
                (uint16_t *)calloc((size_t)w * h, sizeof(uint16_t));
            int nChans = GetNumberOfEnabledChannels(device);
            for (int ch = 0; ch < nChans; ++ch) {
                if (!OScDev_Acquisition_CallFrameCallback(acq, ch, zeroBuffer))
                    break;
            }
            free(zeroBuffer);
        } else {
            int rb = 0;

            EnterCriticalSection(&GetImplData(device)->frameMutex);
            while (GetImplData(device)->readBufferState != READ_BUFFER_READY) {
                if (!SleepConditionVariableCS(&GetImplData(device)->frameReady,
                                              &GetImplData(device)->frameMutex,
                                              2 * estFrameTimeMs)) {
                    break; // timeout
                }
                bool stopReq;
                EnterCriticalSection(&GetImplData(device)->acquisition.mutex);
                stopReq = GetImplData(device)->acquisition.stopRequested;
                LeaveCriticalSection(&GetImplData(device)->acquisition.mutex);
                if (stopReq)
                    break;
            }

            bool gotFrame =
                GetImplData(device)->readBufferState == READ_BUFFER_READY;
            if (gotFrame) {
                rb = 1 - GetImplData(device)->activeWriteBuffer;
                GetImplData(device)->readBufferState = READ_BUFFER_READING;
            }
            LeaveCriticalSection(&GetImplData(device)->frameMutex);

            if (!gotFrame) {
                EnterCriticalSection(&GetImplData(device)->acquisition.mutex);
                stopRequested = GetImplData(device)->acquisition.stopRequested;
                LeaveCriticalSection(&GetImplData(device)->acquisition.mutex);
                if (!stopRequested)
                    OScDev_Log_Error(device, "Error: Acquisition timeout!");
                break;
            }

            int nChans = GetNumberOfEnabledChannels(device);
            bool ok = true;
            for (int ch = 0; ch < nChans; ++ch) {
                ok = OScDev_Acquisition_CallFrameCallback(
                    acq, ch, GetImplData(device)->frameBuffers[rb][ch]);
                if (!ok) {
                    break;
                }
            }

            // Mark buffer finished even if we fail (!ok), so that the producer
            // thread (DAQmx callback) won't have a buffer overflow error in
            // the meantime.
            EnterCriticalSection(&GetImplData(device)->frameMutex);
            GetImplData(device)->readBufferState = READ_BUFFER_IDLE;
            LeaveCriticalSection(&GetImplData(device)->frameMutex);

            if (!ok) {
                OScDev_Log_Error(
                    device,
                    "Stopping acquisition because frame could not be transmitted");
                break;
            }
        }
    }

    err = StopScan(device);
    if (err)
        LogRichError(device, err);

park:
    err = CreateScannerTask(device, &GetImplData(device)->scannerConfig);
    if (err) {
        LogRichError(device, err);
        goto finish;
    }

    err =
        ConfigureParkTiming(device, &GetImplData(device)->scannerConfig, acq);
    if (err) {
        LogRichError(device, err);
        goto finish;
    }

    err = WriteParkOutput(device, &GetImplData(device)->scannerConfig, acq);
    if (err) {
        LogRichError(device, err);
        goto finish;
    }

    err = GenerateParkOutput(device, &GetImplData(device)->scannerConfig, acq);
    if (err)
        LogRichError(device, err);

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
    HANDLE thread = CreateThread(NULL, 0, AcquisitionLoop, device, 0, &id);
    if (!thread) {
        EnterCriticalSection(&GetImplData(device)->acquisition.mutex);
        GetImplData(device)->acquisition.started = false;
        GetImplData(device)->acquisition.running = false;
        LeaveCriticalSection(&GetImplData(device)->acquisition.mutex);
        return OScDev_Error_Create("Failed to create acquisition thread");
    }
    CloseHandle(thread);
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
