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

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <Windows.h>

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
        GetImplData(device)->scannerOnly ? estFrameTime : yRetraceTime;
    char msg[OScDev_MAX_STR_LEN + 1];
    snprintf(msg, OScDev_MAX_STR_LEN, "Wait %f s for scan to finish...",
             secondsToWait);
    OScDev_Log_Debug(device, msg);
    Sleep((DWORD)ceil(secondsToWait * 1000));

    return OScDev_RichError_OK;
}

static OScDev_RichError *StopScan(OScDev_Device *device,
                                  OScDev_Acquisition *acq) {
    OScDev_RichError *err, *lastErr = OScDev_RichError_OK;

    // Stopping a task may return an error if it failed, so make sure to stop
    // all tasks even if we get errors.

    if (!GetImplData(device)->scannerOnly) {
        err = StopDetector(&GetImplData(device)->detectorConfig);
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

    err = StopClock(&GetImplData(device)->clockConfig);
    if (err)
        lastErr = err;

    err = StopScanner(&GetImplData(device)->scannerConfig);
    if (err)
        lastErr = err;

    return lastErr;
}

static OScDev_RichError *AcquireFrame(OScDev_Device *device,
                                      OScDev_Acquisition *acq) {
    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);
    GetImplData(device)->oneFrameScanDone = false;
    GetImplData(device)->framePixelsFilled = 0;

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
        GetImplData(device)->scannerConfig.aoTask, 2 * estFrameTimeMs * 1e-3));
    if (err) {
        err = OScDev_Error_Wrap(err,
                                "Failed to wait for scanner task to finish");
        return err;
    }

    // Wait for data
    if (!GetImplData(device)->scannerOnly) {
        while (!GetImplData(device)->oneFrameScanDone) {
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

    if (!GetImplData(device)->scannerOnly) {
        int nChans = GetNumberOfEnabledChannels(device);
        for (int ch = 0; ch < nChans; ++ch) {
            bool shouldContinue = OScDev_Acquisition_CallFrameCallback(
                acq, ch, GetImplData(device)->frameBuffers[ch]);
            if (!shouldContinue) {
                // TODO Stop acquisition
            }
        }
    }

    return OScDev_RichError_OK;
}

static DWORD WINAPI AcquisitionLoop(void *param) {
    OScDev_Device *device = (OScDev_Device *)param;
    OScDev_Acquisition *acq = GetImplData(device)->acquisition.acquisition;

    uint32_t totalFrames = OScDev_Acquisition_GetNumberOfFrames(acq);

    // insert from parking to start here
    CreateScannerTask(device, &GetImplData(device)->scannerConfig);
    ConfigureUnparkTiming(device, &GetImplData(device)->scannerConfig, acq);
    WriteUnparkOutput(device, &GetImplData(device)->scannerConfig, acq);
    GenerateUnparkOutput(device, &GetImplData(device)->scannerConfig, acq);

    // prepare raster waveform
    SetUpScanner(device, &GetImplData(device)->scannerConfig, acq);

    for (uint32_t frame = 0; frame < totalFrames; ++frame) {
        bool stopRequested;
        EnterCriticalSection(&(GetImplData(device)->acquisition.mutex));
        stopRequested = GetImplData(device)->acquisition.stopRequested;
        LeaveCriticalSection(&(GetImplData(device)->acquisition.mutex));
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
            OScDev_Error_FormatRecursive(err, msg, sizeof(msg));
            OScDev_Log_Error(device, msg);
            break;
        }
    }

    // insert from start to parking here
    ConfigureParkTiming(device, &GetImplData(device)->scannerConfig, acq);
    WriteParkOutput(device, &GetImplData(device)->scannerConfig, acq);
    GenerateParkOutput(device, &GetImplData(device)->scannerConfig, acq);

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
    err = ReconfigDAQ(device);
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
            LeaveCriticalSection(&(GetImplData(device)->acquisition.mutex));
        } else if (GetImplData(device)->acquisition.started) {
            LeaveCriticalSection(&(GetImplData(device)->acquisition.mutex));
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
