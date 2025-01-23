#include "Acquisition.h"

#include "Clock.h"
#include "Detector.h"
#include "OScNIDAQ.h"
#include "OScNIDAQPrivateData.h"
#include "ParkUnpark.h"
#include "Scanner.h"
#include "Waveform.h"

#include <OpenScanDeviceLib.h>

#include <stdint.h>

#include <Windows.h>

// DAQ version; start all tasks
// Arm acquisition task first. Then make sure the (digital) line clock output
// is armed before the (analog) waveform output.
// This will ensure both tasks will start at the same time.
static OScDev_RichError *StartScan(OScDev_Device *device) {
    OScDev_RichError *err;
    if (!GetData(device)->scannerOnly) {
        err = StartDetector(&GetData(device)->detectorConfig);
        if (err)
            return err;
    } else
        OScDev_Log_Debug(device, "DAQ not used as detector");

    err = StartClock(&GetData(device)->clockConfig);
    if (err)
        return err;

    err = StartScanner(&GetData(device)->scannerConfig);
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
        err = StopDetector(&GetData(device)->detectorConfig);
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

    err = StopClock(&GetData(device)->clockConfig);
    if (err)
        lastErr = err;

    err = StopScanner(&GetData(device)->scannerConfig);
    if (err)
        lastErr = err;

    return lastErr;
}

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
