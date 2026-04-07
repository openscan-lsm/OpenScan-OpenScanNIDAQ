#include "Scanner.h"

#include "DAQConfig.h"
#include "DAQError.h"
#include "DeviceImplData.h"
#include "Waveform.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>
#include <ss8str.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static OScDev_RichError *ConfigureScannerTiming(OScDev_Device *device,
                                                struct ScannerConfig *config,
                                                OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    double sampleRateHz;

    if (GetImplData(device)->spiralScanEnabled) {
        sampleRateHz = SPIRAL_SAMPLE_RATE_HZ;

        uInt64 bufferSize = 100000;
        err = CreateDAQmxError(DAQmxCfgSampClkTiming(
            config->aoTask, "", sampleRateHz, DAQmx_Val_Rising,
            DAQmx_Val_ContSamps, bufferSize));
        if (err) {
            err = OScDev_Error_Wrap(err,
                                    "Failed to configure timing for scanner");
            return err;
        }

        err = CreateDAQmxError(
            DAQmxSetWriteRegenMode(config->aoTask, DAQmx_Val_DoNotAllowRegen));
        if (err) {
            err =
                OScDev_Error_Wrap(err, "Failed to set regen mode for scanner");
            return err;
        }
    } else {
        sampleRateHz = OScDev_Acquisition_GetPixelRate(acq);
        struct WaveformParams params;
        SetWaveformParamsFromDevice(device, &params, acq);
        int32 totalElementsPerFramePerChan = GetScannerWaveformSize(&params);

        uint32_t totalFrames = OScDev_Acquisition_GetNumberOfFrames(acq);

        int sampleMode;
        uInt64 samplesPerChan;
        if (totalFrames >= INT32_MAX) {
            sampleMode = DAQmx_Val_ContSamps;
            samplesPerChan = totalElementsPerFramePerChan;
        } else {
            uInt64 totalSamples =
                (uInt64)totalFrames * totalElementsPerFramePerChan;
            if (totalSamples > UINT32_MAX)
                return OScDev_Error_Create(
                    "Total scanner samples exceed maximum for finite mode");
            sampleMode = DAQmx_Val_FiniteSamps;
            samplesPerChan = totalSamples;
        }

        err = CreateDAQmxError(DAQmxCfgSampClkTiming(
            config->aoTask, "", sampleRateHz, DAQmx_Val_Rising, sampleMode,
            samplesPerChan));
        if (err) {
            err = OScDev_Error_Wrap(err,
                                    "Failed to configure timing for scanner");
            return err;
        }

        err = CreateDAQmxError(
            DAQmxSetWriteRegenMode(config->aoTask, DAQmx_Val_AllowRegen));
        if (err) {
            err =
                OScDev_Error_Wrap(err, "Failed to set regen mode for scanner");
            return err;
        }
    }

    return OScDev_RichError_OK;
}

static const int32_t SPIRAL_CHUNK_SIZE = 10000;

static OScDev_RichError *WriteScannerOutput(OScDev_Device *device,
                                            struct ScannerConfig *config,
                                            OScDev_Acquisition *acq) {
    if (GetImplData(device)->spiralScanEnabled) {
        struct SpiralWaveformParams spiralParams;
        SetSpiralWaveformParamsFromDevice(device, &spiralParams, acq);

        if (config->spiralState)
            DestroySpiralGenState(config->spiralState);
        config->spiralState = CreateSpiralGenState(&spiralParams);
        config->spiralTotalFrames = OScDev_Acquisition_GetNumberOfFrames(acq);

        double *buf = (double *)malloc(sizeof(double) * SPIRAL_CHUNK_SIZE * 2);
        GenerateSpiralChunk(config->spiralState, buf, SPIRAL_CHUNK_SIZE);

        int32 numWritten = 0;
        OScDev_RichError *err = CreateDAQmxError(DAQmxWriteAnalogF64(
            config->aoTask, SPIRAL_CHUNK_SIZE, FALSE, 10.0,
            DAQmx_Val_GroupByChannel, buf, &numWritten, NULL));
        free(buf);
        if (err)
            return OScDev_Error_Wrap(err, "Failed to write scanner waveforms");
        if (numWritten != SPIRAL_CHUNK_SIZE)
            return OScDev_Error_Create(
                "Failed to write complete scan waveform");
        return OScDev_RichError_OK;
    }

    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);
    int32 totalElementsPerFramePerChan = GetScannerWaveformSize(&params);
    double *xyWaveformFrame =
        (double *)malloc(sizeof(double) * totalElementsPerFramePerChan * 2);
    GenerateGalvoWaveformFrame(&params, xyWaveformFrame);

    int32 numWritten = 0;
    OScDev_RichError *err = CreateDAQmxError(DAQmxWriteAnalogF64(
        config->aoTask, totalElementsPerFramePerChan, FALSE, 10.0,
        DAQmx_Val_GroupByChannel, xyWaveformFrame, &numWritten, NULL));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to write scanner waveforms");
        goto cleanup;
    }
    if (numWritten != totalElementsPerFramePerChan) {
        err = OScDev_Error_Create("Failed to write complete scan waveform");
        goto cleanup;
    }

cleanup:
    free(xyWaveformFrame);
    return err;
}

static DWORD WINAPI SpiralWriterThreadFunc(void *param) {
    struct ScannerConfig *config = (struct ScannerConfig *)param;

    double *buf = (double *)malloc(sizeof(double) * SPIRAL_CHUNK_SIZE * 2);

    while (!config->spiralStopRequested) {
        if (config->spiralTotalFrames < (uint32_t)INT32_MAX &&
            GetSpiralArmCycleIndex(config->spiralState) >=
                (int32_t)config->spiralTotalFrames) {
            break;
        }

        GenerateSpiralChunk(config->spiralState, buf, SPIRAL_CHUNK_SIZE);

        int32 numWritten = 0;
        int32 nierr = DAQmxWriteAnalogF64(
            config->aoTask, SPIRAL_CHUNK_SIZE, FALSE, DAQmx_Val_WaitInfinitely,
            DAQmx_Val_GroupByChannel, buf, &numWritten, NULL);
        if (nierr < 0)
            break;
    }

    free(buf);
    return 0;
}

static OScDev_RichError *StartSpiralStreaming(struct ScannerConfig *config) {
    config->spiralStopRequested = false;
    DWORD id;
    config->spiralWriterThread =
        CreateThread(NULL, 0, SpiralWriterThreadFunc, config, 0, &id);
    if (!config->spiralWriterThread)
        return OScDev_Error_Create("Failed to create spiral writer thread");
    return OScDev_RichError_OK;
}

static void StopSpiralStreaming(struct ScannerConfig *config) {
    if (!config->spiralWriterThread)
        return;

    config->spiralStopRequested = true;
    DAQmxStopTask(config->aoTask);

    WaitForSingleObject(config->spiralWriterThread, INFINITE);
    CloseHandle(config->spiralWriterThread);
    config->spiralWriterThread = NULL;

    if (config->spiralState) {
        DestroySpiralGenState(config->spiralState);
        config->spiralState = NULL;
    }
}

// Initialize, configure, and arm the scanner, whatever its current state
OScDev_RichError *SetUpScanner(OScDev_Device *device,
                               struct ScannerConfig *config,
                               OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    bool mustCommit = false;

    if (!config->aoTask) {
        err = CreateDAQmxError(DAQmxCreateTask("Scanner", &config->aoTask));
        if (err) {
            err = OScDev_Error_Wrap(err, "Failed to create scanner task");
            return err;
        }

        ss8str aoTerms;
        ss8_init_copy(&aoTerms, &GetImplData(device)->deviceName);
        ss8_cat_cstr(&aoTerms, "/ao0:1");
        err = CreateDAQmxError(DAQmxCreateAOVoltageChan(
            config->aoTask, ss8_cstr(&aoTerms), "Galvos", -10.0, 10.0,
            DAQmx_Val_Volts, NULL));
        ss8_destroy(&aoTerms);
        if (err) {
            err = OScDev_Error_Wrap(
                err, "Failed to create ao channels for scanner");
            goto error;
        }

        config->mustReconfigureTiming = true;
        config->mustRewriteOutput = true;
        mustCommit = true;
    }

    err = ConfigureScannerTiming(device, config, acq);
    if (err)
        goto error;
    config->mustReconfigureTiming = false;
    mustCommit = true;

    err = WriteScannerOutput(device, config, acq);
    if (err)
        goto error;
    config->mustRewriteOutput = false;
    mustCommit = true;

    if (mustCommit) {
        err = CreateDAQmxError(
            DAQmxTaskControl(config->aoTask, DAQmx_Val_Task_Commit));
        if (err) {
            err = OScDev_Error_Wrap(err, "Failed to commit task for scanner");
            goto error;
        }
    }

    return OScDev_RichError_OK;

error:
    if (ShutdownScanner(config))
        OScDev_Log_Error(device,
                         "Failed to clean up scanner task after error");
    return err;
}

// Remove all DAQmx configuration for the scanner
OScDev_RichError *ShutdownScanner(struct ScannerConfig *config) {
    OScDev_RichError *err;
    if (config->aoTask) {
        err = CreateDAQmxError(DAQmxClearTask(config->aoTask));
        if (err) {
            err = OScDev_Error_Wrap(err, "Failed to clear scanner task");
            return err;
        }
        config->aoTask = 0;
    }
    return OScDev_RichError_OK;
}

OScDev_RichError *StartScanner(OScDev_Device *device,
                               struct ScannerConfig *config) {
    OScDev_RichError *err;
    err = CreateDAQmxError(DAQmxStartTask(config->aoTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to start scanner task");
        ShutdownScanner(config); // Force re-setup next time
        return err;
    }

    if (GetImplData(device)->spiralScanEnabled) {
        err = StartSpiralStreaming(config);
        if (err) {
            DAQmxStopTask(config->aoTask);
            ShutdownScanner(config);
            return err;
        }
    }

    return OScDev_RichError_OK;
}

OScDev_RichError *StopScanner(OScDev_Device *device,
                              struct ScannerConfig *config) {
    if (GetImplData(device)->spiralScanEnabled) {
        StopSpiralStreaming(config);
        return OScDev_RichError_OK;
    }

    OScDev_RichError *err;
    err = CreateDAQmxError(DAQmxStopTask(config->aoTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to stop scanner task");
        ShutdownScanner(config); // Force re-setup next time
        return err;
    }
    return OScDev_RichError_OK;
}

OScDev_RichError *CreateScannerTask(OScDev_Device *device,
                                    struct ScannerConfig *config) {
    if (!(config)->aoTask) {
        OScDev_RichError *err;
        err = CreateDAQmxError(DAQmxCreateTask("Scanner", &config->aoTask));
        if (err) {
            err = OScDev_Error_Wrap(err, "Failed to create scanner task");
            return err;
        }

        ss8str aoTerms;
        ss8_init_copy(&aoTerms, &GetImplData(device)->deviceName);
        ss8_cat_cstr(&aoTerms, "/ao0:1");
        err = CreateDAQmxError(DAQmxCreateAOVoltageChan(
            config->aoTask, ss8_cstr(&aoTerms), "Galvos", -10.0, 10.0,
            DAQmx_Val_Volts, NULL));
        ss8_destroy(&aoTerms);
        if (err) {
            err = OScDev_Error_Wrap(
                err, "Failed to create ao channels for scanner");
            if (ShutdownScanner(config))
                OScDev_Log_Error(
                    device, "Failed to clean up scanner task after error");
            return err;
        }
    }
    return OScDev_RichError_OK;
}
