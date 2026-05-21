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

// Append the scanner AO channel range ("<dev>/ao0:1" or "<dev>/ao0:2") to
// terms
static void AppendScannerAOTerms(OScDev_Device *device, ss8str *terms) {
    ss8_cat(terms, &GetImplData(device)->deviceName);
    int nch = GetNumberOfScannerAOChannels(device);
    ss8_cat_cstr(terms, nch >= 3 ? "/ao0:2" : "/ao0:1");
}

static OScDev_RichError *ConfigureScannerTiming(OScDev_Device *device,
                                                struct ScannerConfig *config,
                                                OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    double aoRateHz = GetImplData(device)->aoRateHz;
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

    err = CreateDAQmxError(DAQmxCfgSampClkTiming(config->aoTask, "", aoRateHz,
                                                 DAQmx_Val_Rising, sampleMode,
                                                 samplesPerChan));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to configure timing for scanner");
        return err;
    }

    err = CreateDAQmxError(
        DAQmxSetWriteRegenMode(config->aoTask, DAQmx_Val_AllowRegen));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to set regen mode for scanner");
        return err;
    }

    return OScDev_RichError_OK;
}

static OScDev_RichError *WriteScannerOutput(OScDev_Device *device,
                                            struct ScannerConfig *config,
                                            OScDev_Acquisition *acq) {
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);

    int nch = GetNumberOfScannerAOChannels(device);
    int32 totalElementsPerFramePerChan = GetScannerWaveformSize(&params);
    double *xyWaveformFrame =
        (double *)malloc(sizeof(double) * totalElementsPerFramePerChan * nch);

    GenerateGalvoWaveformFrame(&params, xyWaveformFrame);
    if (nch == 3)
        GenerateLaserBlankingWaveform(
            &params, xyWaveformFrame + 2 * totalElementsPerFramePerChan);

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
        ss8_init(&aoTerms);
        AppendScannerAOTerms(device, &aoTerms);
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

OScDev_RichError *StartScanner(struct ScannerConfig *config) {
    OScDev_RichError *err;
    err = CreateDAQmxError(DAQmxStartTask(config->aoTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to start scanner task");
        ShutdownScanner(config); // Force re-setup next time
        return err;
    }
    return OScDev_RichError_OK;
}

OScDev_RichError *StopScanner(struct ScannerConfig *config) {
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
        ss8_init(&aoTerms);
        AppendScannerAOTerms(device, &aoTerms);
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

OScDev_RichError *ApplyIdleLaserOutput(OScDev_Device *device) {
    if (!LaserBlankingSupported(device))
        return OScDev_RichError_OK;

    CRITICAL_SECTION *mutex = &GetImplData(device)->acquisition.mutex;
    EnterCriticalSection(mutex);
    if (GetImplData(device)->acquisition.running) {
        LeaveCriticalSection(mutex);
        return OScDev_RichError_OK;
    }

    struct ScannerConfig *config = &GetImplData(device)->scannerConfig;
    OScDev_RichError *err;

    // Recreate the task to drop any leftover sample-clock timing from a prior
    // scan; the on-demand task has no timing configured.
    err = ShutdownScanner(config);
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to reset scanner task for idle "
                                     "laser output");
        goto cleanup;
    }
    err = CreateScannerTask(device, config);
    if (err)
        goto cleanup;

    float64 buf[3] = {
        GetImplData(device)->prevXParkVoltage,
        GetImplData(device)->prevYParkVoltage,
        GetImplData(device)->laserManualOn
            ? GetImplData(device)->laserOnVoltage
            : GetImplData(device)->laserOffVoltage,
    };

    int32 numWritten = 0;
    err = CreateDAQmxError(DAQmxWriteAnalogF64(config->aoTask, 1, TRUE, 10.0,
                                               DAQmx_Val_GroupByChannel, buf,
                                               &numWritten, NULL));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to write idle laser output");
        ShutdownScanner(config);
        goto cleanup;
    }

    err = CreateDAQmxError(DAQmxStopTask(config->aoTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to stop idle laser output task");
        ShutdownScanner(config);
        goto cleanup;
    }

cleanup:
    LeaveCriticalSection(mutex);
    return err;
}
