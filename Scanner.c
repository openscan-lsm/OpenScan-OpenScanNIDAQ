#include "Scanner.h"

#include "OScNIDAQ.h"
#include "OScNIDAQPrivateData.h"
#include "Waveform.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>
#include <ss8str.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

static OScDev_RichError *ConfigureScannerTiming(OScDev_Device *device,
                                                struct ScannerConfig *config,
                                                OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);

    int32 totalElementsPerFramePerChan = GetScannerWaveformSize(&params);

    err = CreateDAQmxError(DAQmxCfgSampClkTiming(
        config->aoTask, "", pixelRateHz, DAQmx_Val_Rising,
        DAQmx_Val_FiniteSamps, totalElementsPerFramePerChan));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to configure timing for scanner");
        return err;
    }

    return OScDev_RichError_OK;
}

static OScDev_RichError *WriteScannerOutput(OScDev_Device *device,
                                            struct ScannerConfig *config,
                                            OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);

    int32 totalElementsPerFramePerChan = GetScannerWaveformSize(&params);
    double *xyWaveformFrame =
        (double *)malloc(sizeof(double) * totalElementsPerFramePerChan * 2);

    err = GenerateGalvoWaveformFrame(&params, xyWaveformFrame);
    if (err)
        return err;

    int32 numWritten = 0;
    err = CreateDAQmxError(DAQmxWriteAnalogF64(
        config->aoTask, totalElementsPerFramePerChan, FALSE, 10.0,
        DAQmx_Val_GroupByChannel, xyWaveformFrame, &numWritten, NULL));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to write scanner waveforms");
        goto cleanup;
    }
    if (numWritten != totalElementsPerFramePerChan) {
        err = OScDev_Error_Wrap(err, "Failed to write complete scan waveform");
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
        ss8_init_copy(&aoTerms, &GetData(device)->deviceName);
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
        ss8_init_copy(&aoTerms, &GetData(device)->deviceName);
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
