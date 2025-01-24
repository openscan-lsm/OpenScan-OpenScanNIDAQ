#include "ParkUnpark.h"

#include "NIDAQ_DeviceImplData.h"
#include "OScNIDAQ.h"
#include "Scanner.h"
#include "Waveform.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

OScDev_RichError *ConfigureUnparkTiming(OScDev_Device *device,
                                        struct ScannerConfig *config,
                                        OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);

    int32 totalElementsPerFramePerChan = GetParkWaveformSize(&params);

    err = CreateDAQmxError(DAQmxCfgSampClkTiming(
        config->aoTask, "", pixelRateHz, DAQmx_Val_Rising,
        DAQmx_Val_FiniteSamps, totalElementsPerFramePerChan));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to configure timing for unpark");
        return err;
    }

    return OScDev_RichError_OK;
}

OScDev_RichError *ConfigureParkTiming(OScDev_Device *device,
                                      struct ScannerConfig *config,
                                      OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);

    int32 totalElementsPerFramePerChan = GetParkWaveformSize(&params);

    err = CreateDAQmxError(DAQmxCfgSampClkTiming(
        config->aoTask, "", pixelRateHz, DAQmx_Val_Rising,
        DAQmx_Val_FiniteSamps, totalElementsPerFramePerChan));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to configure timing for park");
        return err;
    }

    return OScDev_RichError_OK;
}

OScDev_RichError *WriteUnparkOutput(OScDev_Device *device,
                                    struct ScannerConfig *config,
                                    OScDev_Acquisition *acq) {
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);

    int32 totalElementsPerFramePerChan = GetParkWaveformSize(&params);
    double *xyWaveformFrame =
        (double *)malloc(sizeof(double) * totalElementsPerFramePerChan * 2);

    GenerateGalvoUnparkWaveform(&params, xyWaveformFrame);

    int32 numWritten = 0;
    OScDev_RichError *err = CreateDAQmxError(DAQmxWriteAnalogF64(
        config->aoTask, totalElementsPerFramePerChan, FALSE, 10.0,
        DAQmx_Val_GroupByChannel, xyWaveformFrame, &numWritten, NULL));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to write unpark waveforms");
        goto cleanup;
    }
    if (numWritten != totalElementsPerFramePerChan) {
        err =
            OScDev_Error_Wrap(err, "Failed to write complete unpark waveform");
        goto cleanup;
    }

cleanup:
    free(xyWaveformFrame);
    return err;
}

OScDev_RichError *WriteParkOutput(OScDev_Device *device,
                                  struct ScannerConfig *config,
                                  OScDev_Acquisition *acq) {
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);

    int32 totalElementsPerFramePerChan = GetParkWaveformSize(&params);
    double *xyWaveformFrame =
        (double *)malloc(sizeof(double) * totalElementsPerFramePerChan * 2);

    GenerateGalvoParkWaveform(&params, xyWaveformFrame);
    GetImplData(device)->prevXParkVoltage =
        xyWaveformFrame[totalElementsPerFramePerChan - 1];
    GetImplData(device)->prevYParkVoltage =
        xyWaveformFrame[(totalElementsPerFramePerChan * 2) - 1];

    int32 numWritten = 0;
    OScDev_RichError *err = CreateDAQmxError(DAQmxWriteAnalogF64(
        config->aoTask, totalElementsPerFramePerChan, FALSE, 10.0,
        DAQmx_Val_GroupByChannel, xyWaveformFrame, &numWritten, NULL));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to write park waveforms");
        goto cleanup;
    }
    if (numWritten != totalElementsPerFramePerChan) {
        err = OScDev_Error_Wrap(err, "Failed to write complete park waveform");
        goto cleanup;
    }

cleanup:
    free(xyWaveformFrame);
    return err;
}

OScDev_RichError *GenerateUnparkOutput(OScDev_Device *device,
                                       struct ScannerConfig *config,
                                       OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);
    uint32_t totalElementsPerFramePerChan = GetParkWaveformSize(&params);
    // changed from 1e3 to 1e4 - works but does not reconfigure timing
    uint32_t estFrameTimeMs =
        (uint32_t)(1e3 * totalElementsPerFramePerChan / pixelRateHz);
    uint32_t maxWaitTimeMs = 2 * estFrameTimeMs;
    if (maxWaitTimeMs < 1000) {
        maxWaitTimeMs = 1000;
    }

    // start task
    err = CreateDAQmxError(DAQmxStartTask(config->aoTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to start unpark task");
        ShutdownScanner(config); // Force re-setup next time
        return err;
    }

    // Wait for scan to complete
    err = CreateDAQmxError(DAQmxWaitUntilTaskDone(
        GetImplData(device)->scannerConfig.aoTask, maxWaitTimeMs * 1e-3));
    if (err) {
        err =
            OScDev_Error_Wrap(err, "Failed to wait for unpark task to finish");
        return err;
    }

    err = CreateDAQmxError(DAQmxStopTask(config->aoTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to stop unpark task");
        ShutdownScanner(config); // Force re-setup next time
        return err;
    }

    return OScDev_RichError_OK;
}

OScDev_RichError *GenerateParkOutput(OScDev_Device *device,
                                     struct ScannerConfig *config,
                                     OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);
    uint32_t totalElementsPerFramePerChan = GetParkWaveformSize(&params);
    uint32_t estFrameTimeMs =
        (uint32_t)(1e3 * totalElementsPerFramePerChan / pixelRateHz);
    uint32_t maxWaitTimeMs = 2 * estFrameTimeMs;
    if (maxWaitTimeMs < 1000) {
        maxWaitTimeMs = 1000;
    }

    // start task
    err = CreateDAQmxError(DAQmxStartTask(config->aoTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to start park task");
        ShutdownScanner(config); // Force re-setup next time
        return err;
    }

    // Wait for scan to complete
    err = CreateDAQmxError(DAQmxWaitUntilTaskDone(
        GetImplData(device)->scannerConfig.aoTask, maxWaitTimeMs * 1e-3));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to wait for park task to finish");
        return err;
    }

    err = CreateDAQmxError(DAQmxStopTask(config->aoTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to stop park task");
        ShutdownScanner(config); // Force re-setup next time
        return err;
    }

    return OScDev_RichError_OK;
}
