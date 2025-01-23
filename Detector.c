#include "Detector.h"

#include "OScNIDAQ.h"
#include "OScNIDAQPrivateData.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>
#include <ss8str.h>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static OScDev_RichError *
GetAIVoltageRange(OScDev_Device *device, double *minVolts, double *maxVolts) {
    OScDev_RichError *err = OScDev_RichError_OK;
    float64 ranges[2 * 64];
    memset(ranges, 0, sizeof(ranges));

    // TODO How does this relate to the setting "Input Voltage Range"?
    // BUG: This should be AIVoltageRngs, but keeping existing behavior for now
    err = CreateDAQmxError(
        DAQmxGetDevAOVoltageRngs(ss8_cstr(&GetImplData(device)->deviceName),
                                 ranges, sizeof(ranges) / sizeof(float64)));
    if (err) {
        OScDev_Log_Error(device, OScDev_Error_GetMessage(err));
        return err;
    }

    *minVolts = INFINITY;
    *maxVolts = -INFINITY;
    for (int i = 0; i < sizeof(ranges) / sizeof(float64) / 2; ++i) {
        if (ranges[2 * i] == 0.0 && ranges[2 * i + 1] == 0.0) {
            if (i == 0)
                return OScDev_Error_Create(
                    "AI channel voltage range appears to be empty");
            break;
        }

        // Find the range with the highest max voltage
        // FIXME This is not robust: there may be multiple ranges with the same
        // maximum but different minima, which would change the sample value
        // ranges
        if (ranges[2 * i + 1] > *maxVolts) {
            *minVolts = ranges[2 * i];
            *maxVolts = ranges[2 * i + 1];
        }
    }

    return err;
}

// Process data in rawDataBuffer and place the result into frameBuffers
static int32 HandleRawData(OScDev_Device *device) {
    // Some amount of data (rawDataSize samples) is in the rawDataBuffer.
    // Since we have C channels and are binning every B samples per channel,
    // we can only process a multiple of (C * B) samples at a time. We
    // shift any remaining samples to the front of rawDataBuffer so that they
    // can be handled when more data is available.

    size_t availableSamples = GetImplData(device)->rawDataSize;
    uint32_t numChannels = GetNumberOfEnabledChannels(device);
    size_t leftoverSamples = availableSamples % numChannels;
    size_t samplesToProcess = availableSamples - leftoverSamples;
    size_t pixelsToProducePerChan = samplesToProcess / numChannels;

    double inputVoltageRange = GetImplData(device)->inputVoltageRange;

    float64 *rawDataBuffer = GetImplData(device)->rawDataBuffer;

    // Given 2 channels and 2 samples per pixel per channel, rawDataBuffer
    // contains data in the following order:
    // | ch0_samp0 ch1_samp0 ch0_samp1 ch1_samp1 | ch0_samp0 ...
    // We need to transfer this into per-channel frame buffers.

    // Process raw data and fill in frame buffers
    for (size_t p = 0; p < pixelsToProducePerChan; ++p) {
        size_t rawPixelStart = p * numChannels;
        size_t pixelIndex = GetImplData(device)->framePixelsFilled++;

        for (size_t ch = 0; ch < numChannels; ++ch) {
            size_t rawChannelStart = rawPixelStart + ch;

            double volts = rawDataBuffer[rawChannelStart];

            // TODO We need a positive offset so as not to clip the background
            // noise
            double offsetVolts = 1.0; // Temporary

            double dpixel =
                65535.0 * (volts + offsetVolts) / inputVoltageRange;
            if (dpixel < 0) {
                dpixel = 0.0;
            }
            if (dpixel > 65535.0) {
                dpixel = 65535.0;
            }
            uint16_t pixel = (uint16_t)dpixel;

            GetImplData(device)->frameBuffers[ch][pixelIndex] = pixel;
        }
    }

    // Shift the leftover raw samples to the front of the buffer for future
    // consumption
    memmove(rawDataBuffer, rawDataBuffer + samplesToProcess,
            sizeof(float64) * leftoverSamples);
    GetImplData(device)->rawDataSize = leftoverSamples;

    // TODO Cleaner to get raster size from the OScDev_Acquisition (a future
    // OpenScanLib should allow getting the current device from the
    // acquisition, so that we can pass the acquisition as callback data)
    uint32_t pixelsPerLine = GetImplData(device)->configuredRasterWidth;
    uint32_t linesPerFrame = GetImplData(device)->configuredRasterHeight;

    size_t pixelsPerFrame = pixelsPerLine * linesPerFrame;
    char msg[OScDev_MAX_STR_LEN + 1];
    snprintf(msg, OScDev_MAX_STR_LEN, "Read %zd pixels",
             GetImplData(device)->framePixelsFilled);
    OScDev_Log_Debug(device, msg);

    if (GetImplData(device)->framePixelsFilled == pixelsPerFrame) {
        // TODO This method of communication is unreliable without a mutex
        // TODO But we should use a condition variable in any case
        GetImplData(device)->oneFrameScanDone = true;

        // TODO This reset should occur at start of frame
        GetImplData(device)->framePixelsFilled = 0;
    }

    return OScDev_OK;
}

static int32 DetectorDataCallback(TaskHandle taskHandle,
                                  int32 everyNsamplesEventType,
                                  uInt32 nSamples, void *callbackData) {
    OScDev_Error errCode;
    OScDev_Device *device = (OScDev_Device *)(callbackData);

    if (taskHandle != GetImplData(device)->detectorConfig.aiTask)
        return OScDev_OK;
    if (everyNsamplesEventType != DAQmx_Val_Acquired_Into_Buffer)
        return OScDev_OK;

    char msg[1024];
    snprintf(msg, sizeof(msg) - 1, "Detector callback (%d samples)", nSamples);
    OScDev_Log_Debug(device, msg);

    uint32_t numChannels = GetNumberOfEnabledChannels(device);

    OScDev_RichError *err;

    // TODO We should use DAQmxReadBinaryU16, so that users have access to raw
    // ADC values. This is often critical for correct interpretation or
    // processing of the data.

    // Read all available samples without waiting (because we have set the
    // Read All Available Samples property on the task)
    int32 samplesPerChanRead;
    errCode = DAQmxReadAnalogF64(
        taskHandle, DAQmx_Val_Auto, 0.0, DAQmx_Val_GroupByScanNumber,
        GetImplData(device)->rawDataBuffer + GetImplData(device)->rawDataSize,
        (uInt32)(GetImplData(device)->rawDataCapacity -
                 GetImplData(device)->rawDataSize),
        &samplesPerChanRead, NULL);
    if (errCode == DAQmxErrorTimeoutExceeded) {
        OScDev_Log_Error(device, "Error: DAQ read data timeout");
        return OScDev_OK;
    }

    if (errCode) {
        err = CreateDAQmxError(errCode);
        err = OScDev_Error_Wrap(err, "Failed to read detector samples");
        goto error;
    }

    if (samplesPerChanRead == 0) {
        OScDev_Log_Error(device, "Error: DAQ failed to read any sample");
        return OScDev_OK;
    }

    GetImplData(device)->rawDataSize += samplesPerChanRead * numChannels;

    errCode = HandleRawData(device);
    if (errCode)
        goto error;

    return OScDev_OK;

error:
    if (GetImplData(device)->detectorConfig.aiTask)
        ShutdownDetector(&GetImplData(device)->detectorConfig);
    return errCode;
}

static OScDev_RichError *CreateDetectorTask(OScDev_Device *device,
                                            struct DetectorConfig *config) {
    OScDev_RichError *err = OScDev_RichError_OK;
    err = CreateDAQmxError(DAQmxCreateTask("Detector", &config->aiTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to create detector task");
        return err;
    }

    double minVolts, maxVolts;
    err = GetAIVoltageRange(device, &minVolts, &maxVolts);
    if (err)
        goto error;

    ss8str aiChans;
    ss8_init(&aiChans);
    GetEnabledChannels(device, &aiChans);
    err = CreateDAQmxError(DAQmxCreateAIVoltageChan(
        config->aiTask, ss8_cstr(&aiChans), "", DAQmx_Val_Cfg_Default,
        minVolts, maxVolts, DAQmx_Val_Volts, NULL));
    ss8_destroy(&aiChans);
    if (err) {
        err =
            OScDev_Error_Wrap(err, "Failed to create ai channel for detector");
        goto error;
    }

    return OScDev_RichError_OK;

error:
    if (ShutdownDetector(config))
        OScDev_Log_Error(device,
                         "Failed to clean up detector task after error");
    return err;
}

static OScDev_RichError *ConfigureDetectorTiming(struct DetectorConfig *config,
                                                 OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    uint32_t xOffset, yOffset, width, height;
    OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);

    err = CreateDAQmxError(
        DAQmxCfgSampClkTiming(config->aiTask, "", pixelRateHz,
                              DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, width));
    if (err) {
        err =
            OScDev_Error_Wrap(err, "Failed to configure timing for detector");
        return err;
    }

    return OScDev_RichError_OK;
}

static OScDev_RichError *
ConfigureDetectorTrigger(OScDev_Device *device,
                         struct DetectorConfig *config) {
    // For now we hard-code the trigger to PFI12, which is the output of CTR0

    // Alternative: virtually connect counter (line clock) output terminal and
    // acquisition triggerIn terminal without physical wiring:
    // nierr = DAQmxConnectTerms("/PXI1Slot2/Ctr0InternalOutput",
    // "/PXI1Slot2/PFI8", DAQmx_Val_DoNotInvertPolarity); nierr =
    // DAQmxCfgDigEdgeStartTrig(acqTaskHandle_, "/PXI1Slot2/PFI8",
    // DAQmx_Val_Rising);
    OScDev_RichError *err;

    ss8str trigSrc;
    ss8_init_copy_ch(&trigSrc, '/');
    ss8_cat(&trigSrc, &GetImplData(device)->deviceName);
    ss8_cat_cstr(&trigSrc, "/PFI12");
    err = CreateDAQmxError(DAQmxCfgDigEdgeStartTrig(
        config->aiTask, ss8_cstr(&trigSrc), DAQmx_Val_Rising));
    ss8_destroy(&trigSrc);
    if (err) {
        err = OScDev_Error_Wrap(
            err, "Failed to set start trigger for detector task");
        return err;
    }

    err = CreateDAQmxError(DAQmxSetStartTrigRetriggerable(config->aiTask, 1));
    if (err) {
        err = OScDev_Error_Wrap(err,
                                "Failed to set detector task retriggerable");
        return err;
    }
    return OScDev_RichError_OK;
}

static OScDev_RichError *
UnconfigureDetectorCallback(struct DetectorConfig *config) {
    OScDev_RichError *err;
    err = CreateDAQmxError(DAQmxRegisterEveryNSamplesEvent(
        config->aiTask, DAQmx_Val_Acquired_Into_Buffer, 0, 0, NULL, NULL));
    if (err) {
        err = OScDev_Error_Wrap(err,
                                "Failed to unregister callback for detector");
        return err;
    }
    return OScDev_RichError_OK;
}

static OScDev_RichError *
ConfigureDetectorCallback(OScDev_Device *device, struct DetectorConfig *config,
                          OScDev_Acquisition *acq) {
    uint32_t xOffset, yOffset, width, height;
    OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);

    OScDev_RichError *err;

    // Registering the callback in DAQmx is not idempotent, so we need to
    // clear any existing callback.
    err = UnconfigureDetectorCallback(config);
    if (err)
        return err;

    // TODO: It probably makes sense to scale the buffer based on the time
    // duration of data it holds (e.g. 500 ms), rather than the number of
    // lines. We could still provide a user-settable scaling factor.

    uint32_t pixelsPerLine = width;
    uint32_t pixelsPerFrame = pixelsPerLine * height;
    uint32_t samplesPerChanPerLine = pixelsPerLine;
    uint32_t numChannels = GetNumberOfEnabledChannels(device);
    size_t bufferSize = GetImplData(device)->numLinesToBuffer *
                        samplesPerChanPerLine * numChannels;

    char msg[1024];
    snprintf(msg, sizeof(msg) - 1, "Using DAQmx input buffer of size %zd",
             bufferSize);
    OScDev_Log_Debug(device, msg);

    err = CreateDAQmxError(
        DAQmxCfgInputBuffer(config->aiTask, (uInt32)bufferSize));
    if (err) {
        err = OScDev_Error_Wrap(
            err, "Failed to configure input buffer for detector");
        return err;
    }

    // Allocate buffer into which we read data. Set it to be large enough
    // to read all available data from the input buffer in one go.
    GetImplData(device)->rawDataCapacity = bufferSize;
    GetImplData(device)->rawDataSize = 0;
    GetImplData(device)->rawDataBuffer =
        realloc(GetImplData(device)->rawDataBuffer,
                sizeof(float64) * GetImplData(device)->rawDataCapacity);

    // Allocate frame buffers for the enabled channels
    for (uint32_t ch = 0; ch < numChannels; ++ch) {
        GetImplData(device)->frameBuffers[ch] =
            realloc(GetImplData(device)->frameBuffers[ch],
                    sizeof(uint16_t) * pixelsPerFrame);
    }
    // Free the frame buffers for unused channels
    for (uint32_t ch = numChannels; ch < MAX_PHYSICAL_CHANS; ++ch) {
        free(GetImplData(device)->frameBuffers[ch]);
        GetImplData(device)->frameBuffers[ch] = NULL;
    }

    // Set DAQmxRead*() with DAQmx_Val_Auto to immediately return all
    // available samples instead of waiting for the requested number of
    // samples to become available.
    err = CreateDAQmxError(DAQmxSetReadReadAllAvailSamp(config->aiTask, TRUE));
    if (err) {
        err = OScDev_Error_Wrap(
            err,
            "Failed to set the Read All Available Samples property for the detector task");
        return err;
    }

    // TODO: It probably makes sense to scale the callback frequency so that
    // it is called at roughly 10 Hz. For now it is called once per line.

    err = CreateDAQmxError(DAQmxRegisterEveryNSamplesEvent(
        config->aiTask, DAQmx_Val_Acquired_Into_Buffer, samplesPerChanPerLine,
        0, DetectorDataCallback, device));
    if (err) {
        err =
            OScDev_Error_Wrap(err, "Failed to register callback for detector");
        err =
            OScDev_Error_Wrap(err, "Failed to register callback for detector");
        return err;
    }

    return OScDev_RichError_OK;
}

// Initialize, configure, and arm the detector, whatever its current state
OScDev_RichError *SetUpDetector(OScDev_Device *device,
                                struct DetectorConfig *config,
                                OScDev_Acquisition *acq) {
    OScDev_RichError *err = OScDev_RichError_OK;
    bool mustCommit = false;

    if (!config->aiTask) {
        err = CreateDetectorTask(device, config);
        if (err)
            return err;

        config->mustReconfigureTiming = true;
        config->mustReconfigureTrigger = true;
        config->mustReconfigureCallback = true;
        mustCommit = true;
    }

    if (config->mustReconfigureTiming) {
        err = ConfigureDetectorTiming(config, acq);
        if (err)
            goto error;
        config->mustReconfigureTiming = false;
        mustCommit = true;
    }

    if (config->mustReconfigureTrigger) {
        err = ConfigureDetectorTrigger(device, config);
        if (err)
            goto error;
        config->mustReconfigureTrigger = false;
        mustCommit = true;
    }

    if (config->mustReconfigureCallback) {
        err = ConfigureDetectorCallback(device, config, acq);
        if (err)
            goto error;
        config->mustReconfigureCallback = false;
        mustCommit = true;
    }

    if (mustCommit) {
        err = CreateDAQmxError(
            DAQmxTaskControl(config->aiTask, DAQmx_Val_Task_Commit));
        if (err) {
            err = OScDev_Error_Wrap(err, "Failed to commit task for detector");
            goto error;
        }
    }

    return OScDev_RichError_OK;

error:
    if (ShutdownDetector(config))
        OScDev_Log_Error(device,
                         "Failed to clean up detector task after error");
    return err;
}

// Remove all DAQmx configuration for the detector
// This can be called to force task recreation the next time the detector is
// armed
OScDev_RichError *ShutdownDetector(struct DetectorConfig *config) {
    OScDev_RichError *err;
    if (config->aiTask) {
        err = CreateDAQmxError(DAQmxClearTask(config->aiTask));
        if (err) {
            err = OScDev_Error_Wrap(err, "Failed to clear detector task");
            return err;
        }
        config->aiTask = 0;
    }
    return OScDev_RichError_OK;
}

OScDev_RichError *StartDetector(struct DetectorConfig *config) {
    OScDev_RichError *err;
    err = CreateDAQmxError(DAQmxStartTask(config->aiTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to start detector task");
        ShutdownDetector(config); // Force re-setup next time
        return err;
    }
    return OScDev_RichError_OK;
}

OScDev_RichError *StopDetector(struct DetectorConfig *config) {
    OScDev_RichError *err;
    // The task may have been cleared in the case of an error
    if (!config->aiTask)
        return OScDev_RichError_OK;

    err = CreateDAQmxError(DAQmxStopTask(config->aiTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to stop detector task");
        ShutdownDetector(config); // Force re-setup next time
        return err;
    }
    return OScDev_RichError_OK;
}
