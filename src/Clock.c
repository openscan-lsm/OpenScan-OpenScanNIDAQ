#include "Clock.h"

#include "DAQConfig.h"
#include "DAQError.h"
#include "DeviceImplData.h"
#include "Waveform.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>
#include <ss8str.h>

#include <stdint.h>
#include <stdlib.h>

static OScDev_RichError *CreateClockTasks(OScDev_Device *device,
                                          struct ClockConfig *config,
                                          OScDev_Acquisition *acq) {
    OScDev_RichError *err;
    err = CreateDAQmxError(DAQmxCreateTask("ClockDO", &config->doTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to create clock do task");
        return err;
    }

    // P0.5 = line clock
    // P0.6 = inverted line clock (for FLIM)
    // P0.7 = frame clock
    // This needs to be port0 to support buffered output

    ss8str doTerms;
    ss8_init_copy(&doTerms, &GetImplData(device)->deviceName);
    ss8_cat_cstr(&doTerms, "/port0/line5:7");
    err = CreateDAQmxError(DAQmxCreateDOChan(
        config->doTask, ss8_cstr(&doTerms), "ClockDO", DAQmx_Val_ChanPerLine));
    ss8_destroy(&doTerms);
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to create clock do channel");
        return err;
    }

    err = CreateDAQmxError(DAQmxGetReadNumChans(
        config->doTask, &GetImplData(device)->numDOChannels));
    if (err) {
        err = OScDev_Error_Wrap(
            err, "Failed to get number of channels from clock do task");
        return err;
    }

    err = CreateDAQmxError(DAQmxCreateTask("ClockCtr", &config->lineCtrTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to create clock lineCtr task");
        return err;
    }

    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    uint32_t xOffset, yOffset, width, height;
    OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);
    uint32_t elementsPerLine = GetLineWaveformSize(&params);
    double effectiveScanPortion = (double)width / elementsPerLine;
    double lineFreqHz = pixelRateHz / elementsPerLine;
    double scanPhase = 1.0 / pixelRateHz * GetImplData(device)->lineDelay;

    ss8str ctrTerms;
    ss8_init_copy(&ctrTerms, &GetImplData(device)->deviceName);
    ss8_cat_cstr(&ctrTerms, "/ctr0");
    err = CreateDAQmxError(DAQmxCreateCOPulseChanFreq(
        config->lineCtrTask, ss8_cstr(&ctrTerms), "ClockLineCTR", DAQmx_Val_Hz,
        DAQmx_Val_Low, scanPhase, lineFreqHz, effectiveScanPortion));
    ss8_destroy(&ctrTerms);
    if (err) {
        err =
            OScDev_Error_Wrap(err, "Failed to create clock co pulse channel");
        return err;
    }

    return OScDev_RichError_OK;
}

static OScDev_RichError *ConfigureClockTiming(OScDev_Device *device,
                                              struct ClockConfig *config,
                                              OScDev_Acquisition *acq) {
    OScDev_RichError *err;

    double pixelRateHz = OScDev_Acquisition_GetPixelRate(acq);
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);
    uint32_t xOffset, yOffset, width, height;
    OScDev_Acquisition_GetROI(acq, &xOffset, &yOffset, &width, &height);

    uint32_t elementsPerLine = GetLineWaveformSize(&params);
    int32 elementsPerFramePerChan = GetClockWaveformSize(&params);

    err = CreateDAQmxError(DAQmxCfgSampClkTiming(
        config->doTask, "", pixelRateHz, DAQmx_Val_Rising,
        DAQmx_Val_FiniteSamps, elementsPerFramePerChan));
    if (err) {
        err = OScDev_Error_Wrap(
            err, "Failed to configure timing for clock do task");
        return err;
    }

    double effectiveScanPortion = (double)width / elementsPerLine;
    double lineFreqHz = pixelRateHz / elementsPerLine;
    double scanPhase = 1.0 / pixelRateHz * GetImplData(device)->lineDelay;

    err = CreateDAQmxError(DAQmxSetChanAttribute(
        config->lineCtrTask, "", DAQmx_CO_Pulse_Freq, lineFreqHz));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to set clock lineCtr frequency");
        return err;
    }

    err = CreateDAQmxError(DAQmxSetChanAttribute(
        config->lineCtrTask, "", DAQmx_CO_Pulse_Freq_InitialDelay, scanPhase));
    if (err) {
        err = OScDev_Error_Wrap(err,
                                "Failed to set clock lineCtr initial delay");
        return err;
    }

    err = CreateDAQmxError(DAQmxSetChanAttribute(config->lineCtrTask, "",
                                                 DAQmx_CO_Pulse_DutyCyc,
                                                 effectiveScanPortion));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to set clock lineCtr duty cycle");
        return err;
    }

    err = CreateDAQmxError(DAQmxCfgImplicitTiming(
        config->lineCtrTask, DAQmx_Val_FiniteSamps, height));
    if (err) {
        err = OScDev_Error_Wrap(
            err, "Failed to configure timing for clock lineCtr");
        return err;
    }

    return OScDev_RichError_OK;
}

static OScDev_RichError *ConfigureClockTriggers(OScDev_Device *device,
                                                struct ClockConfig *config) {
    OScDev_RichError *err;

    ss8str trigSrc;
    ss8_init_copy_ch(&trigSrc, '/');
    ss8_cat(&trigSrc, &GetImplData(device)->deviceName);
    ss8_cat_cstr(&trigSrc, "/ao/StartTrigger");
    err = CreateDAQmxError(DAQmxCfgDigEdgeStartTrig(
        config->doTask, ss8_cstr(&trigSrc), DAQmx_Val_Rising));
    if (err) {
        ss8_destroy(&trigSrc);
        err = OScDev_Error_Wrap(
            err, "Failed to configure trigger for clock do task");
        return err;
    }

    err = CreateDAQmxError(DAQmxSetStartTrigRetriggerable(config->doTask, 1));
    if (err) {
        ss8_destroy(&trigSrc);
        err = OScDev_Error_Wrap(err,
                                "Failed to set retriggerable clock do task");
        return err;
    }

    err = CreateDAQmxError(DAQmxCfgDigEdgeStartTrig(
        config->lineCtrTask, ss8_cstr(&trigSrc), DAQmx_Val_Rising));
    if (err) {
        ss8_destroy(&trigSrc);
        err = OScDev_Error_Wrap(
            err, "Failed to configure trigger for clock lineCtr task");
        return err;
    }

    ss8_destroy(&trigSrc);
    return OScDev_RichError_OK;
}

static OScDev_RichError *WriteClockOutput(OScDev_Device *device,
                                          struct ClockConfig *config,
                                          OScDev_Acquisition *acq) {
    struct WaveformParams params;
    SetWaveformParamsFromDevice(device, &params, acq);

    int32 elementsPerFramePerChan = GetClockWaveformSize(&params);

    // Q: Why do we use elementsPerFramePerChan, not
    // totalElementsPerFramePerChan?

    // digital line clock pattern for triggering acquisition line by line
    uInt8 *lineClockPattern = (uInt8 *)malloc(elementsPerFramePerChan);
    // digital line clock pattern for FLIM
    uInt8 *lineClockFLIM = (uInt8 *)malloc(elementsPerFramePerChan);
    // digital frame clock pattern for FLIM
    uInt8 *frameClockFLIM = (uInt8 *)malloc(elementsPerFramePerChan);
    // combination of lineClock, lineClockFLIM, and frameClock
    uInt8 *lineClockPatterns = (uInt8 *)malloc(
        elementsPerFramePerChan * GetImplData(device)->numDOChannels);

    // TODO: why use elementsPerLine instead of elementsPerFramePerChan?
    GenerateLineClock(&params, lineClockPattern);
    GenerateFLIMLineClock(&params, lineClockFLIM);
    GenerateFLIMFrameClock(&params, frameClockFLIM);

    // combine line, inverted line, and frame clocks
    // TODO: make it more generic
    for (int i = 0; i < elementsPerFramePerChan; i++) {
        lineClockPatterns[i] = lineClockPattern[i];
        lineClockPatterns[i + elementsPerFramePerChan] = lineClockFLIM[i];
        lineClockPatterns[i + 2 * elementsPerFramePerChan] = frameClockFLIM[i];
    }

    int32 numWritten = 0;
    OScDev_RichError *err = CreateDAQmxError(DAQmxWriteDigitalLines(
        config->doTask, elementsPerFramePerChan, FALSE, 10.0,
        DAQmx_Val_GroupByChannel, lineClockPatterns, &numWritten, NULL));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to write clock do waveforms");
        goto cleanup;
    }
    if (numWritten != elementsPerFramePerChan) {
        err =
            OScDev_Error_Wrap(err, "Failed to write complete clock waveform");
        goto cleanup;
    }

cleanup:
    free(lineClockPattern);
    free(lineClockFLIM);
    free(frameClockFLIM);
    free(lineClockPatterns);
    return err;
}

// Initialize, configure, and arm the clock, whatever its current state
OScDev_RichError *SetUpClock(OScDev_Device *device, struct ClockConfig *config,
                             OScDev_Acquisition *acq) {
    OScDev_RichError *err;

    bool mustCommit = false;

    if (!config->doTask || !config->lineCtrTask) {
        // In case one of the two tasks exists
        err = ShutdownClock(config);
        if (err)
            return err;

        err = CreateClockTasks(device, config, acq);
        if (err)
            return err;
        config->mustReconfigureTiming = true;
        config->mustReconfigureTriggers = true;
        config->mustRewriteOutput = true;
    }

    if (config->mustReconfigureTiming) {
        err = ConfigureClockTiming(device, config, acq);
        if (err)
            goto error;

        config->mustReconfigureTiming = false;
        mustCommit = true;
    }

    if (config->mustReconfigureTriggers) {
        err = ConfigureClockTriggers(device, config);
        if (err)
            goto error;

        config->mustReconfigureTriggers = false;
        mustCommit = true;
    }

    if (config->mustRewriteOutput) {
        err = WriteClockOutput(device, config, acq);
        if (err)
            goto error;

        config->mustRewriteOutput = false;
        mustCommit = true;
    }

    if (mustCommit) {
        err = CreateDAQmxError(
            DAQmxTaskControl(config->doTask, DAQmx_Val_Task_Commit));
        if (err) {
            err = OScDev_Error_Wrap(err, "Failed to commit clock do task");
            goto error;
        }

        err = CreateDAQmxError(
            DAQmxTaskControl(config->lineCtrTask, DAQmx_Val_Task_Commit));
        if (err) {
            err =
                OScDev_Error_Wrap(err, "Failed to commit clock lineCtr task");
            goto error;
        }
    }

    return OScDev_RichError_OK;

error:
    if (ShutdownClock(config))
        err = OScDev_Error_Wrap(
            err, "Failed to clean up clock task(s) after error");
    return err;
}

// Remove all DAQmx configuration for the clock
OScDev_RichError *ShutdownClock(struct ClockConfig *config) {
    OScDev_RichError *err1 = NULL, *err2 = NULL;

    if (config->doTask) {
        err1 = CreateDAQmxError(DAQmxClearTask(config->doTask));
        if (err1)
            err1 = OScDev_Error_Wrap(err1, "Failed to clear clock do task");
        config->doTask = 0;
    }

    if (config->lineCtrTask) {
        err2 = CreateDAQmxError(DAQmxClearTask(config->lineCtrTask));
        if (err2)
            err2 =
                OScDev_Error_Wrap(err2, "Failed to clear clock lineCtr task");
        config->lineCtrTask = 0;
    }

    if (err1) {
        OScDev_Error_Destroy(err2);
        return err1;
    } else {
        OScDev_Error_Destroy(err1);
        return err2;
    }
}

OScDev_RichError *StartClock(struct ClockConfig *config) {
    OScDev_RichError *err;

    err = CreateDAQmxError(DAQmxStartTask(config->doTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to start clock do task");
        ShutdownClock(config);
        return err;
    }

    err = CreateDAQmxError(DAQmxStartTask(config->lineCtrTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to start clock lineCtr task");
        ShutdownClock(config);
        return err;
    }

    return OScDev_RichError_OK;
}

OScDev_RichError *StopClock(struct ClockConfig *config) {
    OScDev_RichError *err;

    err = CreateDAQmxError(DAQmxStopTask(config->doTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to stop clock do task");
        ShutdownClock(config);
        return err;
    }

    err = CreateDAQmxError(DAQmxStopTask(config->lineCtrTask));
    if (err) {
        err = OScDev_Error_Wrap(err, "Failed to stop clock lineCtr task");
        ShutdownClock(config);
        return err;
    }

    return OScDev_RichError_OK;
}
