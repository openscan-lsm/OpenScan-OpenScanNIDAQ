#include "OpenScanDevice.h"

#include "Acquisition.h"
#include "DAQConfig.h"
#include "DAQError.h"
#include "DeviceImplData.h"
#include "OpenScanSettings.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>
#include <ss8str.h>

#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <Windows.h>

static OScDev_Error NIDAQGetModelName(const char **name) {
    *name = "OpenScan-NIDAQ";
    return OScDev_OK;
}

static OScDev_Error NIDAQEnumerateInstances(OScDev_PtrArray **devices) {
    OScDev_RichError *err = OScDev_RichError_OK;
    ss8str names;
    ss8_init(&names);
    ss8str name;
    ss8_init(&name);
    ss8str msg;
    ss8_init(&msg);
    *devices = OScDev_PtrArray_Create();

    ss8_set_len(&names, 4096);
    err = CreateDAQmxError(DAQmxGetSysDevNames(ss8_mutable_cstr(&names),
                                               (uInt32)ss8_len(&names)));
    if (err)
        goto finish;
    ss8_set_len_to_cstrlen(&names);

    size_t start = 0;
    for (;;) {
        size_t stop = ss8_find_ch(&names, start, ',');
        ss8_copy_substr(&name, &names, start, stop - start);
        ss8_strip_ch(&name, ' ');

        struct DeviceImplData *data = malloc(sizeof(struct DeviceImplData));
        InitializeImplData(data);
        ss8_copy(&data->deviceName, &name);

        OScDev_Device *device;
        err = OScDev_Error_AsRichError(
            OScDev_Device_Create(&device, &NIDAQDeviceImpl, data));
        if (err) {
            ss8_copy_cstr(&msg, "Failed to create device for ");
            ss8_cat(&msg, &name);
            err = OScDev_Error_Wrap(err, ss8_cstr(&msg));
            // TODO We have no way to destroy the already-created devices.
            // (But this failure is unlikely unless out of memory.)
            goto finish;
        }
        OScDev_PtrArray_Append(*devices, device);

        if (stop == SIZE_MAX)
            break;
        start = stop + 1;
    }

finish:
    if (err) {
        OScDev_PtrArray_Destroy(*devices);
    }
    ss8_destroy(&msg);
    ss8_destroy(&name);
    ss8_destroy(&names);
    return OScDev_Error_ReturnAsCode(err);
}

static OScDev_Error NIDAQReleaseInstance(OScDev_Device *device) {
    ss8_destroy(&GetImplData(device)->deviceName);
    ss8_destroy(&GetImplData(device)->aiPhysChans);
    free(GetImplData(device));
    return OScDev_OK;
}

static OScDev_Error NIDAQGetName(OScDev_Device *device, char *name) {
    ss8_copy_to_cstr(&GetImplData(device)->deviceName, name,
                     OScDev_MAX_STR_SIZE);
    return OScDev_OK;
}

static OScDev_Error NIDAQOpen(OScDev_Device *device) {
    OScDev_RichError *err = OScDev_RichError_OK;
    ss8str msg;
    ss8_init(&msg);

    err = CreateDAQmxError(
        DAQmxResetDevice(ss8_cstr(&GetImplData(device)->deviceName)));
    if (err) {
        ss8_copy_cstr(&msg, "Cannot reset device: ");
        ss8_cat(&msg, &GetImplData(device)->deviceName);
        err = OScDev_Error_Wrap(err, ss8_cstr(&msg));
    }

    ss8_destroy(&msg);
    return OScDev_Error_ReturnAsCode(err);
}

static OScDev_Error NIDAQClose(OScDev_Device *device) {
    StopAcquisitionAndWait(device);
    return OScDev_OK;
}

static OScDev_Error NIDAQHasClock(OScDev_Device *device, bool *hasClock) {
    (void)device; // Unused
    *hasClock = true;
    return OScDev_OK;
}

static OScDev_Error NIDAQHasScanner(OScDev_Device *device, bool *hasScanner) {
    (void)device; // Unused
    *hasScanner = true;
    return OScDev_OK;
}

static OScDev_Error NIDAQHasDetector(OScDev_Device *device,
                                     bool *hasDetector) {
    (void)device; // Unused
    *hasDetector = true;
    return OScDev_OK;
}

static OScDev_Error NIDAQGetPixelRates(OScDev_Device *device,
                                       OScDev_NumRange **pixelRatesHz) {
    (void)device; // Unused
    static const double ratesMHz[] = {
        0.0500, 0.1000, 0.1250, 0.2000, 0.2500,
        0.4000, 0.5000, 0.6250, 1.0000, 1.2500,
        0.0 // End mark
    };
    *pixelRatesHz = OScDev_NumRange_CreateDiscrete();
    for (size_t i = 0; ratesMHz[i] != 0.0; ++i) {
        OScDev_NumRange_AppendDiscrete(*pixelRatesHz, 1e6 * ratesMHz[i]);
    }
    return OScDev_OK;
}

static OScDev_Error NIDAQGetResolutions(OScDev_Device *device,
                                        OScDev_NumRange **resolutions) {
    (void)device; // Unused
    *resolutions = OScDev_NumRange_CreateDiscrete();
    OScDev_NumRange_AppendDiscrete(*resolutions, 256);
    OScDev_NumRange_AppendDiscrete(*resolutions, 512);
    OScDev_NumRange_AppendDiscrete(*resolutions, 1024);
    OScDev_NumRange_AppendDiscrete(*resolutions, 2048);
    return OScDev_OK;
}

static OScDev_Error NIDAQGetZoomFactors(OScDev_Device *device,
                                        OScDev_NumRange **zooms) {
    (void)device; // Unused
    *zooms = OScDev_NumRange_CreateContinuous(0.2, 20.0);
    return OScDev_OK;
}

static OScDev_Error NIDAQIsROIScanSupported(OScDev_Device *device,
                                            bool *supported) {
    (void)device; // Unused
    *supported = true;
    return OScDev_OK;
}

static OScDev_Error NIDAQGetNumberOfChannels(OScDev_Device *device,
                                             uint32_t *nChannels) {
    *nChannels = GetNumberOfEnabledChannels(device);
    return OScDev_OK;
}

static OScDev_Error NIDAQGetBytesPerSample(OScDev_Device *device,
                                           uint32_t *bytesPerSample) {
    (void)device; // Unused
    *bytesPerSample = 2;
    return OScDev_OK;
}

static OScDev_Error NIDAQArm(OScDev_Device *device, OScDev_Acquisition *acq) {
    bool useClock, useScanner, useDetector;
    OScDev_Acquisition_IsClockRequested(acq, &useClock);
    OScDev_Acquisition_IsScannerRequested(acq, &useScanner);
    OScDev_Acquisition_IsDetectorRequested(acq, &useDetector);

    if (!useClock || !useScanner)
        return OScDev_Error_ReturnAsCode(OScDev_Error_Create(
            "Unsupported operation (cannot disable clock or scanner)"));

    OScDev_TriggerSource clockStartTriggerSource;
    OScDev_Acquisition_GetClockStartTriggerSource(acq,
                                                  &clockStartTriggerSource);
    if (clockStartTriggerSource != OScDev_TriggerSource_Software)
        return OScDev_Error_ReturnAsCode(OScDev_Error_Create(
            "Unsupported operation (trigger source must be software)"));

    OScDev_ClockSource clockSource;
    OScDev_Acquisition_GetClockSource(acq, &clockSource);
    if (clockSource != OScDev_ClockSource_Internal)
        return OScDev_Error_ReturnAsCode(OScDev_Error_Create(
            "Unsupported operation (clock source must be internal)"));

    return OScDev_Error_ReturnAsCode(
        ArmAcquisition(device, acq, !useDetector));
}

static OScDev_Error NIDAQStart(OScDev_Device *device) {
    return OScDev_Error_ReturnAsCode(StartAcquisition(device));
}

static OScDev_Error NIDAQStop(OScDev_Device *device) {
    return OScDev_Error_ReturnAsCode(StopAcquisitionAndWait(device));
}

static OScDev_Error NIDAQIsRunning(OScDev_Device *device, bool *isRunning) {
    return OScDev_Error_ReturnAsCode(IsAcquisitionRunning(device, isRunning));
}

static OScDev_Error NIDAQWait(OScDev_Device *device) {
    return OScDev_Error_ReturnAsCode(WaitForAcquisitionToFinish(device));
}

OScDev_DeviceImpl NIDAQDeviceImpl = {
    .GetModelName = NIDAQGetModelName,
    .EnumerateInstances = NIDAQEnumerateInstances,
    .ReleaseInstance = NIDAQReleaseInstance,
    .GetName = NIDAQGetName,
    .Open = NIDAQOpen,
    .Close = NIDAQClose,
    .HasClock = NIDAQHasClock,
    .HasScanner = NIDAQHasScanner,
    .HasDetector = NIDAQHasDetector,
    .MakeSettings = NIDAQMakeSettings,
    .GetPixelRates = NIDAQGetPixelRates,
    .GetResolutions = NIDAQGetResolutions,
    .GetZoomFactors = NIDAQGetZoomFactors,
    .IsROIScanSupported = NIDAQIsROIScanSupported,
    .GetNumberOfChannels = NIDAQGetNumberOfChannels,
    .GetBytesPerSample = NIDAQGetBytesPerSample,
    .Arm = NIDAQArm,
    .Start = NIDAQStart,
    .Stop = NIDAQStop,
    .IsRunning = NIDAQIsRunning,
    .Wait = NIDAQWait,
};
