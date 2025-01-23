#pragma once

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>

#include <stdbool.h>

// DAQmx task and flags to track invalidated configurations for detector
// See Detector.c
struct DetectorConfig {
    TaskHandle aiTask;
    bool mustReconfigureTiming;
    bool mustReconfigureTrigger;
    bool mustReconfigureCallback;
};

OScDev_RichError *SetUpDetector(OScDev_Device *device,
                                struct DetectorConfig *config,
                                OScDev_Acquisition *acq);
OScDev_RichError *ShutdownDetector(OScDev_Device *device,
                                   struct DetectorConfig *config);
OScDev_RichError *StartDetector(OScDev_Device *device,
                                struct DetectorConfig *config);
OScDev_RichError *StopDetector(OScDev_Device *device,
                               struct DetectorConfig *config);
