#pragma once

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>

// DAQmx tasks and flags to track invalidated configurations for clock
// See Clock.c
struct ClockConfig {
    TaskHandle doTask;
    TaskHandle lineCtrTask;
    bool mustReconfigureTiming;
    bool mustReconfigureTriggers;
    bool mustRewriteOutput;
};

OScDev_RichError *SetUpClock(OScDev_Device *device, struct ClockConfig *config,
                             OScDev_Acquisition *acq);
OScDev_RichError *ShutdownClock(struct ClockConfig *config);
OScDev_RichError *StartClock(struct ClockConfig *config);
OScDev_RichError *StopClock(struct ClockConfig *config);
