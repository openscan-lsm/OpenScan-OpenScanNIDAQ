#pragma once

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>

// DAQmx task and flags to track invalidated configurations for scanner
// See Scanner.c
struct ScannerConfig {
    TaskHandle aoTask;
    bool mustReconfigureTiming;
    bool mustRewriteOutput;
};

OScDev_RichError *SetUpScanner(OScDev_Device *device,
                               struct ScannerConfig *config,
                               OScDev_Acquisition *acq);
OScDev_RichError *ShutdownScanner(struct ScannerConfig *config);
OScDev_RichError *StartScanner(struct ScannerConfig *config);
OScDev_RichError *StopScanner(struct ScannerConfig *config);

OScDev_RichError *CreateScannerTask(OScDev_Device *device,
                                    struct ScannerConfig *config);

// Drive a single static sample on ao0:2 (galvos held at last park voltages,
// EOM at its current manual/blank level) when no acquisition is running. No-op
// if blanking is unsupported or an acquisition is in progress.
OScDev_RichError *ApplyIdleLaserOutput(OScDev_Device *device);
