#pragma once

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>

#include <Windows.h>

struct SpiralGenState;

// DAQmx task and flags to track invalidated configurations for scanner
// See Scanner.c
struct ScannerConfig {
    TaskHandle aoTask;
    bool mustReconfigureTiming;
    bool mustRewriteOutput;

    struct SpiralGenState *spiralState;
    HANDLE spiralWriterThread;
    volatile bool spiralStopRequested;
    uint32_t spiralTotalFrames;
};

OScDev_RichError *SetUpScanner(OScDev_Device *device,
                               struct ScannerConfig *config,
                               OScDev_Acquisition *acq);
OScDev_RichError *ShutdownScanner(struct ScannerConfig *config);
OScDev_RichError *StartScanner(OScDev_Device *device,
                               struct ScannerConfig *config);
OScDev_RichError *StopScanner(OScDev_Device *device,
                              struct ScannerConfig *config);

OScDev_RichError *CreateScannerTask(OScDev_Device *device,
                                    struct ScannerConfig *config);
