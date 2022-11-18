#pragma once

#include "OpenScanDeviceLib.h"

#include <NIDAQmx.h>

#include <ss8str.h>

#include <Windows.h>

#define MAX_PHYSICAL_CHANS 8

// DAQmx tasks and flags to track invalidated configurations for clock
// See Clock.c
struct ClockConfig {
    TaskHandle doTask;
    TaskHandle lineCtrTask;
    bool mustReconfigureTiming;
    bool mustReconfigureTriggers;
    bool mustRewriteOutput;
};

// DAQmx task and flags to track invalidated configurations for scanner
// See Scanner.c
struct ScannerConfig {
    TaskHandle aoTask;
    bool mustReconfigureTiming;
    bool mustRewriteOutput;
};

// DAQmx task and flags to track invalidated configurations for detector
// See Detector.c
struct DetectorConfig {
    TaskHandle aiTask;
    bool mustReconfigureTiming;
    bool mustReconfigureTrigger;
    bool mustReconfigureCallback;
};

struct OScNIDAQPrivateData {
    // The DAQmx name for the DAQ card
    char deviceName[OScDev_MAX_STR_LEN + 1];

    struct ClockConfig clockConfig;
    struct ScannerConfig scannerConfig;
    struct DetectorConfig detectorConfig;
    double configuredPixelRateHz;
    uint32_t configuredResolution;
    double configuredZoomFactor;
    uint32_t configuredXOffset, configuredYOffset;
    uint32_t configuredRasterWidth, configuredRasterHeight;

    bool oneFrameScanDone;
    bool scannerOnly;

    // counted as number of pixels.
    // to adjust for the lag between the mirror control signal and the actual
    // position of the mirror scan phase (uSec) = line delay / scan rate
    uint32_t lineDelay;

    int32_t xPark;
    int32_t yPark;
    double prevXParkVoltage;
    double prevYParkVoltage;

    uint32_t numLinesToBuffer;
    double inputVoltageRange;
    uInt32
        numDOChannels; // Number of DO lines under current clock configuration
    double offsetXY[2];
    double minVolts_; // min possible for device
    double maxVolts_; // max possible for device

    int numAIPhysChans; // Not to exceed MAX_PHYSICAL_CHANS
    char *
        aiPhysChans; // ", "-delimited string; at least numAIPhysChans elements
    bool channelEnabled[MAX_PHYSICAL_CHANS];

    // Read, but unprocessed, raw samples; channels interleaved
    // Leftover data from the previous read, if any, is at the start of the
    // buffer and consists of rawDataSize samples.
    float64 *rawDataBuffer;
    size_t rawDataSize;     // Current data size
    size_t rawDataCapacity; // Buffer size

    // Per-channel frame buffers that we fill in and pass to OpenScanLib
    // Index is order among currently enabled channels.
    // Buffers for unused channels may not be allocated.
    uint16_t *frameBuffers[MAX_PHYSICAL_CHANS];
    size_t framePixelsFilled;

    struct {
        CRITICAL_SECTION mutex;
        HANDLE thread;
        CONDITION_VARIABLE acquisitionFinishCondition;
        bool running;
        bool armed;         // Valid when running == true
        bool started;       // Valid when running == true
        bool stopRequested; // Valid when running == true
        OScDev_Acquisition *acquisition;
    } acquisition;
};

static inline struct OScNIDAQPrivateData *GetData(OScDev_Device *device) {
    return (struct OScNIDAQPrivateData *)OScDev_Device_GetImplData(device);
}

void SetWaveformParamsFromDevice(OScDev_Device *device,
                                 struct WaveformParams *parameters,
                                 OScDev_Acquisition *acq);
OScDev_RichError *EnumerateInstances(OScDev_PtrArray **devices,
                                     OScDev_DeviceImpl *impl);
OScDev_RichError *EnumerateAIPhysChans(OScDev_Device *device);
int GetNumberOfEnabledChannels(OScDev_Device *device);
void GetEnabledChannels(OScDev_Device *device, ss8str *chans);
int GetNumberOfAIPhysChans(OScDev_Device *device);
bool GetAIPhysChan(OScDev_Device *device, int index, ss8str *chan);
OScDev_Error NIDAQMakeSettings(OScDev_Device *device,
                               OScDev_PtrArray **settings);

OScDev_RichError *SetUpClock(OScDev_Device *device, struct ClockConfig *config,
                             OScDev_Acquisition *acq);
OScDev_RichError *ShutdownClock(OScDev_Device *device,
                                struct ClockConfig *config);
OScDev_RichError *StartClock(OScDev_Device *device,
                             struct ClockConfig *config);
OScDev_RichError *StopClock(OScDev_Device *device, struct ClockConfig *config);
OScDev_RichError *SetUpScanner(OScDev_Device *device,
                               struct ScannerConfig *config,
                               OScDev_Acquisition *acq);
OScDev_RichError *ShutdownScanner(OScDev_Device *device,
                                  struct ScannerConfig *config);
OScDev_RichError *StartScanner(OScDev_Device *device,
                               struct ScannerConfig *config);
OScDev_RichError *StopScanner(OScDev_Device *device,
                              struct ScannerConfig *config);
OScDev_RichError *SetUpDetector(OScDev_Device *device,
                                struct DetectorConfig *config,
                                OScDev_Acquisition *acq);
OScDev_RichError *ShutdownDetector(OScDev_Device *device,
                                   struct DetectorConfig *config);
OScDev_RichError *StartDetector(OScDev_Device *device,
                                struct DetectorConfig *config);
OScDev_RichError *StopDetector(OScDev_Device *device,
                               struct DetectorConfig *config);
OScDev_RichError *CreateScannerTask(OScDev_Device *device,
                                    struct ScannerConfig *config);
OScDev_RichError *ConfigureUnparkTiming(OScDev_Device *device,
                                        struct ScannerConfig *config,
                                        OScDev_Acquisition *acq);
OScDev_RichError *ConfigureParkTiming(OScDev_Device *device,
                                      struct ScannerConfig *config,
                                      OScDev_Acquisition *acq);
OScDev_RichError *WriteUnparkOutput(OScDev_Device *device,
                                    struct ScannerConfig *config,
                                    OScDev_Acquisition *acq);
OScDev_RichError *WriteParkOutput(OScDev_Device *device,
                                  struct ScannerConfig *config,
                                  OScDev_Acquisition *acq);
OScDev_RichError *GenerateUnparkOutput(OScDev_Device *device,
                                       struct ScannerConfig *config,
                                       OScDev_Acquisition *acq);
OScDev_RichError *GenerateParkOutput(OScDev_Device *device,
                                     struct ScannerConfig *config,
                                     OScDev_Acquisition *acq);

// Must be called immediately after failed DAQmx function
void LogNiError(OScDev_Device *device, int32 nierr, const char *when);
char *ErrorCodeDomain();
// Must be called immediately after failed DAQmx function
OScDev_RichError *CreateDAQmxError(int32 nierr);
