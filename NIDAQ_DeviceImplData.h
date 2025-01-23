#pragma once

#include "Clock.h"
#include "Detector.h"
#include "Scanner.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>
#include <ss8str.h>

#include <Windows.h>

#define MAX_PHYSICAL_CHANS 8

// This struct holds the NIDAQ-specific device state and is associated with the
// OpenScan device through the "impl data" mechanism.
struct DeviceImplData {
    // The DAQmx name for the DAQ card
    ss8str deviceName;

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
    ss8str aiPhysChans; // at least numAIPhysChans elements separated by ", "
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

static inline struct DeviceImplData *GetImplData(OScDev_Device *device) {
    return (struct DeviceImplData *)OScDev_Device_GetImplData(device);
}

void InitializeImplData(struct DeviceImplData *data);
