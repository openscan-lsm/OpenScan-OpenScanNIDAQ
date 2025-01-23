#pragma once

#include "Waveform.h"

#include <NIDAQmx.h>
#include <ss8str.h>

// Must be called immediately after failed DAQmx function
void LogNiError(OScDev_Device *device, int32 nierr, const char *when);
// Must be called immediately after failed DAQmx function
OScDev_RichError *CreateDAQmxError(int32 nierr);

void SetWaveformParamsFromDevice(OScDev_Device *device,
                                 struct WaveformParams *parameters,
                                 OScDev_Acquisition *acq);
OScDev_RichError *EnumerateInstances(OScDev_PtrArray **devices,
                                     OScDev_DeviceImpl *impl);
OScDev_RichError *EnumerateAIPhysChans(OScDev_Device *device);
void GetEnabledChannels(OScDev_Device *device, ss8str *chans);
int GetNumberOfEnabledChannels(OScDev_Device *device);
int GetNumberOfAIPhysChans(OScDev_Device *device);

OScDev_RichError *RunAcquisitionLoop(OScDev_Device *device);
OScDev_RichError *StopAcquisitionAndWait(OScDev_Device *device);
OScDev_RichError *IsAcquisitionRunning(OScDev_Device *device, bool *isRunning);
OScDev_RichError *WaitForAcquisitionToFinish(OScDev_Device *device);
OScDev_RichError *ReconfigDAQ(OScDev_Device *device, OScDev_Acquisition *acq);
