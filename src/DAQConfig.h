#pragma once

#include "Waveform.h"

#include <NIDAQmx.h>
#include <OpenScanDeviceLib.h>
#include <ss8str.h>

#include <stdbool.h>

bool ComputeAORateHz(double pixelRateHz, double timebaseHz, double aoMaxHz,
                     double *aoRateHz);
OScDev_RichError *EnsureTimingCapsQueried(OScDev_Device *device);
void SetWaveformParamsFromDevice(OScDev_Device *device,
                                 struct WaveformParams *parameters,
                                 OScDev_Acquisition *acq);
OScDev_RichError *EnumerateAIPhysChans(OScDev_Device *device);
void GetEnabledChannels(OScDev_Device *device, ss8str *chans);
int GetNumberOfEnabledChannels(OScDev_Device *device);
int GetNumberOfAIPhysChans(OScDev_Device *device);

OScDev_RichError *EnumerateAOPhysChans(OScDev_Device *device);
int GetNumberOfAOPhysChans(OScDev_Device *device);
bool LaserBlankingSupported(OScDev_Device *device);
int GetNumberOfScannerAOChannels(OScDev_Device *device);
