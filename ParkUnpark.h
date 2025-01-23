#pragma once

#include <OpenScanDeviceLib.h>

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
