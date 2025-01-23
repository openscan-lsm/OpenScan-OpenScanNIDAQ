#include "NIDAQ_OpenScanSettings.h"

#include "OScNIDAQ.h"
#include "OScNIDAQPrivateData.h"

#include <stdio.h>
#include <string.h>

// For most settings, we set the setting's implData to the device.
// This function can then be used to retrieve the device implData.
static inline struct OScNIDAQPrivateData *
GetSettingDeviceData(OScDev_Setting *setting) {
    return (struct OScNIDAQPrivateData *)OScDev_Device_GetImplData(
        (OScDev_Device *)OScDev_Setting_GetImplData(setting));
}

static OScDev_Error GetNumericConstraintTypeImpl_DiscreteValues(
    OScDev_Setting *setting, OScDev_ValueConstraint *constraintType) {
    *constraintType = OScDev_ValueConstraint_DiscreteValues;
    return OScDev_OK;
}

static OScDev_Error
GetNumericConstraintTypeImpl_Range(OScDev_Setting *setting,
                                   OScDev_ValueConstraint *constraintType) {
    *constraintType = OScDev_ValueConstraint_Range;
    return OScDev_OK;
}

static OScDev_Error GetLineDelay(OScDev_Setting *setting, int32_t *value) {
    *value = GetSettingDeviceData(setting)->lineDelay;

    return OScDev_OK;
}

static OScDev_Error SetLineDelay(OScDev_Setting *setting, int32_t value) {
    GetSettingDeviceData(setting)->lineDelay = value;

    GetSettingDeviceData(setting)->clockConfig.mustReconfigureTiming = true;
    GetSettingDeviceData(setting)->scannerConfig.mustReconfigureTiming = true;
    GetSettingDeviceData(setting)->clockConfig.mustRewriteOutput = true;
    GetSettingDeviceData(setting)->scannerConfig.mustRewriteOutput = true;

    return OScDev_OK;
}

static OScDev_Error GetLineDelayRange(OScDev_Setting *setting, int32_t *min,
                                      int32_t *max) {
    *min = 1;
    *max = 200;
    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_LineDelay = {
    .GetInt32 = GetLineDelay,
    .SetInt32 = SetLineDelay,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
    .GetInt32Range = GetLineDelayRange,
};

static OScDev_Error GetParkingPositionX(OScDev_Setting *setting,
                                        int32_t *xPark) {
    *xPark = GetSettingDeviceData(setting)->xPark;

    return OScDev_OK;
}

static OScDev_Error SetParkingPositionX(OScDev_Setting *setting,
                                        int32_t xPark) {
    GetSettingDeviceData(setting)->xPark = xPark;

    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_ParkingPositionX = {
    .GetInt32 = GetParkingPositionX,
    .SetInt32 = SetParkingPositionX,
};

static OScDev_Error GetParkingPositionY(OScDev_Setting *setting,
                                        int32_t *yPark) {
    *yPark = GetSettingDeviceData(setting)->yPark;

    return OScDev_OK;
}

static OScDev_Error SetParkingPositionY(OScDev_Setting *setting,
                                        int32_t yPark) {
    GetSettingDeviceData(setting)->yPark = yPark;

    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_ParkingPositionY = {
    .GetInt32 = GetParkingPositionY,
    .SetInt32 = SetParkingPositionY,
};

static OScDev_Error GetAcqBufferSize(OScDev_Setting *setting, int32_t *value) {
    *value = GetSettingDeviceData(setting)->numLinesToBuffer;
    return OScDev_OK;
}

// OnAcqBufferSize
static OScDev_Error SetAcqBufferSize(OScDev_Setting *setting, int32_t value) {
    GetSettingDeviceData(setting)->numLinesToBuffer = value;

    GetSettingDeviceData(setting)->detectorConfig.mustReconfigureCallback =
        true;

    return OScDev_OK;
}

static OScDev_Error GetAcqBufferSizeValues(OScDev_Setting *setting,
                                           OScDev_NumArray **values) {
    static const uint32_t v[] = {
        2, 4, 8, 16, 32, 64, 128, 256,
        0 // End mark
    };
    *values = OScDev_NumArray_Create();
    for (size_t i = 0; v[i] != 0; ++i) {
        OScDev_NumArray_Append(*values, v[i]);
    }
    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_AcqBufferSize = {
    .GetInt32 = GetAcqBufferSize,
    .SetInt32 = SetAcqBufferSize,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_DiscreteValues,
    .GetInt32DiscreteValues = GetAcqBufferSizeValues,
};

static OScDev_Error GetInputVoltageRange(OScDev_Setting *setting,
                                         double *value) {
    *value = GetSettingDeviceData(setting)->inputVoltageRange;
    return OScDev_OK;
}

static OScDev_Error SetInputVoltageRange(OScDev_Setting *setting,
                                         double value) {
    GetSettingDeviceData(setting)->inputVoltageRange = value;
    return OScDev_OK;
}

static OScDev_Error GetInputVoltageRangeValues(OScDev_Setting *setting,
                                               OScDev_NumArray **values) {
    static const double v[] = {
        1.0000, 2.0000, 5.0000, 10.0000,
        0.0 // End mark
    };
    *values = OScDev_NumArray_Create();
    for (size_t i = 0; v[i] != 0.0; ++i) {
        OScDev_NumArray_Append(*values, v[i]);
    }
    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_InputVoltageRange = {
    .GetFloat64 = GetInputVoltageRange,
    .SetFloat64 = SetInputVoltageRange,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_DiscreteValues,
    .GetFloat64DiscreteValues = GetInputVoltageRangeValues,
};

struct EnableChannelData {
    OScDev_Device *device;
    int hwChannel;
};

static void ReleaseEnableChannel(OScDev_Setting *setting) {
    free(OScDev_Setting_GetImplData(setting));
}

static OScDev_Error GetEnableChannel(OScDev_Setting *setting, bool *value) {
    struct EnableChannelData *settingData =
        OScDev_Setting_GetImplData(setting);
    struct OScNIDAQPrivateData *devData = GetData(settingData->device);
    *value = devData->channelEnabled[settingData->hwChannel];
    return OScDev_OK;
}

static OScDev_Error SetEnableChannel(OScDev_Setting *setting, bool value) {
    struct EnableChannelData *settingData =
        OScDev_Setting_GetImplData(setting);
    struct OScNIDAQPrivateData *devData = GetData(settingData->device);
    devData->channelEnabled[settingData->hwChannel] = value;

    // Force recreation of detector task next time
    OScDev_RichError *err =
        ShutdownDetector(settingData->device, &devData->detectorConfig);
    return OScDev_Error_ReturnAsCode(err);
}

static OScDev_SettingImpl SettingImpl_EnableChannel = {
    .Release = ReleaseEnableChannel,
    .GetBool = GetEnableChannel,
    .SetBool = SetEnableChannel,
};

static OScDev_Error GetScannerOnly(OScDev_Setting *setting, bool *value) {
    *value = GetSettingDeviceData(setting)->scannerOnly;
    return OScDev_OK;
}

static OScDev_Error SetScannerOnly(OScDev_Setting *setting, bool value) {
    GetSettingDeviceData(setting)->scannerOnly = value;
    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_ScannerOnly = {
    .GetBool = GetScannerOnly,
    .SetBool = SetScannerOnly,
};

struct OffsetSettingData {
    OScDev_Device *device;
    int axis; // 0 = x, 1 = y
};

static OScDev_Error GetOffset(OScDev_Setting *setting, double *value) {
    struct OffsetSettingData *data =
        (struct OffsetSettingData *)OScDev_Setting_GetImplData(setting);
    *value = GetData(data->device)->offsetXY[data->axis];
    return OScDev_OK;
}

static OScDev_Error SetOffset(OScDev_Setting *setting, double value) {
    struct OffsetSettingData *data =
        (struct OffsetSettingData *)OScDev_Setting_GetImplData(setting);
    GetData(data->device)->offsetXY[data->axis] = value;

    GetData(data->device)->clockConfig.mustRewriteOutput = true;
    GetData(data->device)->scannerConfig.mustRewriteOutput = true;

    return OScDev_OK;
}

static OScDev_Error GetOffsetRange(OScDev_Setting *setting, double *min,
                                   double *max) {
    /*The galvoOffsetX and galvoOffsetY variables are expressed  in optical
    degrees This is a rough correspondence - it likely needs to be calibrated
    to the actual sensitivity of the galvos*/
    *min = -5.0;
    *max = +5.0;
    return OScDev_OK;
}

static void ReleaseOffset(OScDev_Setting *setting) {
    struct OffsetSettingData *data = OScDev_Setting_GetImplData(setting);
    free(data);
}

static OScDev_SettingImpl SettingImpl_Offset = {
    .GetFloat64 = GetOffset,
    .SetFloat64 = SetOffset,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
    .GetFloat64Range = GetOffsetRange,
    .Release = ReleaseOffset,
};

OScDev_Error NIDAQMakeSettings(OScDev_Device *device,
                               OScDev_PtrArray **settings) {
    OScDev_RichError *err;

    err = EnumerateAIPhysChans(device);
    if (err)
        OScDev_Error_ReturnAsCode(err);

    *settings = OScDev_PtrArray_Create();

    OScDev_Setting *lineDelay;
    err = OScDev_Error_AsRichError(OScDev_Setting_Create(
        &lineDelay, "Line Delay (pixels)", OScDev_ValueType_Int32,
        &SettingImpl_LineDelay, device));
    if (err)
        goto error;
    OScDev_PtrArray_Append(*settings, lineDelay);

    OScDev_Setting *parkingPositionX;
    err = OScDev_Error_AsRichError(OScDev_Setting_Create(
        &parkingPositionX, "Parking Position X (pixels)",
        OScDev_ValueType_Int32, &SettingImpl_ParkingPositionX, device));
    if (err)
        goto error;
    OScDev_PtrArray_Append(*settings, parkingPositionX);

    OScDev_Setting *parkingPositionY;
    err = OScDev_Error_AsRichError(OScDev_Setting_Create(
        &parkingPositionY, "Parking Position Y (pixels)",
        OScDev_ValueType_Int32, &SettingImpl_ParkingPositionY, device));
    if (err)
        goto error;
    OScDev_PtrArray_Append(*settings, parkingPositionY);

    for (int i = 0; i < 2; ++i) {
        OScDev_Setting *offset;
        struct OffsetSettingData *data =
            malloc(sizeof(struct OffsetSettingData));
        data->device = device;
        data->axis = i;
        const char *name =
            i == 0 ? "GalvoOffsetX (degree)" : "GalvoOffsetY (degree)";
        err = OScDev_Error_AsRichError(
            OScDev_Setting_Create(&offset, name, OScDev_ValueType_Float64,
                                  &SettingImpl_Offset, data));
        if (err)
            goto error;
        OScDev_PtrArray_Append(*settings, offset);
    }

    OScDev_Setting *numLinesToBuffer;
    err = OScDev_Error_AsRichError(OScDev_Setting_Create(
        &numLinesToBuffer, "Acq Buffer Size (lines)", OScDev_ValueType_Int32,
        &SettingImpl_AcqBufferSize, device));
    if (err)
        goto error;
    OScDev_PtrArray_Append(*settings, numLinesToBuffer);

    int nPhysChans = GetNumberOfAIPhysChans(device);
    for (int i = 0; i < nPhysChans; ++i) {
        struct EnableChannelData *data =
            malloc(sizeof(struct EnableChannelData));
        data->device = device;
        data->hwChannel = i;
        char name[64];
        snprintf(name, sizeof(name), "EnableChannel%d", i);
        OScDev_Setting *enableChannel;
        err = OScDev_Error_AsRichError(
            OScDev_Setting_Create(&enableChannel, name, OScDev_ValueType_Bool,
                                  &SettingImpl_EnableChannel, data));
        if (err)
            goto error;
        OScDev_PtrArray_Append(*settings, enableChannel);
    }

    OScDev_Setting *inputVoltageRange;
    err = OScDev_Error_AsRichError(OScDev_Setting_Create(
        &inputVoltageRange, "Input Voltage Range", OScDev_ValueType_Float64,
        &SettingImpl_InputVoltageRange, device));
    if (err)
        goto error;
    OScDev_PtrArray_Append(*settings, inputVoltageRange);

    OScDev_Setting *scannerOnly;
    err = OScDev_Error_AsRichError(OScDev_Setting_Create(
        &scannerOnly, "ScannerOnly", OScDev_ValueType_Bool,
        &SettingImpl_ScannerOnly, device));
    if (err)
        goto error;
    OScDev_PtrArray_Append(
        *settings, scannerOnly); // TODO Remove when supported by OpenScanLib

    return OScDev_OK;

error:
    for (size_t i = 0; i < OScDev_PtrArray_Size(*settings); ++i) {
        OScDev_Setting_Destroy(OScDev_PtrArray_At(*settings, i));
    }
    OScDev_PtrArray_Destroy(*settings);
    *settings = NULL;
    return OScDev_Error_ReturnAsCode(err);
}
