#include "OpenScanSettings.h"

#include "DAQConfig.h"
#include "DeviceImplData.h"

#include <OpenScanDeviceLib.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// For most settings, we set the setting's implData to the device.
// This function can then be used to retrieve the device implData.
static inline struct DeviceImplData *
GetSettingDeviceData(OScDev_Setting *setting) {
    return (struct DeviceImplData *)OScDev_Device_GetImplData(
        (OScDev_Device *)OScDev_Setting_GetImplData(setting));
}

static OScDev_Error GetNumericConstraintTypeImpl_DiscreteValues(
    OScDev_Setting *setting, OScDev_ValueConstraint *constraintType) {
    (void)setting; // Unused
    *constraintType = OScDev_ValueConstraint_DiscreteValues;
    return OScDev_OK;
}

static OScDev_Error
GetNumericConstraintTypeImpl_Range(OScDev_Setting *setting,
                                   OScDev_ValueConstraint *constraintType) {
    (void)setting; // Unused
    *constraintType = OScDev_ValueConstraint_Range;
    return OScDev_OK;
}

static OScDev_Error GetUndershoot(OScDev_Setting *setting, double *value) {
    *value = GetSettingDeviceData(setting)->undershootUs;
    return OScDev_OK;
}

static OScDev_Error SetUndershoot(OScDev_Setting *setting, double value) {
    GetSettingDeviceData(setting)->undershootUs = value;
    GetSettingDeviceData(setting)->clockConfig.mustReconfigureTiming = true;
    GetSettingDeviceData(setting)->scannerConfig.mustReconfigureTiming = true;
    GetSettingDeviceData(setting)->clockConfig.mustRewriteOutput = true;
    GetSettingDeviceData(setting)->scannerConfig.mustRewriteOutput = true;
    return OScDev_OK;
}

static OScDev_Error GetUndershootRange(OScDev_Setting *setting, double *min,
                                       double *max) {
    (void)setting;
    *min = 0.0;
    *max = 5000.0;
    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_Undershoot = {
    .GetFloat64 = GetUndershoot,
    .SetFloat64 = SetUndershoot,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
    .GetFloat64Range = GetUndershootRange,
};

static OScDev_Error GetScanPhase(OScDev_Setting *setting, double *value) {
    *value = GetSettingDeviceData(setting)->scanPhaseUs;
    return OScDev_OK;
}

static OScDev_Error SetScanPhase(OScDev_Setting *setting, double value) {
    GetSettingDeviceData(setting)->scanPhaseUs = value;
    GetSettingDeviceData(setting)->clockConfig.mustReconfigureTiming = true;
    GetSettingDeviceData(setting)->scannerConfig.mustReconfigureTiming = true;
    GetSettingDeviceData(setting)->clockConfig.mustRewriteOutput = true;
    GetSettingDeviceData(setting)->scannerConfig.mustRewriteOutput = true;
    return OScDev_OK;
}

static OScDev_Error GetScanPhaseRange(OScDev_Setting *setting, double *min,
                                      double *max) {
    (void)setting;
    *min = 0.0;
    *max = 1000.0;
    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_ScanPhase = {
    .GetFloat64 = GetScanPhase,
    .SetFloat64 = SetScanPhase,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
    .GetFloat64Range = GetScanPhaseRange,
};

static OScDev_Error GetRetraceScale(OScDev_Setting *setting, double *value) {
    *value = GetSettingDeviceData(setting)->retraceScaleUsPerVolt;
    return OScDev_OK;
}

static OScDev_Error SetRetraceScale(OScDev_Setting *setting, double value) {
    GetSettingDeviceData(setting)->retraceScaleUsPerVolt = value;
    GetSettingDeviceData(setting)->clockConfig.mustReconfigureTiming = true;
    GetSettingDeviceData(setting)->scannerConfig.mustReconfigureTiming = true;
    GetSettingDeviceData(setting)->clockConfig.mustRewriteOutput = true;
    GetSettingDeviceData(setting)->scannerConfig.mustRewriteOutput = true;
    return OScDev_OK;
}

static OScDev_Error GetRetraceScaleRange(OScDev_Setting *setting, double *min,
                                         double *max) {
    (void)setting;
    *min = 100.0;
    *max = 10000.0;
    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_RetraceScale = {
    .GetFloat64 = GetRetraceScale,
    .SetFloat64 = SetRetraceScale,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
    .GetFloat64Range = GetRetraceScaleRange,
};

struct LaserSettingData {
    OScDev_Device *device;
    int laserIndex;
};

static void ReleaseLaserSetting(OScDev_Setting *setting) {
    free(OScDev_Setting_GetImplData(setting));
}

static struct LaserChannelParams *GetSettingLaser(OScDev_Setting *setting) {
    struct LaserSettingData *data = OScDev_Setting_GetImplData(setting);
    return &GetImplData(data->device)->lasers[data->laserIndex];
}

static OScDev_Device *GetLaserSettingDevice(OScDev_Setting *setting) {
    struct LaserSettingData *data = OScDev_Setting_GetImplData(setting);
    return data->device;
}

static OScDev_Error UpdateLaserOutput(OScDev_Setting *setting) {
    OScDev_Device *device = GetLaserSettingDevice(setting);
    GetImplData(device)->scannerConfig.mustRewriteOutput = true;
    return OScDev_Error_ReturnAsCode(ApplyIdleLaserOutput(device));
}

static const char *const laserControlNames[] = {
    [LaserControlOff] = "Off",
    [LaserControlBlank] = "Blank",
    [LaserControlOn] = "On",
};
#define NUM_LASER_CONTROL_VALUES                                              \
    (sizeof(laserControlNames) / sizeof(laserControlNames[0]))

static OScDev_Error GetLaserControl(OScDev_Setting *setting, uint32_t *value) {
    *value = GetSettingLaser(setting)->control;
    return OScDev_OK;
}

static OScDev_Error SetLaserControl(OScDev_Setting *setting, uint32_t value) {
    if (value >= NUM_LASER_CONTROL_VALUES)
        return OScDev_Error_Illegal_Argument;
    GetSettingLaser(setting)->control = (enum LaserControl)value;
    return UpdateLaserOutput(setting);
}

static OScDev_Error GetLaserControlNumValues(OScDev_Setting *setting,
                                             uint32_t *count) {
    (void)setting;
    *count = NUM_LASER_CONTROL_VALUES;
    return OScDev_OK;
}

static OScDev_Error GetLaserControlNameForValue(OScDev_Setting *setting,
                                                uint32_t value, char *name) {
    (void)setting;
    if (value >= NUM_LASER_CONTROL_VALUES)
        return OScDev_Error_Illegal_Argument;
    snprintf(name, OScDev_MAX_STR_SIZE, "%s", laserControlNames[value]);
    return OScDev_OK;
}

static OScDev_Error GetLaserControlValueForName(OScDev_Setting *setting,
                                                uint32_t *value,
                                                const char *name) {
    (void)setting;
    for (uint32_t i = 0; i < NUM_LASER_CONTROL_VALUES; ++i) {
        if (strcmp(name, laserControlNames[i]) == 0) {
            *value = i;
            return OScDev_OK;
        }
    }
    return OScDev_Error_Illegal_Argument;
}

static OScDev_SettingImpl SettingImpl_LaserControl = {
    .Release = ReleaseLaserSetting,
    .GetEnum = GetLaserControl,
    .SetEnum = SetLaserControl,
    .GetEnumNumValues = GetLaserControlNumValues,
    .GetEnumNameForValue = GetLaserControlNameForValue,
    .GetEnumValueForName = GetLaserControlValueForName,
};

static OScDev_Error GetLaserOnVoltage(OScDev_Setting *setting, double *value) {
    *value = GetSettingLaser(setting)->onVoltage;
    return OScDev_OK;
}

static OScDev_Error SetLaserOnVoltage(OScDev_Setting *setting, double value) {
    GetSettingLaser(setting)->onVoltage = value;
    return UpdateLaserOutput(setting);
}

static OScDev_Error GetLaserVoltageRange(OScDev_Setting *setting, double *min,
                                         double *max) {
    (void)setting;
    *min = -1.0;
    *max = 1.0;
    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_LaserOnVoltage = {
    .Release = ReleaseLaserSetting,
    .GetFloat64 = GetLaserOnVoltage,
    .SetFloat64 = SetLaserOnVoltage,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
    .GetFloat64Range = GetLaserVoltageRange,
};

static OScDev_Error GetLaserOffVoltage(OScDev_Setting *setting,
                                       double *value) {
    *value = GetSettingLaser(setting)->offVoltage;
    return OScDev_OK;
}

static OScDev_Error SetLaserOffVoltage(OScDev_Setting *setting, double value) {
    GetSettingLaser(setting)->offVoltage = value;
    return UpdateLaserOutput(setting);
}

static OScDev_SettingImpl SettingImpl_LaserOffVoltage = {
    .Release = ReleaseLaserSetting,
    .GetFloat64 = GetLaserOffVoltage,
    .SetFloat64 = SetLaserOffVoltage,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
    .GetFloat64Range = GetLaserVoltageRange,
};

static OScDev_Error GetLaserOnLead(OScDev_Setting *setting, double *value) {
    *value = GetSettingLaser(setting)->leadUs;
    return OScDev_OK;
}

static OScDev_Error SetLaserOnLead(OScDev_Setting *setting, double value) {
    GetSettingLaser(setting)->leadUs = value;
    GetImplData(GetLaserSettingDevice(setting))
        ->scannerConfig.mustRewriteOutput = true;
    return OScDev_OK;
}

static OScDev_Error GetLaserLeadLagRange(OScDev_Setting *setting, double *min,
                                         double *max) {
    (void)setting;
    *min = 0.0;
    *max = 100.0;
    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_LaserOnLead = {
    .Release = ReleaseLaserSetting,
    .GetFloat64 = GetLaserOnLead,
    .SetFloat64 = SetLaserOnLead,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
    .GetFloat64Range = GetLaserLeadLagRange,
};

static OScDev_Error GetLaserOnLag(OScDev_Setting *setting, double *value) {
    *value = GetSettingLaser(setting)->lagUs;
    return OScDev_OK;
}

static OScDev_Error SetLaserOnLag(OScDev_Setting *setting, double value) {
    GetSettingLaser(setting)->lagUs = value;
    GetImplData(GetLaserSettingDevice(setting))
        ->scannerConfig.mustRewriteOutput = true;
    return OScDev_OK;
}

static OScDev_SettingImpl SettingImpl_LaserOnLag = {
    .Release = ReleaseLaserSetting,
    .GetFloat64 = GetLaserOnLag,
    .SetFloat64 = SetLaserOnLag,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
    .GetFloat64Range = GetLaserLeadLagRange,
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
    (void)setting; // Unused
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
    (void)setting; // Unused
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
    struct DeviceImplData *devData = GetImplData(settingData->device);
    *value = devData->channelEnabled[settingData->hwChannel];
    return OScDev_OK;
}

static OScDev_Error SetEnableChannel(OScDev_Setting *setting, bool value) {
    struct EnableChannelData *settingData =
        OScDev_Setting_GetImplData(setting);
    struct DeviceImplData *devData = GetImplData(settingData->device);
    devData->channelEnabled[settingData->hwChannel] = value;

    // Force recreation of detector task next time
    OScDev_RichError *err = ShutdownDetector(&devData->detectorConfig);
    return OScDev_Error_ReturnAsCode(err);
}

static OScDev_SettingImpl SettingImpl_EnableChannel = {
    .Release = ReleaseEnableChannel,
    .GetBool = GetEnableChannel,
    .SetBool = SetEnableChannel,
};

struct TransformSettingData {
    OScDev_Device *device;
    int index; // 0-3 = matrix[0-3], 4 = offsetX, 5 = offsetY
};

static double *GetTransformField(struct DeviceImplData *devData, int index) {
    if (index < 4)
        return &devData->xformMatrix[index];
    if (index == 4)
        return &devData->xformOffsetX;
    return &devData->xformOffsetY;
}

static OScDev_Error GetTransform(OScDev_Setting *setting, double *value) {
    struct TransformSettingData *data = OScDev_Setting_GetImplData(setting);
    *value = *GetTransformField(GetImplData(data->device), data->index);
    return OScDev_OK;
}

static OScDev_Error SetTransform(OScDev_Setting *setting, double value) {
    struct TransformSettingData *data = OScDev_Setting_GetImplData(setting);
    *GetTransformField(GetImplData(data->device), data->index) = value;

    GetImplData(data->device)->scannerConfig.mustRewriteOutput = true;

    return OScDev_OK;
}

static OScDev_Error GetTransformRange(OScDev_Setting *setting, double *min,
                                      double *max) {
    struct TransformSettingData *data = OScDev_Setting_GetImplData(setting);
    if (data->index == 0 || data->index == 3) {
        *min = 0.9;
        *max = 1.1;
    } else if (data->index < 4) {
        *min = -0.1;
        *max = 0.1;
    } else {
        *min = -5.0;
        *max = 5.0;
    }
    return OScDev_OK;
}

static void ReleaseTransform(OScDev_Setting *setting) {
    free(OScDev_Setting_GetImplData(setting));
}

static OScDev_SettingImpl SettingImpl_Transform = {
    .GetFloat64 = GetTransform,
    .SetFloat64 = SetTransform,
    .GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
    .GetFloat64Range = GetTransformRange,
    .Release = ReleaseTransform,
};

OScDev_Error NIDAQMakeSettings(OScDev_Device *device,
                               OScDev_PtrArray **settings) {
    OScDev_RichError *err;

    err = EnumerateAIPhysChans(device);
    if (err)
        return OScDev_Error_ReturnAsCode(err);

    err = EnumerateAOPhysChans(device);
    if (err)
        return OScDev_Error_ReturnAsCode(err);

    *settings = OScDev_PtrArray_Create();

    OScDev_Setting *undershoot;
    err = OScDev_Error_AsRichError(OScDev_Setting_Create(
        &undershoot, "Undershoot (us)", OScDev_ValueType_Float64,
        &SettingImpl_Undershoot, device));
    if (err)
        goto error;
    OScDev_PtrArray_Append(*settings, undershoot);

    OScDev_Setting *scanPhase;
    err = OScDev_Error_AsRichError(OScDev_Setting_Create(
        &scanPhase, "Scan Phase (us)", OScDev_ValueType_Float64,
        &SettingImpl_ScanPhase, device));
    if (err)
        goto error;
    OScDev_PtrArray_Append(*settings, scanPhase);

    OScDev_Setting *retraceScale;
    err = OScDev_Error_AsRichError(OScDev_Setting_Create(
        &retraceScale, "Retrace Scale (us/V)", OScDev_ValueType_Float64,
        &SettingImpl_RetraceScale, device));
    if (err)
        goto error;
    OScDev_PtrArray_Append(*settings, retraceScale);

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

    {
        static const char *transformNames[] = {
            "TransformA",
            "TransformB",
            "TransformC",
            "TransformD",
            "TransformOffsetXVolts",
            "TransformOffsetYVolts",
        };
        for (int i = 0; i < 6; ++i) {
            OScDev_Setting *xformSetting;
            struct TransformSettingData *data =
                malloc(sizeof(struct TransformSettingData));
            data->device = device;
            data->index = i;
            err = OScDev_Error_AsRichError(OScDev_Setting_Create(
                &xformSetting, transformNames[i], OScDev_ValueType_Float64,
                &SettingImpl_Transform, data));
            if (err)
                goto error;
            OScDev_PtrArray_Append(*settings, xformSetting);
        }
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

    {
        static const struct {
            const char *suffix;
            OScDev_ValueType type;
            OScDev_SettingImpl *impl;
        } laserSettings[] = {
            {"Control", OScDev_ValueType_Enum, &SettingImpl_LaserControl},
            {"On Voltage (V)", OScDev_ValueType_Float64,
             &SettingImpl_LaserOnVoltage},
            {"Off Voltage (V)", OScDev_ValueType_Float64,
             &SettingImpl_LaserOffVoltage},
            {"On Lead (us)", OScDev_ValueType_Float64,
             &SettingImpl_LaserOnLead},
            {"On Lag (us)", OScDev_ValueType_Float64, &SettingImpl_LaserOnLag},
        };
        int nLasers = GetNumberOfLaserChannels(device);
        for (int i = 0; i < nLasers; ++i) {
            for (size_t j = 0;
                 j < sizeof(laserSettings) / sizeof(laserSettings[0]); ++j) {
                struct LaserSettingData *data =
                    malloc(sizeof(struct LaserSettingData));
                data->device = device;
                data->laserIndex = i;
                char name[64];
                snprintf(name, sizeof(name), "LaserAO%d %s", 2 + i,
                         laserSettings[j].suffix);
                OScDev_Setting *laserSetting;
                err = OScDev_Error_AsRichError(OScDev_Setting_Create(
                    &laserSetting, name, laserSettings[j].type,
                    laserSettings[j].impl, data));
                if (err) {
                    free(data);
                    goto error;
                }
                OScDev_PtrArray_Append(*settings, laserSetting);
            }
        }
    }

    return OScDev_OK;

error:
    for (size_t i = 0; i < OScDev_PtrArray_Size(*settings); ++i) {
        OScDev_Setting_Destroy(OScDev_PtrArray_At(*settings, i));
    }
    OScDev_PtrArray_Destroy(*settings);
    *settings = NULL;
    return OScDev_Error_ReturnAsCode(err);
}
