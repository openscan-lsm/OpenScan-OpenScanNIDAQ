#include "OScNIDAQDevicePrivate.h"

#include <NIDAQmx.h>

#include <string.h>

const char* const PROPERTY_VALUE_Channel1 = "Channel1";
const char* const PROPERTY_VALUE_Channel2 = "Channel2";
const char* const PROPERTY_VALUE_Channel3 = "Channel3";
const char* const PROPERTY_VALUE_Channel1and2 = "Channel1and2";
const char* const PROPERTY_VALUE_Channel1and3 = "Channel1and3";
const char* const PROPERTY_VALUE_Channel1and2and3 = "Channels1-3";


// For most settings, we set the setting's implData to the device.
// This function can then be used to retrieve the device implData.
static inline struct OScNIDAQPrivateData *GetSettingDeviceData(OScDev_Setting *setting)
{
	return (struct OScNIDAQPrivateData *)OScDev_Device_GetImplData((OScDev_Device *)OScDev_Setting_GetImplData(setting));
}


OScDev_Error GetNumericConstraintTypeImpl_DiscreteValues(OScDev_Setting *setting, enum OScDev_ValueConstraint *constraintType)
{
	*constraintType = OScDev_ValueConstraint_DiscreteValues;
	return OScDev_OK;
}


OScDev_Error GetNumericConstraintTypeImpl_Range(OScDev_Setting *setting, enum OScDev_ValueConstraint *constraintType)
{
	*constraintType = OScDev_ValueConstraint_Range;
	return OScDev_OK;
}


static OScDev_Error GetScanRate(OScDev_Setting *setting, double *value)
{
	*value = GetSettingDeviceData(setting)->scanRate;
	return OScDev_OK;
}


static OScDev_Error SetScanRate(OScDev_Setting *setting, double value)
{
	GetSettingDeviceData(setting)->scanRate = value;

	GetSettingDeviceData(setting)->clockConfig.mustReconfigureTiming = true;
	GetSettingDeviceData(setting)->scannerConfig.mustReconfigureTiming = true;
	GetSettingDeviceData(setting)->detectorConfig.mustReconfigureTiming = true;

	return OScDev_OK;
}


static OScDev_Error GetScanRateValues(OScDev_Setting *setting, double **values, size_t *count)
{
	static double v[] = {
		0.0500,
		0.1000,
		0.1250,
		0.2000,
		0.2500,
		0.4000,
		0.5000,
		0.6250,
		1.0000,
		1.2500,
	};
	*values = v;
	*count = sizeof(v) / sizeof(double);
	return OScDev_OK;
}


static struct OScDev_SettingImpl SettingImpl_ScanRate = {
	.GetFloat64 = GetScanRate,
	.SetFloat64 = SetScanRate,
	.GetNumericConstraintType = GetNumericConstraintTypeImpl_DiscreteValues,
	.GetFloat64DiscreteValues = GetScanRateValues,
};


static OScDev_Error GetZoom(OScDev_Setting *setting, double *value)
{
	*value = GetSettingDeviceData(setting)->zoom;

	GetSettingDeviceData(setting)->magnification =
		(double)GetSettingDeviceData(setting)->resolution / (double)OSc_DEFAULT_RESOLUTION
		* GetSettingDeviceData(setting)->zoom / OSc_DEFAULT_ZOOM;

	return OScDev_OK;
}

static OScDev_Error SetZoom(OScDev_Setting *setting, double value)
{
	GetSettingDeviceData(setting)->zoom = value;

	GetSettingDeviceData(setting)->clockConfig.mustRewriteOutput = true;
	GetSettingDeviceData(setting)->scannerConfig.mustRewriteOutput = true;

	// reflect the change to magnification as well
	GetSettingDeviceData(setting)->magnification =
		(double)GetSettingDeviceData(setting)->resolution / (double)OSc_DEFAULT_RESOLUTION
		* value / OSc_DEFAULT_ZOOM;

	return OScDev_OK;
}


static OScDev_Error GetZoomRange(OScDev_Setting *setting, double *min, double *max)
{
	*min = 0.2;
	*max = 20.0;
	return OScDev_OK;
}


static struct OScDev_SettingImpl SettingImpl_Zoom = {
	.GetFloat64 = GetZoom,
	.SetFloat64 = SetZoom,
	.GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
	.GetFloat64Range = GetZoomRange,
};

static OScDev_Error GetBinFactor(OScDev_Setting *setting, int32_t *value)
{
	*value = GetSettingDeviceData(setting)->binFactor;
	return OScDev_OK;
}

// OnBinFactor
static OScDev_Error SetBinFactor(OScDev_Setting *setting, int32_t value)
{
	GetSettingDeviceData(setting)->binFactor = value;

	GetSettingDeviceData(setting)->clockConfig.mustReconfigureTiming = true;
	GetSettingDeviceData(setting)->scannerConfig.mustReconfigureTiming = true;
	GetSettingDeviceData(setting)->detectorConfig.mustReconfigureTiming = true;
	GetSettingDeviceData(setting)->detectorConfig.mustReconfigureCallback = true;

	return OScDev_OK;
}


static OScDev_Error GetBinFactorRange(OScDev_Setting *setting, int32_t *min, int32_t *max)
{
	*min = 1;
	*max = 25;
	return OScDev_OK;
}

static struct OScDev_SettingImpl SettingImpl_BinFactor = {
	.GetInt32 = GetBinFactor,
	.SetInt32 = SetBinFactor,
	.GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
	.GetInt32Range = GetBinFactorRange,
};

static OScDev_Error GetAcqBufferSize(OScDev_Setting *setting, int32_t *value)
{
	*value = GetSettingDeviceData(setting)->numLinesToBuffer;
	return OScDev_OK;
}

// OnAcqBufferSize
static OScDev_Error SetAcqBufferSize(OScDev_Setting *setting, int32_t value)
{
	GetSettingDeviceData(setting)->numLinesToBuffer = value;

	GetSettingDeviceData(setting)->clockConfig.mustReconfigureTiming = true;
	GetSettingDeviceData(setting)->scannerConfig.mustReconfigureTiming = true;
	GetSettingDeviceData(setting)->detectorConfig.mustReconfigureTiming = true;
	GetSettingDeviceData(setting)->detectorConfig.mustReconfigureCallback = true;

	return OScDev_OK;
}

static OScDev_Error GetAcqBufferSizeValues(OScDev_Setting *setting, int32_t **values, size_t *count)
{
	static int32_t v[] = {
		2,
		4,
		8,
		16,
		32,
		64,
		128,
		256,
	};
	*values = v;
	*count = sizeof(v) / sizeof(int32_t);
	return OScDev_OK;
}

static struct OScDev_SettingImpl SettingImpl_AcqBufferSize = {
	.GetInt32 = GetAcqBufferSize,
	.SetInt32 = SetAcqBufferSize,
	.GetNumericConstraintType = GetNumericConstraintTypeImpl_DiscreteValues,
	.GetInt32DiscreteValues = GetAcqBufferSizeValues,
};

static OScDev_Error GetInputVoltageRange(OScDev_Setting *setting, double *value)
{
	*value = GetSettingDeviceData(setting)->inputVoltageRange;
	return OScDev_OK;
}


static OScDev_Error SetInputVoltageRange(OScDev_Setting *setting, double value)
{
	GetSettingDeviceData(setting)->inputVoltageRange = value;
	return OScDev_OK;
}


static OScDev_Error GetInputVoltageRangeValues(OScDev_Setting *setting, double **values, size_t *count)
{
	static double v[] = {
		1.0000,
		2.0000,
		5.0000,
		10.0000,
	};
	*values = v;
	*count = sizeof(v) / sizeof(double);
	return OScDev_OK;
}


static struct OScDev_SettingImpl SettingImpl_InputVoltageRange = {
	.GetFloat64 = GetInputVoltageRange,
	.SetFloat64 = SetInputVoltageRange,
	.GetNumericConstraintType = GetNumericConstraintTypeImpl_DiscreteValues,
	.GetFloat64DiscreteValues = GetInputVoltageRangeValues,
};


static OScDev_Error GetChannels(OScDev_Setting *setting, uint32_t *value)
{
	*value = GetSettingDeviceData(setting)->channels;
	return OScDev_OK;
}


static OScDev_Error SetChannels(OScDev_Setting *setting, uint32_t value)
{
	GetSettingDeviceData(setting)->channels = value;

	// Force recreation of detector task next time
	OScDev_Device *device = (OScDev_Device *)OScDev_Setting_GetImplData(setting);
	OScDev_Error err = ShutdownDetector(device,
		&GetSettingDeviceData(setting)->detectorConfig);

	return OScDev_OK;
}


static OScDev_Error GetChannelsNumValues(OScDev_Setting *setting, uint32_t *count)
{
	*count = CHANNELS_NUM_VALUES;
	return OScDev_OK;
}


static OScDev_Error GetChannelsNameForValue(OScDev_Setting *setting, uint32_t value, char *name)
{
	switch (value)
	{
	case CHANNEL1:
		strcpy(name, "Channel1");
		break;
	case CHANNEL2:
		strcpy(name, "Channel2");
		break;
	case CHANNEL3:
		strcpy(name, "Channel3");
		break;
		break;
	case CHANNELS_1_AND_2:
		strcpy(name, "Channel_1_and_2");
		break;
	case CHANNELS_1_AND_3:
		strcpy(name, "Channel_1_and_3");
		break;
	case CHANNELS1_2_3:
		strcpy(name, "Channel_1_2_3");
		break;
	default:
		strcpy(name, "");
		return OScDev_Error_Unknown;
	}

	OScDev_Device *device = (OScDev_Device *)OScDev_Setting_GetImplData(setting);
	OScDev_Error err;
	if (OScDev_CHECK(err, GetSelectedDispChannels(device))) {
		OScDev_Log_Error(device, "Fail to get selected disp channels");
	}
	return OScDev_OK;
}

static OScDev_Error GetChannelsValueForName(OScDev_Setting *setting, uint32_t *value, const char *name)
{
	if (!strcmp(name, "Channel1"))
		*value = CHANNEL1;
	else if (!strcmp(name, "Channel2"))
		*value = CHANNEL2;
	else if (!strcmp(name, "Channel3"))
		*value = CHANNEL3;
	else if (!strcmp(name, "Channel_1_and_2"))
		*value = CHANNELS_1_AND_2;
	else if (!strcmp(name, "Channel_1_and_3"))
		*value = CHANNELS_1_AND_3;
	else if (!strcmp(name, "Channel_1_2_3"))
		*value = CHANNELS1_2_3;
	else
		return OScDev_Error_Unknown;
	return OScDev_OK;
}


static struct OScDev_SettingImpl SettingImpl_Channels = {
	.GetEnum = GetChannels,
	.SetEnum = SetChannels,
	.GetEnumNumValues = GetChannelsNumValues,
	.GetEnumNameForValue = GetChannelsNameForValue,
	.GetEnumValueForName = GetChannelsValueForName,
};

static OScDev_Error GetScannerOnly(OScDev_Setting *setting, bool *value)
{
	*value = GetSettingDeviceData(setting)->scannerOnly;
	return OScDev_OK;
}

static OScDev_Error SetScannerOnly(OScDev_Setting *setting, bool value)
{
	GetSettingDeviceData(setting)->scannerOnly = value;
	return OScDev_OK;
}

static struct OScDev_SettingImpl SettingImpl_ScannerOnly = {
	.GetBool = GetScannerOnly,
	.SetBool = SetScannerOnly,
};


struct OffsetSettingData
{
	OScDev_Device *device;
	int axis; // 0 = x, 1 = y
};


static OScDev_Error GetOffset(OScDev_Setting *setting, double *value)
{
	struct OffsetSettingData *data = (struct OffsetSettingData *)OScDev_Setting_GetImplData(setting);
	*value = GetData(data->device)->offsetXY[data->axis];
	return OScDev_OK;
}


static OScDev_Error SetOffset(OScDev_Setting *setting, double value)
{
	struct OffsetSettingData *data = (struct OffsetSettingData *)OScDev_Setting_GetImplData(setting);
	GetData(data->device)->offsetXY[data->axis] = value;

	GetData(data->device)->clockConfig.mustRewriteOutput = true;
	GetData(data->device)->scannerConfig.mustRewriteOutput = true;

	return OScDev_OK;
}


static OScDev_Error GetOffsetRange(OScDev_Setting *setting, double *min, double *max)
{
	/*The galvoOffsetX and galvoOffsetY variables are expressed  in optical degrees
	This is a rough correspondence - it likely needs to be calibrated to the actual
	sensitivity of the galvos*/
	*min = -5.0;
	*max = +5.0;
	return OScDev_OK;
}


static struct OScDev_SettingImpl SettingImpl_Offset = {
	.GetFloat64 = GetOffset,
	.SetFloat64 = SetOffset,
	.GetNumericConstraintType = GetNumericConstraintTypeImpl_Range,
	.GetFloat64Range = GetOffsetRange,
};


OScDev_Error NIDAQ_PrepareSettings(OScDev_Device *device)
{
	if (GetData(device)->settings)
		return OScDev_OK;

	OScDev_Error err;

	OScDev_Setting *scanRate;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&scanRate, "ScanRate", OScDev_ValueType_Float64,
		&SettingImpl_ScanRate, device)))
		return err;

	OScDev_Setting *zoom;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&zoom, "Zoom", OScDev_ValueType_Float64,
		&SettingImpl_Zoom, device)))
		return err;

	OScDev_Setting *offsets[2];
	for (int i = 0; i < 2; ++i)
	{
		// TODO We currently never free the OffsetSettingData allocated here.
		// Call free() once OpenScanDeviceLib supports a method to release setting data.
		struct OffsetSettingData *data = malloc(sizeof(struct OffsetSettingData));
		data->device = device;
		data->axis = i;
		const char *name = i == 0 ? "GalvoOffsetX (degree)" : "GalvoOffsetY (degree)";
		if (OScDev_CHECK(err, OScDev_Setting_Create(&offsets[i], name, OScDev_ValueType_Float64,
			&SettingImpl_Offset, data)))
			return err;
	}

	OScDev_Setting *binFactor;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&binFactor, "Bin Factor", OScDev_ValueType_Int32,
		&SettingImpl_BinFactor, device)))
		return err;

	OScDev_Setting *numLinesToBuffer;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&numLinesToBuffer, "Acq Buffer Size (lines)", OScDev_ValueType_Int32,
		&SettingImpl_AcqBufferSize, device)))
		return err;

	OScDev_Setting *channels;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&channels, "Channels", OScDev_ValueType_Enum,
		&SettingImpl_Channels, device)))
		return err;

	OScDev_Setting *inputVoltageRange;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&inputVoltageRange, "Input Voltage Range", OScDev_ValueType_Float64,
		&SettingImpl_InputVoltageRange, device)))
		return err;

	OScDev_Setting *scannerOnly;
	if (OScDev_CHECK(err, OScDev_Setting_Create(&scannerOnly, "ScannerOnly", OScDev_ValueType_Bool,
		&SettingImpl_ScannerOnly, device)))
		return err;

	OScDev_Setting *ss[] = {
		scanRate, zoom, offsets[0], offsets[1], binFactor, numLinesToBuffer,
		inputVoltageRange, channels, scannerOnly,
	};
	size_t nSettings = sizeof(ss) / sizeof(OScDev_Setting *);
	OScDev_Setting **settings = malloc(sizeof(ss));
	memcpy(settings, ss, sizeof(ss));

	GetData(device)->settings = settings;
	GetData(device)->settingCount = nSettings;
	return OScDev_OK;
}

static OScDev_Error GetSelectedDispChannels(OScDev_Device *device)
{
	// clear selectedDispChan
	GetData(device)->selectedDispChan_ = calloc(OSc_Total_Channel_Num * (OScDev_MAX_STR_LEN + 1), sizeof(char));

	switch (GetData(device)->channels)
	{
	case CHANNEL1:
		GetData(device)->selectedDispChan_[0] = PROPERTY_VALUE_Channel1;
		GetData(device)->channelCount = 1;
		break;
	case CHANNEL2:
		GetData(device)->selectedDispChan_[0] = PROPERTY_VALUE_Channel2;
		GetData(device)->channelCount = 1;
		break;
	case CHANNEL3:
		GetData(device)->selectedDispChan_[0] = PROPERTY_VALUE_Channel3;
		GetData(device)->channelCount = 1;
		break;
	case CHANNELS_1_AND_2:
		GetData(device)->selectedDispChan_[0] = PROPERTY_VALUE_Channel1;
		GetData(device)->selectedDispChan_[1] = PROPERTY_VALUE_Channel2;
		GetData(device)->channelCount = 2;
		break;
	case CHANNELS_1_AND_3:
		GetData(device)->selectedDispChan_[0] =  PROPERTY_VALUE_Channel1;
		GetData(device)->selectedDispChan_[1] = PROPERTY_VALUE_Channel3;
		GetData(device)->channelCount = 2;
		break;
	case CHANNELS1_2_3:
		GetData(device)->selectedDispChan_[0] = PROPERTY_VALUE_Channel1;
		GetData(device)->selectedDispChan_[1] = PROPERTY_VALUE_Channel2;
		GetData(device)->selectedDispChan_[2] = PROPERTY_VALUE_Channel3;
		GetData(device)->channelCount = 3;
		break;
	}

	return OScDev_OK;
}
