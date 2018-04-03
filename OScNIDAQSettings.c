#include "OScNIDAQDevicePrivate.h"
#include "OpenScanLibPrivate.h"

#include <NIDAQmx.h>

#include <string.h>


static OSc_Error GetScanRate(OSc_Setting *setting, double *value)
{
	*value = GetData(setting->device)->scanRate;
	return OSc_Error_OK;
}


static OSc_Error SetScanRate(OSc_Setting *setting, double value)
{
	GetData(setting->device)->scanRate = value;
	GetData(setting->device)->timingSettingsChanged = true;
	return OSc_Error_OK;
}


static OSc_Error GetScanRateValues(OSc_Setting *setting, double **values, size_t *count)
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
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_ScanRate = {
	.GetFloat64 = GetScanRate,
	.SetFloat64 = SetScanRate,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintDiscreteValues,
	.GetFloat64DiscreteValues = GetScanRateValues,
};


static OSc_Error GetZoom(OSc_Setting *setting, double *value)
{
	*value = GetData(setting->device)->zoom;
	return OSc_Error_OK;
}

static OSc_Error SetZoom(OSc_Setting *setting, double value)
{
	GetData(setting->device)->zoom = value;
	GetData(setting->device)->waveformSettingsChanged = true;
	return OSc_Error_OK;
}


static OSc_Error GetZoomRange(OSc_Setting *setting, double *min, double *max)
{
	*min = 1.0;
	*max = 40.0;
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_Zoom = {
	.GetFloat64 = GetZoom,
	.SetFloat64 = SetZoom,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintRange,
	.GetFloat64Range = GetZoomRange,
};

static OSc_Error GetBinFactor(OSc_Setting *setting, double *value)
{
	*value = GetData(setting->device)->binFactor;
	return OSc_Error_OK;
}

// OnBinFactor
static OSc_Error SetBinFactor(OSc_Setting *setting, double value)
{
	GetData(setting->device)->binFactor = value;
	GetData(setting->device)->timingSettingsChanged = true;
	GetData(setting->device)->acqSettingsChanged = true;
	return OSc_Error_OK;
}


static OSc_Error GetBinFactorRange(OSc_Setting *setting, uint32_t *min, uint32_t *max)
{
	*min = 1;
	*max = 25;
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_BinFactor = {
	.GetInt32 = GetBinFactor,
	.SetInt32 = SetBinFactor,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintRange,
	.GetInt32Range = GetBinFactorRange,
};

static OSc_Error GetInputVoltageRange(OSc_Setting *setting, double *value)
{
	*value = GetData(setting->device)->inputVoltageRange;
	return OSc_Error_OK;
}


static OSc_Error SetInputVoltageRange(OSc_Setting *setting, double value)
{
	GetData(setting->device)->inputVoltageRange = value;
	return OSc_Error_OK;
}


static OSc_Error GetInputVoltageRangeValues(OSc_Setting *setting, double **values, size_t *count)
{
	static double v[] = {
		1.0000,
		2.0000,
		5.0000,
		10.0000,
	};
	*values = v;
	*count = sizeof(v) / sizeof(double);
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_InputVoltageRange = {
	.GetFloat64 = GetInputVoltageRange,
	.SetFloat64 = SetInputVoltageRange,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintDiscreteValues,
	.GetFloat64DiscreteValues = GetInputVoltageRangeValues,
};


static OSc_Error GetChannels(OSc_Setting *setting, uint32_t *value)
{
	*value = GetData(setting->device)->channels;
	return OSc_Error_OK;
}


static OSc_Error SetChannels(OSc_Setting *setting, uint32_t value)
{
	GetData(setting->device)->channels = value;
	return OSc_Error_OK;
}


static OSc_Error GetChannelsNumValues(OSc_Setting *setting, uint32_t *count)
{
	*count = CHANNELS_NUM_VALUES;
	return OSc_Error_OK;
}


static OSc_Error GetChannelsNameForValue(OSc_Setting *setting, uint32_t value, char *name)
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
	case CHANNEL4:
		strcpy(name, "Channel4");
		break;
	case CHANNELS_1_AND_2:
		strcpy(name, "Channel_1_and_2");
		break;
	case CHANNELS1_2_3:
		strcpy(name, "Channel_1_2_3");
		break;
	default:
		strcpy(name, "");
		return OSc_Error_Unknown;
	}
	return OSc_Error_OK;
}

static OSc_Error GetChannelsValueForName(OSc_Setting *setting, uint32_t *value, const char *name)
{
	if (!strcmp(name, "Channel1"))
		*value = CHANNEL1;
	else if (!strcmp(name, "Channel2"))
		*value = CHANNEL2;
	else if (!strcmp(name, "Channel3"))
		*value = CHANNEL3;
	else if (!strcmp(name, "Channel4"))
		*value = CHANNEL4;
	else if (!strcmp(name, "Channel_1_and_2"))
		*value = CHANNELS_1_AND_2;
	else if (!strcmp(name, "Channel_1_2_3"))
		*value = CHANNELS1_2_3;
	else
		return OSc_Error_Unknown;
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_Channels = {
	.GetEnum = GetChannels,
	.SetEnum = SetChannels,
	.GetEnumNumValues = GetChannelsNumValues,
	.GetEnumNameForValue = GetChannelsNameForValue,
	.GetEnumValueForName = GetChannelsValueForName,
};


OSc_Error PrepareSettings(OSc_Device *device)
{
	if (GetData(device)->settings)
		return OSc_Error_OK;

	OSc_Setting *scanRate;
	OSc_Return_If_Error(OSc_Setting_Create(&scanRate, device, "ScanRate", OSc_Value_Type_Float64,
		&SettingImpl_ScanRate, NULL));

	OSc_Setting *zoom;
	OSc_Return_If_Error(OSc_Setting_Create(&zoom, device, "Zoom", OSc_Value_Type_Float64,
		&SettingImpl_Zoom, NULL));

	OSc_Setting *binFactor;
	OSc_Return_If_Error(OSc_Setting_Create(&binFactor, device, "Bin Factor", OSc_Value_Type_Int32,
		&SettingImpl_BinFactor, NULL));

	OSc_Setting *channels;
	OSc_Return_If_Error(OSc_Setting_Create(&channels, device, "Channels", OSc_Value_Type_Enum,
		&SettingImpl_Channels, NULL));

	OSc_Setting *inputVoltageRange;
	OSc_Return_If_Error(OSc_Setting_Create(&inputVoltageRange, device, "Input Voltage Range", OSc_Value_Type_Float64,
		&SettingImpl_InputVoltageRange, NULL));

	OSc_Setting *ss[] = {
		scanRate, zoom, binFactor,
		inputVoltageRange, channels,
	};
	size_t nSettings = sizeof(ss) / sizeof(OSc_Setting *);
	OSc_Setting **settings = malloc(sizeof(ss));
	memcpy(settings, ss, sizeof(ss));

	GetData(device)->settings = settings;
	GetData(device)->settingCount = nSettings;
	return OSc_Error_OK;
}