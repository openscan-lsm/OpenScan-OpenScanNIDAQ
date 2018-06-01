#include "OScNIDAQDevicePrivate.h"
#include "OpenScanLibPrivate.h"

#include <NIDAQmx.h>

#include <string.h>

const char* const PROPERTY_VALUE_Channel1 = "Channel1";
const char* const PROPERTY_VALUE_Channel2 = "Channel2";
const char* const PROPERTY_VALUE_Channel3 = "Channel3";
const char* const PROPERTY_VALUE_Channel1and2 = "Channel1and2";
const char* const PROPERTY_VALUE_Channel1and3 = "Channel1and3";
const char* const PROPERTY_VALUE_Channel1and2and3 = "Channels1-3";

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

	GetData(setting->device)->magnification =
		(double)GetData(setting->device)->resolution / (double)OSc_DEFAULT_RESOLUTION 
		* GetData(setting->device)->zoom / OSc_DEFAULT_ZOOM;

	return OSc_Error_OK;
}

static OSc_Error SetZoom(OSc_Setting *setting, double value)
{
	GetData(setting->device)->zoom = value;
	GetData(setting->device)->waveformSettingsChanged = true;

	// reflect the change to magnification as well
	GetData(setting->device)->magnification = 
		(double)GetData(setting->device)->resolution / (double)OSc_DEFAULT_RESOLUTION 
		* value / OSc_DEFAULT_ZOOM;

	return OSc_Error_OK;
}


static OSc_Error GetZoomRange(OSc_Setting *setting, double *min, double *max)
{
	*min = 0.2;
	*max = 20.0;
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_Zoom = {
	.GetFloat64 = GetZoom,
	.SetFloat64 = SetZoom,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintRange,
	.GetFloat64Range = GetZoomRange,
};

static OSc_Error GetBinFactor(OSc_Setting *setting, int32_t *value)
{
	*value = GetData(setting->device)->binFactor;
	return OSc_Error_OK;
}

// OnBinFactor
static OSc_Error SetBinFactor(OSc_Setting *setting, int32_t value)
{
	GetData(setting->device)->binFactor = value;
	GetData(setting->device)->timingSettingsChanged = true;
	GetData(setting->device)->acqSettingsChanged = true;
	return OSc_Error_OK;
}


static OSc_Error GetBinFactorRange(OSc_Setting *setting, int32_t *min, int32_t *max)
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

static OSc_Error GetAcqBufferSize(OSc_Setting *setting, int32_t *value)
{
	*value = GetData(setting->device)->numLinesToBuffer;
	return OSc_Error_OK;
}

// OnAcqBufferSize
static OSc_Error SetAcqBufferSize(OSc_Setting *setting, int32_t value)
{
	GetData(setting->device)->numLinesToBuffer = value;
	GetData(setting->device)->timingSettingsChanged = true;
	GetData(setting->device)->acqSettingsChanged = true;
	return OSc_Error_OK;
}

static OSc_Error GetAcqBufferSizeValues(OSc_Setting *setting, int32_t **values, size_t *count)
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
	return OSc_Error_OK;
}

static struct OSc_Setting_Impl SettingImpl_AcqBufferSize = {
	.GetInt32 = GetAcqBufferSize,
	.SetInt32 = SetAcqBufferSize,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintDiscreteValues,
	.GetInt32DiscreteValues = GetAcqBufferSizeValues,
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
	GetData(setting->device)->channelSettingsChanged = true;
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
	case CHANNELS_1_AND_3:
		strcpy(name, "Channel_1_and_3");
		break;
	case CHANNELS1_2_3:
		strcpy(name, "Channel_1_2_3");
		break;
	default:
		strcpy(name, "");
		return OSc_Error_Unknown;
	}
	OSc_Error err;
	if (OSc_Check_Error(err, GetSelectedDispChannels(setting->device))) {
		OSc_Log_Error(setting->device, "Fail to get selected disp channels");
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
	else if (!strcmp(name, "Channel_1_and_3"))
		*value = CHANNELS_1_AND_3;
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

static OSc_Error GetScannerOnly(OSc_Setting *setting, bool *value)
{
	*value = GetData(setting->device)->scannerOnly;
	return OSc_Error_OK;
}

static OSc_Error SetScannerOnly(OSc_Setting *setting, bool value)
{
	GetData(setting->device)->scannerOnly = value;
	return OSc_Error_OK;
}

static struct OSc_Setting_Impl SettingImpl_ScannerOnly = {
	.GetBool = GetScannerOnly,
	.SetBool = SetScannerOnly,
};

static OSc_Error GetOffset(OSc_Setting *setting, double *value)
{
	*value = GetData(setting->device)->offsetXY[(intptr_t)(setting->implData)];
	return OSc_Error_OK;
}


static OSc_Error SetOffset(OSc_Setting *setting, double value)
{
	GetData(setting->device)->offsetXY[(intptr_t)(setting->implData)] = value;
	GetData(setting->device)->waveformSettingsChanged = true;
	GetData(setting->device)->settingsChanged = true;
	return OSc_Error_OK;
}


static OSc_Error GetOffsetRange(OSc_Setting *setting, double *min, double *max)
{
	/*The galvoOffsetX and galvoOffsetY variables are expressed  in optical degrees
	This is a rough correspondence - it likely needs to be calibrated to the actual
	sensitivity of the galvos*/
	*min = -5.0;
	*max = +5.0;
	return OSc_Error_OK;
}


static struct OSc_Setting_Impl SettingImpl_Offset = {
	.GetFloat64 = GetOffset,
	.SetFloat64 = SetOffset,
	.GetNumericConstraintType = OSc_Setting_NumericConstraintRange,
	.GetFloat64Range = GetOffsetRange,
};




OSc_Error NIDAQ_PrepareSettings(OSc_Device *device)
{
	if (GetData(device)->settings)
		return OSc_Error_OK;

	OSc_Setting *scanRate;
	OSc_Return_If_Error(OSc_Setting_Create(&scanRate, device, "ScanRate", OSc_Value_Type_Float64,
		&SettingImpl_ScanRate, NULL));

	OSc_Setting *zoom;
	OSc_Return_If_Error(OSc_Setting_Create(&zoom, device, "Zoom", OSc_Value_Type_Float64,
		&SettingImpl_Zoom, NULL));

	OSc_Setting *offsetX;
	OSc_Return_If_Error(OSc_Setting_Create(&offsetX, device, "GalvoOffsetX (degree)", OSc_Value_Type_Float64,
		&SettingImpl_Offset, (void *)0));

	OSc_Setting *offsetY;
	OSc_Return_If_Error(OSc_Setting_Create(&offsetY, device, "GalvoOffsetY (degree)", OSc_Value_Type_Float64,
		&SettingImpl_Offset, (void *)1));

	OSc_Setting *binFactor;
	OSc_Return_If_Error(OSc_Setting_Create(&binFactor, device, "Bin Factor", OSc_Value_Type_Int32,
		&SettingImpl_BinFactor, NULL));

	OSc_Setting *numLinesToBuffer;
	OSc_Return_If_Error(OSc_Setting_Create(&numLinesToBuffer, device, "Acq Buffer Size (lines)", OSc_Value_Type_Int32,
		&SettingImpl_AcqBufferSize, NULL));

	OSc_Setting *channels;
	OSc_Return_If_Error(OSc_Setting_Create(&channels, device, "Channels", OSc_Value_Type_Enum,
		&SettingImpl_Channels, NULL));

	OSc_Setting *inputVoltageRange;
	OSc_Return_If_Error(OSc_Setting_Create(&inputVoltageRange, device, "Input Voltage Range", OSc_Value_Type_Float64,
		&SettingImpl_InputVoltageRange, NULL));

	OSc_Setting *scannerOnly;
	OSc_Return_If_Error(OSc_Setting_Create(&scannerOnly, device, "ScannerOnly", OSc_Value_Type_Bool,
		&SettingImpl_ScannerOnly, NULL));

	OSc_Setting *ss[] = {
		scanRate, zoom, offsetX, offsetY, binFactor, numLinesToBuffer,
		inputVoltageRange, channels, scannerOnly,
	};
	size_t nSettings = sizeof(ss) / sizeof(OSc_Setting *);
	OSc_Setting **settings = malloc(sizeof(ss));
	memcpy(settings, ss, sizeof(ss));

	GetData(device)->settings = settings;
	GetData(device)->settingCount = nSettings;
	return OSc_Error_OK;
}

static OSc_Error GetSelectedDispChannels(OSc_Device *device)
{
	// clear selectedDispChan
	GetData(device)->selectedDispChan_ = calloc(OSc_Total_Channel_Num * (OSc_MAX_STR_LEN + 1), sizeof(char));
	//memset(GetData(device)->selectedDispChan_, 0, sizeof(char) * 3 * (OSc_MAX_STR_LEN + 1));

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

	return OSc_Error_OK;
}
