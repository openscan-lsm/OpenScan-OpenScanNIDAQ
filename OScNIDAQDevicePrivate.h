#pragma once

#include "OScNIDAQDevice.h"

#include "OpenScanLibPrivate.h"
#include "OpenScanDeviceImpl.h"

#include <NIDAQmx.h>

#include <Windows.h>

enum
{
	DAQ_STATE_IDLE,
	DAQ_STATE_INIT,
	DAQ_STATE_WRITE,
	DAQ_STATE_SCAN,
	DAQ_STATE_DONE,
	DAQ_STATE_STOP,
};


struct OScNIDAQPrivateData
{
	char rioResourceName[OSc_MAX_STR_LEN + 1];
	char deviceName[OSc_MAX_STR_LEN + 1];

	OSc_Setting **settings;
	size_t settingCount;

	TaskHandle  scanWaveformTaskHandle_, lineClockTaskHandle_, acqTaskHandle_, counterTaskHandle_,
		lineClockFLIMTaskHandle_, frameClockFLIMTaskHandle_;
	bool settingsChanged;	
	// True when resolution, scanRate,or binFactor have changed since last acquisition
	bool timingSettingsChanged;
	// True when resolution or zoom have changed since last acq
	bool waveformSettingsChanged;
	// True when resolution or binFactor have changed since last acq
	bool acqSettingsChanged;
	bool isEveryNSamplesEventRegistered;
	bool oneFrameScanDone;
	// Flags for scanner and detector
	bool detectorOnly;
	bool scannerOnly;
	double scanRate;  // MHz
	double zoom;
	uint32_t resolution;
	int32_t binFactor;
	double inputVoltageRange;
	int32_t totalRead;
	uInt32 numDOChannels; // reserved for multiple line and frame clocks

	enum {
		CHANNEL1,
		CHANNEL2,
		CHANNEL3,
		CHANNEL4,
		CHANNELS_1_AND_2,
		CHANNELS1_2_3,
		CHANNELS_NUM_VALUES
	} channels;

	uint16_t* ch1Buffer;
	uint16_t* ch2Buffer;
	uint16_t* ch3Buffer;
	float64* rawLineData;
	float64* avgLineData;
	uint16_t* imageData; // whole-frame image of all channels

	struct
	{
		CRITICAL_SECTION mutex;
		HANDLE thread;
		CONDITION_VARIABLE acquisitionFinishCondition;
		bool running;
		bool armed; // Valid when running == true
		bool started; // Valid when running == true
		bool stopRequested; // Valid when running == true
		int32 totalRead;
		uInt32 numAIChannels;
		OSc_Acquisition *acquisition;
	} acquisition;
};


static inline struct OScNIDAQPrivateData *GetData(OSc_Device *device)
{
	return (struct OScNIDAQPrivateData *)(device->implData);
}


OSc_Error NIDAQ_PrepareSettings(OSc_Device *device);