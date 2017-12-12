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

	TaskHandle  scanWaveformTaskHandle_, lineClockTaskHandle_, acqTaskHandle_,
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

	double scanRate;  // MHz
	double zoom;
	uint32_t resolution;
	uint32_t binFactor;
	double inputVoltageRange;

	uInt32 numDOChannels; // reserved for multiple line and frame clocks

	enum {
		CHANNELS_RAW_IMAGE,
		CHANNELS_KALMAN_AVERAGED,
		CHANNELS_RAW_AND_KALMAN,
		
		CHANNEL1,
		CHANNEL2,
		CHANNELS_1_AND_2,

		CHANNELS_NUM_VALUES,
	} channels;


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


OSc_Error PrepareSettings(OSc_Device *device);