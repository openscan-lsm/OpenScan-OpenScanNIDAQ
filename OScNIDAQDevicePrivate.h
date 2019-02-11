#pragma once
#include "strmap.h"
#include "OScNIDAQDevice.h"

#include "OpenScanDeviceLib.h"

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

#define OSc_DEFAULT_RESOLUTION 512
#define OSc_DEFAULT_ZOOM 1.0
#define OSc_Total_Channel_Num 3


struct OScNIDAQPrivateData
{
	char rioResourceName[OScDev_MAX_STR_LEN + 1];
	char deviceName[OScDev_MAX_STR_LEN + 1];

	OScDev_Setting **settings;
	size_t settingCount;

	TaskHandle  scanWaveformTaskHandle_, lineClockTaskHandle_, acqTaskHandle_, 
		counterTaskHandle_, pixelClockTaskHandle_;
	bool settingsChanged;	
	// True when resolution, scanRate,or binFactor have changed since last acquisition
	bool timingSettingsChanged;
	// True when resolution or zoom have changed since last acq
	bool waveformSettingsChanged;
	// True when resolution or binFactor have changed since last acq
	bool acqSettingsChanged;
	bool channelSettingsChanged;
	bool isEveryNSamplesEventRegistered;
	bool oneFrameScanDone;
	// Flags for scanner and detector
	bool detectorOnly;
	bool scannerOnly;

	double scanRate;  // MHz
	double zoom;
	uint32_t resolution;
	double magnification;  // =(resolution/512) * (zoom/1)
	int32_t binFactor;
	int32_t numLinesToBuffer;
	double inputVoltageRange;
	int32_t totalRead;
	uInt32 numAIChannels;
	uInt32 numDOChannels; // reserved for multiple line and frame clocks
	double offsetXY[2];
	double minVolts_; // min possible for device
	double maxVolts_; // max possible for device
	uint32_t channelCount;
	
	//char* niDAQname_; // DAQ used for OpenScan
	char** aiPorts_;  // std::vector<std::string> aiPorts_
	char* aoChanList_; 
	char* doChanList_; 
	char* coChanList_; 
	char* acqTrigPort_;
	const char** selectedDispChan_; 
	char* enabledAIPorts_;
	StrMap* channelMap_;


	enum {
		CHANNEL1,
		CHANNEL2,
		CHANNEL3,
		CHANNELS_1_AND_2,
		CHANNELS_1_AND_3,
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
		OScDev_Acquisition *acquisition;
	} acquisition;
};


static inline struct OScNIDAQPrivateData *GetData(OScDev_Device *device)
{
	return (struct OScNIDAQPrivateData *)OScDev_Device_GetImplData(device);
}


OScDev_Error NIDAQ_PrepareSettings(OScDev_Device *device);
OScDev_Error GetSelectedDispChannels(OScDev_Device *device);