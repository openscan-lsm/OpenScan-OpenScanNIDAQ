#include "NIDAQ_DeviceImplData.h"

#include <ss8str.h>

#include <string.h>

#include <Windows.h>

void InitializeImplData(struct DeviceImplData *data) {
    memset(data, 0, sizeof(*data));

    ss8_init(&data->deviceName);
    data->lineDelay = 50;
    data->numLinesToBuffer = 8;
    data->inputVoltageRange = 10.0;
    data->minVolts_ = -10.0;
    data->maxVolts_ = 10.0;
    ss8_init(&data->aiPhysChans);
    data->channelEnabled[0] = true;

    InitializeCriticalSection(&(data->acquisition.mutex));
    InitializeConditionVariable(
        &(data->acquisition.acquisitionFinishCondition));
}
