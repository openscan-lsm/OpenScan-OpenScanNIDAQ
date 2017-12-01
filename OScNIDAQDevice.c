#include "OScNIDAQDevice.h"
#include "OScNIDAQDevicePrivate.h"
#include "OpenScanLibPrivate.h"

#include <NIDAQmx.h>

#include <string.h>

#include <Windows.h>


static OSc_Device **g_devices;
static size_t g_deviceCount;


static OSc_Error NIDAQGetModelName(const char **name)
{
	*name = "OpenScan-NIDAQ";
	return OSc_Error_OK;
}


static void ParseDeviceNameList(const char *names,
	void (*callback)(const char *name))
{
	// The value of the Device Names property is a list, separated by ", "
	const char *sep = NULL;
	const char *prevSep = names;
	const char *end = names + strlen(names);
	for (;;)
	{
		// Skip any spaces after comma
		while (*prevSep == ' ')
			++prevSep;

		sep = strchr(prevSep, ',');
		if (!sep)
			sep = end;

		size_t len = sep - prevSep;
		if (len == 0 || len > OSc_MAX_STR_LEN)
			continue;
		char name[OSc_MAX_STR_LEN + 1];
		strncpy(name, prevSep, len);

		callback(name);

		if (sep == end)
			return;
		prevSep = sep;
	}
}


static void CreateDevice(const char* name)
{
	if (!g_devices)
	{
		g_devices = malloc(sizeof(OSc_Device *));
	}
	else
	{
		g_devices = realloc(g_devices, g_deviceCount * sizeof(OSc_Device *));
	}

	struct OScNIDAQPrivateData *data = calloc(1, sizeof(struct OScNIDAQPrivateData));

	OSc_Device *device;
	OSc_Error err;
	if (OSc_Check_Error(err, OSc_Device_Create(&device, &OpenScan_NIDAQ_Device_Impl, data)))
	{
		char msg[OSc_MAX_STR_LEN + 1] = "Failed to create device ";
		strcat(msg, name);
		OSc_Log_Error(device, msg);
		return;
	}

	strncpy(GetData(device)->deviceName, name, OSc_MAX_STR_LEN);

	g_devices[g_deviceCount++] = device;
}


static OSc_Error EnumerateInstances(void)
{
	if (g_devices)
		return OSc_Error_OK;

	char deviceNames[4096];
	int32 nierr = DAQmxGetSysDevNames(deviceNames, sizeof(deviceNames));
	if (nierr)
	{
		OSc_Log_Error(NULL, "Failed to enumerate NI DAQ devices");
		return OSc_Error_Unknown;
	}

	ParseDeviceNameList(deviceNames, CreateDevice);

	return OSc_Error_OK;
}


static OSc_Error NIDAQGetInstances(OSc_Device ***devices, size_t *count)
{
	OSc_Return_If_Error(EnumerateInstances());
	*devices = g_devices;
	*count = g_deviceCount;
	return OSc_Error_OK;
}


static OSc_Error NIDAQReleaseInstance(OSc_Device *device)
{
	free(GetData(device));
	device->implData = NULL;
	return OSc_Error_OK;
}


static OSc_Error NIDAQGetName(OSc_Device *device, char *name)
{
	strncpy(name, GetData(device)->deviceName, OSc_MAX_STR_LEN);
	return OSc_Error_OK;
}


static OSc_Error NIDAQOpen(OSc_Device *device)
{
	int32 nierr = DAQmxResetDevice(GetData(device)->deviceName);
	if (nierr)
	{
		char msg[OSc_MAX_STR_LEN + 1] = "Cannot reset NI DAQ card ";
		strcat(msg, GetData(device)->deviceName);
		OSc_Log_Error(device, msg);
		return OSc_Error_Unknown; // TODO Detailed info
	}

	return OSc_Error_OK;
}


static OSc_Error NIDAQClose(OSc_Device *device)
{
	// TODO Stop and destroy all tasks
	return OSc_Error_OK;
}


static OSc_Error NIDAQHasScanner(OSc_Device *device, bool *hasScanner)
{
	*hasScanner = true;
	return OSc_Error_OK;
}


static OSc_Error NIDAQHasDetector(OSc_Device *device, bool *hasDetector)
{
	*hasDetector = true;
	return OSc_Error_OK;
}


struct OSc_Device_Impl OpenScan_NIDAQ_Device_Impl = {
	.GetModelName = NIDAQGetModelName,
	.GetInstances = NIDAQGetInstances,
	.ReleaseInstance = NIDAQReleaseInstance,
	.GetName = NIDAQGetName,
	.Open = NIDAQOpen,
	.Close = NIDAQClose,
	.HasScanner = NIDAQHasScanner,
	.HasDetector = NIDAQHasDetector,
};