#include <cstring>

#include <openvr_driver.h>

#include "provider.h"

static UdcapProvider g_provider;

extern "C" __attribute__((visibility("default"))) void *
HmdDriverFactory(const char *interface_name, int *return_code)
{
	if (std::strcmp(interface_name, vr::IServerTrackedDeviceProvider_Version) == 0) {
		return &g_provider;
	}
	if (return_code) {
		*return_code = vr::VRInitError_Init_InterfaceNotFound;
	}
	return nullptr;
}
