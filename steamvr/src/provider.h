#pragma once

#include <memory>

#include <openvr_driver.h>

#include "glove_device.h"
#include "shm_reader.h"

// The SteamVR server-side provider: opens the shm and registers the two gloves
// as controllers. The udcap-server (same one Monado uses) must be running.
class UdcapProvider : public vr::IServerTrackedDeviceProvider
{
public:
	vr::EVRInitError Init(vr::IVRDriverContext *ctx) override;
	void Cleanup() override;
	const char *const *GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
	void RunFrame() override;
	bool ShouldBlockStandbyMode() override { return false; }
	void EnterStandby() override {}
	void LeaveStandby() override {}

private:
	std::shared_ptr<ShmReader> shm_;
	std::unique_ptr<GloveDevice> left_;
	std::unique_ptr<GloveDevice> right_;
};
