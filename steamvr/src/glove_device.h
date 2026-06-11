// One UDCAP glove presented to SteamVR as a Valve Index Knuckles controller.
#pragma once

#include <memory>
#include <string>

#include <openvr_driver.h>

#include "hand_simulation.h"
#include "shm_reader.h"

class GloveDevice : public vr::ITrackedDeviceServerDriver
{
public:
	GloveDevice(vr::ETrackedControllerRole role, std::shared_ptr<ShmReader> shm);

	const char *serial() const { return serial_.c_str(); }

	// ITrackedDeviceServerDriver
	vr::EVRInitError Activate(uint32_t object_id) override;
	void Deactivate() override { object_id_ = vr::k_unTrackedDeviceIndexInvalid; }
	void EnterStandby() override {}
	void *GetComponent(const char *) override { return nullptr; }
	void DebugRequest(const char *, char *resp, uint32_t size) override
	{
		if (size) {
			resp[0] = 0;
		}
	}
	vr::DriverPose_t GetPose() override;

	// Called from the provider's RunFrame: push pose + inputs from the shm.
	void run_frame();

private:
	vr::ETrackedControllerRole role_;
	int hand_; // UDCAP_HAND_LEFT / RIGHT
	std::shared_ptr<ShmReader> shm_;

	uint32_t object_id_ = vr::k_unTrackedDeviceIndexInvalid;
	vr::PropertyContainerHandle_t props_ = vr::k_ulInvalidPropertyContainer;
	std::string serial_;

	// Lighthouse tracker this glove rides on (resolved by serial from the shm).
	uint32_t tracker_id_ = vr::k_unTrackedDeviceIndexInvalid;
	uint32_t last_battery_ = 0;
	void find_tracker(const char *want_serial);

	// Input components (Index layout).
	vr::VRInputComponentHandle_t c_a_ = 0, c_b_ = 0, c_sys_ = 0, c_stick_click_ = 0, c_trig_click_ = 0;
	vr::VRInputComponentHandle_t c_trig_ = 0, c_grip_ = 0, c_grip_force_ = 0, c_stick_x_ = 0, c_stick_y_ = 0;
	vr::VRInputComponentHandle_t c_trackpad_touch_ = 0;

	// Skeletal (finger) input.
	vr::VRInputComponentHandle_t c_skeleton_ = 0;
	MyHandSimulation hand_sim_;
	void push_skeleton(const MyFingerCurls &curls, const MyFingerSplays &splays);
};
