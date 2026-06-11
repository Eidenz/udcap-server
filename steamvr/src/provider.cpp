#include "provider.h"

using namespace vr;

EVRInitError UdcapProvider::Init(IVRDriverContext *ctx)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(ctx);

	shm_ = std::make_shared<ShmReader>();
	if (!shm_->open()) {
		VRDriverLog()->Log("[udcap] shared memory not available yet (start udcap-server); will retry per frame.");
	}

	left_ = std::make_unique<GloveDevice>(TrackedControllerRole_LeftHand, shm_);
	right_ = std::make_unique<GloveDevice>(TrackedControllerRole_RightHand, shm_);

	VRServerDriverHost()->TrackedDeviceAdded(left_->serial(), TrackedDeviceClass_Controller, left_.get());
	VRServerDriverHost()->TrackedDeviceAdded(right_->serial(), TrackedDeviceClass_Controller, right_.get());

	VRDriverLog()->Log("[udcap] provider initialised (2 gloves)");
	return VRInitError_None;
}

void UdcapProvider::RunFrame()
{
	if (left_) {
		left_->run_frame();
	}
	if (right_) {
		right_->run_frame();
	}
}

void UdcapProvider::Cleanup()
{
	left_.reset();
	right_.reset();
	shm_.reset();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}
