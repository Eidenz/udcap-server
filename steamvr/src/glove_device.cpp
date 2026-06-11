#include "glove_device.h"

#include <cstring>

#include "vr_math.h"

using namespace vr;
using namespace vrm;

// Resolve a remapped button output to its live source value (mirrors the driver).
static bool
src_value(const udcap_hand &h, uint8_t src)
{
	switch (src) {
	case UDCAP_SRC_A: return h.btn_a != 0;
	case UDCAP_SRC_B: return h.btn_b != 0;
	case UDCAP_SRC_MENU: return h.btn_menu != 0;
	case UDCAP_SRC_STICK: return h.btn_joy != 0;
	default: return false;
	}
}

static float
clamp(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

// Bend magnitude of one bone quaternion, in radians.
static float
bone_curl(const udcap_quat &q)
{
	float w = std::fabs(q.w);
	if (w > 1.0f) {
		w = 1.0f;
	}
	return 2.0f * std::acos(w);
}

// Per-finger curl 0..1 (1 = fist): average of the three bending joints.
static float
finger_curl(const udcap_finger &f)
{
	float c = (bone_curl(f.proximal) + bone_curl(f.intermediate) + bone_curl(f.distal)) / 3.0f;
	return clamp(c / 1.4f, 0.0f, 1.0f);
}

// Per-finger splay -1..1 from the proximal sideways rotation, scaled by the shm gain.
static float
finger_splay(const udcap_finger &f, float gain)
{
	return clamp(gain * 2.0f * std::atan2(f.proximal.y, f.proximal.w) / 0.35f, -1.0f, 1.0f);
}

GloveDevice::GloveDevice(ETrackedControllerRole role, std::shared_ptr<ShmReader> shm)
    : role_(role), shm_(std::move(shm))
{
	hand_ = (role == TrackedControllerRole_LeftHand) ? UDCAP_HAND_LEFT : UDCAP_HAND_RIGHT;
	serial_ = std::string("UDCAP-") + (hand_ == UDCAP_HAND_LEFT ? "L" : "R");
}

EVRInitError GloveDevice::Activate(uint32_t object_id)
{
	object_id_ = object_id;
	props_ = VRProperties()->TrackedDeviceToPropertyContainer(object_id);
	auto p = VRProperties();

	// Present as a Valve Index Knuckles so games bind/skeleton the same way.
	p->SetStringProperty(props_, Prop_ModelNumber_String, "UDCAP Glove");
	p->SetStringProperty(props_, Prop_ManufacturerName_String, "Udexreal");
	p->SetStringProperty(props_, Prop_SerialNumber_String, serial_.c_str());
	p->SetStringProperty(props_, Prop_ControllerType_String, "knuckles");
	p->SetStringProperty(props_, Prop_RenderModelName_String,
	                     hand_ == UDCAP_HAND_LEFT ? "{indexcontroller}valve_controller_knu_1_0_left"
	                                              : "{indexcontroller}valve_controller_knu_1_0_right");
	p->SetInt32Property(props_, Prop_ControllerRoleHint_Int32, role_);
	p->SetInt32Property(props_, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);
	p->SetStringProperty(props_, Prop_InputProfilePath_String,
	                     "{indexcontroller}/input/index_controller_profile.json");
	p->SetBoolProperty(props_, Prop_DeviceProvidesBatteryStatus_Bool, true);

	auto in = VRDriverInput();
	in->CreateBooleanComponent(props_, "/input/a/click", &c_a_);
	in->CreateBooleanComponent(props_, "/input/b/click", &c_b_);
	in->CreateBooleanComponent(props_, "/input/system/click", &c_sys_);
	in->CreateBooleanComponent(props_, "/input/thumbstick/click", &c_stick_click_);
	in->CreateBooleanComponent(props_, "/input/trigger/click", &c_trig_click_);
	in->CreateScalarComponent(props_, "/input/trigger/value", &c_trig_, VRScalarType_Absolute,
	                          VRScalarUnits_NormalizedOneSided);
	in->CreateScalarComponent(props_, "/input/grip/value", &c_grip_, VRScalarType_Absolute,
	                          VRScalarUnits_NormalizedOneSided);
	in->CreateScalarComponent(props_, "/input/grip/force", &c_grip_force_, VRScalarType_Absolute,
	                          VRScalarUnits_NormalizedOneSided);
	in->CreateScalarComponent(props_, "/input/thumbstick/x", &c_stick_x_, VRScalarType_Absolute,
	                          VRScalarUnits_NormalizedTwoSided);
	in->CreateScalarComponent(props_, "/input/thumbstick/y", &c_stick_y_, VRScalarType_Absolute,
	                          VRScalarUnits_NormalizedTwoSided);

	in->CreateSkeletonComponent(
	    props_, hand_ == UDCAP_HAND_LEFT ? "/input/skeleton/left" : "/input/skeleton/right",
	    hand_ == UDCAP_HAND_LEFT ? "/skeleton/hand/left" : "/skeleton/hand/right", "/pose/raw",
	    VRSkeletalTracking_Full, nullptr, 0, &c_skeleton_);

	// SteamVR wants skeletal data right away; prime with the open hand.
	push_skeleton({}, {});
	push_skeleton({}, {});
	return VRInitError_None;
}

void GloveDevice::push_skeleton(const MyFingerCurls &curls, const MyFingerSplays &splays)
{
	if (!c_skeleton_) {
		return;
	}
	VRBoneTransform_t bones[eBone_Count];
	hand_sim_.ComputeSkeletonTransforms(role_, curls, splays, bones);
	VRDriverInput()->UpdateSkeletonComponent(c_skeleton_, VRSkeletalMotionRange_WithController, bones,
	                                         eBone_Count);
	VRDriverInput()->UpdateSkeletonComponent(c_skeleton_, VRSkeletalMotionRange_WithoutController, bones,
	                                         eBone_Count);
}

// Resolve which Lighthouse tracker this glove rides on. Matches the shm serial if
// set, otherwise picks the hand-th generic tracker.
void GloveDevice::find_tracker(const char *want)
{
	auto is_tracker = [](uint32_t i) -> bool {
		auto h = VRProperties()->TrackedDeviceToPropertyContainer(i);
		if (h == k_ulInvalidPropertyContainer) {
			return false;
		}
		ETrackedPropertyError e;
		int32_t cls = VRProperties()->GetInt32Property(h, Prop_DeviceClass_Int32, &e);
		return e == TrackedProp_Success && cls == TrackedDeviceClass_GenericTracker;
	};

	if (tracker_id_ != k_unTrackedDeviceIndexInvalid && is_tracker(tracker_id_)) {
		return; // keep the cached one
	}
	tracker_id_ = k_unTrackedDeviceIndexInvalid;

	int nth = 0;
	for (uint32_t i = 0; i < k_unMaxTrackedDeviceCount; i++) {
		if (i == object_id_ || !is_tracker(i)) {
			continue;
		}
		if (want && want[0]) {
			char serial[256] = {0};
			VRProperties()->GetStringProperty(VRProperties()->TrackedDeviceToPropertyContainer(i),
			                                  Prop_SerialNumber_String, serial, sizeof(serial));
			if (std::strstr(serial, want) != nullptr) {
				tracker_id_ = i;
				return;
			}
		} else if (nth++ == hand_) {
			tracker_id_ = i;
			return;
		}
	}
}

DriverPose_t GloveDevice::GetPose()
{
	DriverPose_t pose{};
	pose.qWorldFromDriverRotation.w = 1.0;
	pose.qDriverFromHeadRotation.w = 1.0;
	pose.qRotation.w = 1.0;

	udcap_hand h{};
	bool have = shm_ && shm_->open() && shm_->live() && shm_->read_hand(hand_, &h) && h.present;
	pose.deviceIsConnected = shm_ && shm_->live();
	if (!have) {
		pose.result = TrackingResult_Running_OutOfRange;
		return pose;
	}

	find_tracker(h.tracker_serial);
	if (tracker_id_ == k_unTrackedDeviceIndexInvalid) {
		pose.result = TrackingResult_Running_OutOfRange;
		return pose;
	}

	TrackedDevicePose_t poses[k_unMaxTrackedDeviceCount];
	VRServerDriverHost()->GetRawTrackedDevicePoses(0.f, poses, k_unMaxTrackedDeviceCount);
	const TrackedDevicePose_t &tp = poses[tracker_id_];
	if (!tp.bPoseIsValid) {
		pose.result = TrackingResult_Running_OutOfRange;
		return pose;
	}

	// controller = tracker pose, offset into the grip in the tracker's frame.
	const HmdVector3d_t tpos = mat_position(tp.mDeviceToAbsoluteTracking);
	const HmdQuaternion_t tori = mat_orientation(tp.mDeviceToAbsoluteTracking);

	pose.qRotation = tori * euler_deg(h.offset_rot_deg[0], h.offset_rot_deg[1], h.offset_rot_deg[2]);

	const HmdVector3d_t rp = rotate(tori, {h.offset_pos[0], h.offset_pos[1], h.offset_pos[2]});
	pose.vecPosition[0] = tpos.v[0] + rp.v[0];
	pose.vecPosition[1] = tpos.v[1] + rp.v[1];
	pose.vecPosition[2] = tpos.v[2] + rp.v[2];

	for (int i = 0; i < 3; i++) {
		pose.vecVelocity[i] = tp.vVelocity.v[i];
		pose.vecAngularVelocity[i] = tp.vAngularVelocity.v[i];
	}

	pose.poseIsValid = true;
	pose.result = TrackingResult_Running_OK;
	return pose;
}

void GloveDevice::run_frame()
{
	if (object_id_ == k_unTrackedDeviceIndexInvalid) {
		return;
	}
	VRServerDriverHost()->TrackedDevicePoseUpdated(object_id_, GetPose(), sizeof(DriverPose_t));

	udcap_hand h{};
	if (!shm_ || !shm_->read_hand(hand_, &h)) {
		return;
	}

	// Inputs, honoring the per-hand button remap (same logic as the Monado driver).
	const uint8_t *map = h.btn_src;
	auto in = VRDriverInput();
	in->UpdateBooleanComponent(c_a_, src_value(h, map[UDCAP_OUT_A]), 0);
	in->UpdateBooleanComponent(c_b_, src_value(h, map[UDCAP_OUT_B]), 0);
	in->UpdateBooleanComponent(c_sys_, src_value(h, map[UDCAP_OUT_SYSTEM]), 0);
	in->UpdateBooleanComponent(c_stick_click_, src_value(h, map[UDCAP_OUT_STICK]), 0);

	const float trig = src_value(h, map[UDCAP_OUT_TRIGGER]) ? 1.0f : h.trigger;
	const float grip = src_value(h, map[UDCAP_OUT_GRIP]) ? 1.0f : h.grip;
	in->UpdateScalarComponent(c_trig_, trig, 0);
	in->UpdateBooleanComponent(c_trig_click_, trig >= 0.7f, 0);
	in->UpdateScalarComponent(c_grip_, grip, 0);
	in->UpdateScalarComponent(c_grip_force_, grip, 0);
	in->UpdateScalarComponent(c_stick_x_, h.joy_x, 0);
	in->UpdateScalarComponent(c_stick_y_, h.joy_y, 0);

	// Skeletal finger tracking (curl per finger + splay scaled by the shm gain).
	const float splay_gain = shm_->raw() ? shm_->raw()->splay_gain : 1.0f;
	const MyFingerCurls curls{finger_curl(h.skel.thumb), finger_curl(h.skel.index),
	                          finger_curl(h.skel.middle), finger_curl(h.skel.ring),
	                          finger_curl(h.skel.little)};
	const MyFingerSplays splays{finger_splay(h.skel.thumb, splay_gain), finger_splay(h.skel.index, splay_gain),
	                            finger_splay(h.skel.middle, splay_gain), finger_splay(h.skel.ring, splay_gain),
	                            finger_splay(h.skel.little, splay_gain)};
	push_skeleton(curls, splays);

	// Battery (shm level 1..5 -> percent).
	if (h.battery != last_battery_) {
		last_battery_ = h.battery;
		VRProperties()->SetFloatProperty(props_, Prop_DeviceBatteryPercentage_Float,
		                                 h.battery ? h.battery * 0.2f : 1.0f);
	}
}
