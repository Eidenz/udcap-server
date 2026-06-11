// SPDX-License-Identifier: MIT
//
// udcap-server: headless bridge between the UDCAP gloves and the Monado
// drv_udcap driver (and any future GUI). It reuses the community core to read
// the gloves over serial, runs the per-session calibration, and publishes each
// hand's state into a shared-memory segment (see shm/udcap_shm.h). Haptic
// requests flow back from the driver via the same segment.
//
// Channels in shm:
//   * server -> reader : skeleton + buttons + joystick, guarded by a seqlock
//                        (the server is the only writer of that payload).
//   * reader -> server : haptic request (haptic_seq + params), which the server
//                        only ever READS, so there is no two-writer conflict.
//
// Build: target `udcap-server` (configured when -DBUILD_TEST_TOOLS=ON).

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <UsbEnumerate.h>
#include <PortAccessor.h>
#include <UdCapProbe.h>
#include <UdCapV1Core.h>
#include <CorePref.h>

#include "udcap_shm.h"

#define LOGP(...)                                                                                                      \
	do {                                                                                                           \
		fprintf(stderr, __VA_ARGS__);                                                                           \
		fflush(stderr);                                                                                        \
	} while (0)

static std::atomic_bool g_stop{false};
// Set in the packet callback on a power-button tap (a 1-frame pulse, easy for the
// main loop to miss), consumed by the main loop. The latch keeps btn_power lit
// briefly so the UI shows the press.
static std::atomic_bool g_power_tap{false};
static std::atomic<uint64_t> g_power_latch_ns{0};
static void
on_signal(int)
{
	g_stop = true;
}

static udcap_shm *g_shm = nullptr;
static int g_shm_fd = -1;

// Per-hand packet counters -> FPS (computed in the main loop, consumed by the
// publishing callback so the seqlock stays single-writer per hand).
static std::atomic<uint32_t> g_pkt_count[UDCAP_HAND_COUNT];
static std::atomic<float> g_fps[UDCAP_HAND_COUNT];

static uint64_t
now_ns()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static int
hand_index(UdTarget t)
{
	if (t == UD_TARGET_LEFT_HAND)
		return UDCAP_HAND_LEFT;
	if (t == UD_TARGET_RIGHT_HAND)
		return UDCAP_HAND_RIGHT;
	return -1;
}

static const char *
hand_str(UdTarget t)
{
	return t == UD_TARGET_LEFT_HAND ? "L" : t == UD_TARGET_RIGHT_HAND ? "R" : "?";
}

// Copy the server-written payload (present .. btn_power) into shm under the
// seqlock, leaving the haptic fields (driver-owned) untouched.
static void
publish(int idx, const udcap_hand &shadow)
{
	udcap_hand *H = &g_shm->hands[idx];
	const size_t off = offsetof(udcap_hand, present);
	const size_t end = offsetof(udcap_hand, haptic_seq);
	udcap_write_begin(H);
	memcpy((char *)H + off, (const char *)&shadow + off, end - off);
	udcap_write_end(H);
}

static bool
shm_setup()
{
	g_shm_fd = shm_open(UDCAP_SHM_NAME, O_CREAT | O_RDWR, 0666);
	if (g_shm_fd < 0) {
		LOGP("[server] shm_open(%s) failed: %s\n", UDCAP_SHM_NAME, strerror(errno));
		return false;
	}
	if (ftruncate(g_shm_fd, sizeof(udcap_shm)) != 0) {
		LOGP("[server] ftruncate failed: %s\n", strerror(errno));
		return false;
	}
	// Force world rw so a root-run monado-service can share our user-created shm.
	(void)fchmod(g_shm_fd, 0666);
	void *p = mmap(nullptr, sizeof(udcap_shm), PROT_READ | PROT_WRITE, MAP_SHARED, g_shm_fd, 0);
	if (p == MAP_FAILED) {
		LOGP("[server] mmap failed: %s\n", strerror(errno));
		return false;
	}
	g_shm = (udcap_shm *)p;
	memset(g_shm, 0, sizeof(*g_shm));
	g_shm->magic = UDCAP_SHM_MAGIC;
	g_shm->version = UDCAP_SHM_VERSION;
	g_shm->server_pid = (uint32_t)getpid();
	LOGP("[server] shm ready at /dev/shm%s (%zu bytes)\n", UDCAP_SHM_NAME, sizeof(udcap_shm));
	return true;
}

static void
countdown(const char *msg, int secs)
{
	LOGP("\n>>> %s  (hold for %ds)\n", msg, secs);
	for (int s = secs; s > 0 && !g_stop; --s) {
		LOGP("    %d...\n", s);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}

struct CoreCtx
{
	std::shared_ptr<PortAccessor> pa;
	std::shared_ptr<UdCapV1Core> core;
	std::string recv_sn;
	std::shared_ptr<std::atomic_bool> linked = std::make_shared<std::atomic_bool>(false);
	int idx = -1; // resolved hand slot once known
};

// Execute a control-app command (currently the guided calibration steps) on all
// linked hands and report progress via g_shm->calib_state.
// 0..1 curl from a bone quaternion (matches the driver's UDCAP_MAX_BEND_RAD).
static float
bone_curl01(const udcap_quat &qq)
{
	float w = std::fabs(qq.w);
	if (w > 1.0f) {
		w = 1.0f;
	}
	float c = (2.0f * std::acos(w)) / 1.35f;
	if (c < 0.0f) {
		c = 0.0f;
	}
	if (c > 1.0f) {
		c = 1.0f;
	}
	return c;
}
static float
remap01(float v, float lo, float hi)
{
	if (hi - lo < 0.01f) {
		return v;
	}
	float r = (v - lo) / (hi - lo);
	if (r < 0.0f) {
		r = 0.0f;
	}
	if (r > 1.0f) {
		r = 1.0f;
	}
	return r;
}

static void
handle_command(uint32_t code, std::vector<std::shared_ptr<CoreCtx>> &ctxs)
{
	for (auto &c : ctxs) {
		auto &core = c->core;
		try {
			switch (code) {
			case UDCAP_CMD_CALIB_START: core->runCalibration(UDCAP_V1_DEVICE_CALI_TYPE_HAND); break;
			case UDCAP_CMD_CALIB_FIST:
				core->captureCalibrationData(UDCAP_V1_HAND_CALI_TYPE_FIST);
				break;
			case UDCAP_CMD_CALIB_TOGETHER:
				core->captureCalibrationData(UDCAP_V1_HAND_CALI_TYPE_PROTRACT);
				break;
			case UDCAP_CMD_CALIB_SPREAD:
				core->captureCalibrationData(UDCAP_V1_HAND_CALI_TYPE_ADDUCTION);
				break;
			case UDCAP_CMD_CALIB_COMPLETE:
				core->completeCalibration(UDCAP_V1_DEVICE_CALI_TYPE_HAND);
				break;
			default: break;
			}
		} catch (const std::exception &e) {
			LOGP("[server] command %u on %s threw: %s\n", code, hand_str(core->getTarget()), e.what());
		}
	}

	switch (code) {
	case UDCAP_CMD_CALIB_START: g_shm->calib_state = UDCAP_CALIB_STARTED; break;
	case UDCAP_CMD_CALIB_FIST: g_shm->calib_state = UDCAP_CALIB_GOT_FIST; break;
	case UDCAP_CMD_CALIB_TOGETHER: g_shm->calib_state = UDCAP_CALIB_GOT_TOGETHER; break;
	case UDCAP_CMD_CALIB_SPREAD: g_shm->calib_state = UDCAP_CALIB_GOT_SPREAD; break;
	case UDCAP_CMD_CALIB_COMPLETE: g_shm->calib_state = UDCAP_CALIB_DONE; break;
	case UDCAP_CMD_CALIB_CANCEL: g_shm->calib_state = UDCAP_CALIB_IDLE; break;
	default: break;
	}
	LOGP("[server] command %u -> calib_state %u\n", code, g_shm->calib_state);
}

// ---- Autonomous timed calibration (glove menu button or CMD_CALIB_AUTO) ----
// Runs the whole fist -> together -> spread -> complete sequence on a timer, so
// it can be done in VR without the GUI. The GUI just watches calib_state for the
// visuals and audio cues.
static bool g_autocal_active = false;
static int g_autocal_step = 0; // 0=get ready, 1=fist, 2=together, 3=spread, 4=finalize
static uint64_t g_autocal_next_ns = 0;
static const uint64_t AUTOCAL_READY_NS = 3000000000ull; // "get ready" before the fist
static const uint64_t AUTOCAL_HOLD_NS = 4000000000ull;  // hold each pose ~4s

static void
start_autocal(std::vector<std::shared_ptr<CoreCtx>> &ctxs)
{
	if (g_autocal_active) {
		return;
	}
	handle_command(UDCAP_CMD_CALIB_START, ctxs); // runCalibration (sets STARTED)
	g_shm->calib_state = UDCAP_CALIB_READY;       // ...but show a "get ready" first
	g_autocal_active = true;
	g_autocal_step = 0;
	g_autocal_next_ns = now_ns() + AUTOCAL_READY_NS;
	LOGP("[server] auto-calibration started (get ready)\n");
}

static void
tick_autocal(std::vector<std::shared_ptr<CoreCtx>> &ctxs)
{
	if (!g_autocal_active || now_ns() < g_autocal_next_ns) {
		return;
	}
	switch (g_autocal_step) {
	case 0: // get ready -> first pose
		g_shm->calib_state = UDCAP_CALIB_STARTED;
		g_autocal_step = 1;
		g_autocal_next_ns = now_ns() + AUTOCAL_HOLD_NS;
		break;
	case 1:
		handle_command(UDCAP_CMD_CALIB_FIST, ctxs);
		g_autocal_step = 2;
		g_autocal_next_ns = now_ns() + AUTOCAL_HOLD_NS;
		break;
	case 2:
		handle_command(UDCAP_CMD_CALIB_TOGETHER, ctxs);
		g_autocal_step = 3;
		g_autocal_next_ns = now_ns() + AUTOCAL_HOLD_NS;
		break;
	case 3:
		handle_command(UDCAP_CMD_CALIB_SPREAD, ctxs);
		g_autocal_step = 4;
		g_autocal_next_ns = now_ns() + 700000000ull; // brief pause before finalizing
		break;
	default:
		handle_command(UDCAP_CMD_CALIB_COMPLETE, ctxs);
		g_autocal_active = false;
		LOGP("[server] auto-calibration complete\n");
		break;
	}
}

int
main(int argc, char **argv)
{
	std::signal(SIGINT, on_signal);
	std::signal(SIGTERM, on_signal);

	bool do_calibrate = true;
	std::string tracker_left, tracker_right;
	std::vector<std::string> ports;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--no-cal")) {
			do_calibrate = false;
		} else if ((!strcmp(argv[i], "--tracker-left") || !strcmp(argv[i], "--tl")) && i + 1 < argc) {
			tracker_left = argv[++i];
		} else if ((!strcmp(argv[i], "--tracker-right") || !strcmp(argv[i], "--tr")) && i + 1 < argc) {
			tracker_right = argv[++i];
		} else {
			ports.push_back(argv[i]);
		}
	}

	LOGP("[server] starting (build %s %s)\n", __DATE__, __TIME__);

	{
		const char *home = getenv("HOME");
		std::string pref_dir = std::string(home ? home : ".") + "/.config/udcap";
		CorePref::getInstance().setDefaultPrefPath(pref_dir);
	}

	if (!shm_setup())
		return 1;

	// Publish tracker<->hand mapping for the Monado driver (static config).
	if (!tracker_left.empty()) {
		strncpy(g_shm->hands[UDCAP_HAND_LEFT].tracker_serial, tracker_left.c_str(),
		        sizeof(g_shm->hands[UDCAP_HAND_LEFT].tracker_serial) - 1);
		LOGP("[server] left  hand -> tracker '%s'\n", tracker_left.c_str());
	}
	if (!tracker_right.empty()) {
		strncpy(g_shm->hands[UDCAP_HAND_RIGHT].tracker_serial, tracker_right.c_str(),
		        sizeof(g_shm->hands[UDCAP_HAND_RIGHT].tracker_serial) - 1);
		LOGP("[server] right hand -> tracker '%s'\n", tracker_right.c_str());
	}

	// Default pose offsets. The tracker sits at the controller grip, so no
	// position offset is needed in Monado's frame (verified on hardware); only
	// the rotation aligns the grip axis. Tunable live via udcap-offset / the GUI.
	{
		udcap_hand &L = g_shm->hands[UDCAP_HAND_LEFT];
		L.offset_pos[0] = 0.0f;
		L.offset_pos[1] = 0.0f;
		L.offset_pos[2] = 0.0f;
		L.offset_rot_deg[0] = 45.0f;
		L.offset_rot_deg[1] = 85.0f;
		L.offset_rot_deg[2] = 0.0f;

		udcap_hand &R = g_shm->hands[UDCAP_HAND_RIGHT];
		R.offset_pos[0] = 0.0f;
		R.offset_pos[1] = 0.0f;
		R.offset_pos[2] = 0.0f;
		R.offset_rot_deg[0] = 45.0f;
		R.offset_rot_deg[1] = -85.0f;
		R.offset_rot_deg[2] = 0.0f;

		// Identity curl remap by default (no scaling).
		for (int h = 0; h < UDCAP_HAND_COUNT; h++) {
			for (int f = 0; f < 5; f++) {
				g_shm->hands[h].curl_min[f] = 0.0f;
				g_shm->hands[h].curl_max[f] = 1.0f;
			}
		}

		// Grip/menu defaults: place VRChat's hand menu on the palm side (tuned on
		// hardware). Mirrored between hands.
		L.grip_pos[0] = 0.06f;
		L.grip_pos[1] = -0.06f;
		L.grip_pos[2] = 0.01f;
		L.grip_rot_deg[0] = 70.0f;
		L.grip_rot_deg[1] = -5.0f;
		L.grip_rot_deg[2] = -55.0f;

		R.grip_pos[0] = -0.06f;
		R.grip_pos[1] = -0.06f;
		R.grip_pos[2] = 0.01f;
		R.grip_rot_deg[0] = 70.0f;
		R.grip_rot_deg[1] = -5.0f;
		R.grip_rot_deg[2] = 75.0f;

		g_shm->curl_gain = 1.5f; // full fist by default (also the maximum)
		// Splay is correct (same path as opengloves); some OpenXR apps use it even
		// though VRChat's hand-tracking ignores finger abduction.
		g_shm->splay_gain = 1.0f;

		// Per-hand input mapping defaults.
		for (int h = 0; h < UDCAP_HAND_COUNT; h++) {
			udcap_hand &H = g_shm->hands[h];
			H.btn_src[UDCAP_OUT_A] = UDCAP_SRC_A;
			H.btn_src[UDCAP_OUT_B] = UDCAP_SRC_B;
			H.btn_src[UDCAP_OUT_SYSTEM] = UDCAP_SRC_MENU;
			H.btn_src[UDCAP_OUT_STICK] = UDCAP_SRC_STICK;
			H.btn_src[UDCAP_OUT_TRIGGER] = UDCAP_SRC_NONE;
			H.btn_src[UDCAP_OUT_GRIP] = UDCAP_SRC_NONE;
			H.trigger_finger = 1;             // index
			H.grip_finger = UDCAP_FINGER_GRIP; // average of middle+ring+little
			H.trigger_min = 0.15f;
			H.trigger_max = 0.85f;
			H.grip_min = 0.6f;
			H.grip_max = 0.85f;
			H.stick_deadzone = 0.0f;
			H.trackpad_threshold = 0.1f;
		}
	}

	if (ports.empty()) {
		for (int i = 0; i < 32; i++) {
			std::string p = "/dev/ttyUSB" + std::to_string(i);
			if (access(p.c_str(), F_OK) == 0)
				ports.push_back(p);
		}
	}
	LOGP("[server] probing %zu serial port(s)\n", ports.size());

	// Per-hand shadow copies the callbacks mutate, then publish().
	std::vector<udcap_hand> shadow(UDCAP_HAND_COUNT);
	memset(shadow.data(), 0, shadow.size() * sizeof(udcap_hand));

	std::vector<std::shared_ptr<CoreCtx>> ctxs;
	std::vector<std::function<void()>> unlisten;
	// core per hand slot, for haptic dispatch
	std::shared_ptr<UdCapV1Core> core_by_hand[UDCAP_HAND_COUNT] = {nullptr, nullptr};

	for (auto &path : ports) {
		if (g_stop)
			break;
		SerialDevice d{};
		d.isHid = false;
		d.vid = 0x1A86;
		d.pid = 0x7523;
		d.composite = false;
		d.interfaceNumber = 0;
		d.portName = path;

		LOGP("[server] probing '%s' ...\n", path.c_str());
		std::shared_ptr<PortAccessor> pa;
		try {
			pa = std::make_shared<PortAccessor>(d);
		} catch (const std::exception &e) {
			LOGP("[server] '%s' open threw: %s\n", path.c_str(), e.what());
			continue;
		}
		UdCapProbe prober(pa);
		if (prober.probe() != UDCAP_PROBE_HAND_V1) {
			LOGP("[server] '%s' not a UDCAP receiver, skipping\n", path.c_str());
			continue;
		}
		LOGP("[server] '%s' -> receiver SN=%s\n", path.c_str(), prober.getUDCapSerial().c_str());

		auto ctx = std::make_shared<CoreCtx>();
		ctx->pa = pa;
		ctx->recv_sn = prober.getUDCapSerial();
		ctx->core = std::make_shared<UdCapV1Core>(pa);
		auto core = ctx->core;
		auto linked = ctx->linked;
		auto *shadow_data = shadow.data();

		auto u = core->listen([core, linked, shadow_data](std::shared_ptr<UdCapV1MCUPacket> p) {
			int idx = hand_index(core->getTarget());
			if (idx < 0)
				return; // hand not yet known
			udcap_hand &H = shadow_data[idx];

			switch (p->commandType) {
			case CMD_LINK_STATE:
				H.present = 1;
				H.link = (uint32_t)p->udState;
				if (p->udState == UD_INIT_STATE_LINKED)
					linked->store(true);
				break;
			case CMD_DATA:
				// DIAGNOSTIC: the 12 raw per-finger sensor channels (pre-sensor2Angle).
				for (int k = 0; k < 12; k++) {
					g_shm->raw_sensors[idx][k] = (float)p->angle[k];
				}
				break;
			case CMD_SERIAL:
				H.present = 1;
				strncpy(H.glove_serial, p->deviceSerialNum.c_str(), sizeof(H.glove_serial) - 1);
				break;
			case CMD_FW_VERSION:
				strncpy(H.fw, p->fwVersion.c_str(), sizeof(H.fw) - 1);
				break;
			case CMD_BATTERY: H.battery = p->battery; break;
			case CMD_INPUT_BUTTON:
				H.btn_a = p->button.btnA;
				H.btn_b = p->button.btnB;
				H.btn_menu = p->button.btnMenu;
				H.btn_joy = p->button.btnJoyStick;
				H.btn_power = p->button.btnPower;
				if (p->button.btnPower) {
					g_power_tap.store(true, std::memory_order_relaxed);
					g_power_latch_ns.store(now_ns() + 350000000ull, std::memory_order_relaxed);
				}
				if (now_ns() < g_power_latch_ns.load(std::memory_order_relaxed)) {
					H.btn_power = 1; // brief UI flash (the raw pulse is 1 frame)
				}
				// trigger/grip are computed from the skeleton (see above).
				H.trackpad = p->button.trackpad;
				break;
			case CMD_INPUT_JOYSTICK: {
				// Radial deadzone (rescaled so motion past the edge starts at 0).
				float jx = p->joystickData.joyX, jy = p->joystickData.joyY;
				float dz = g_shm->hands[idx].stick_deadzone;
				float mag = sqrtf(jx * jx + jy * jy);
				if (mag < 1e-5f || mag <= dz) {
					jx = 0.0f;
					jy = 0.0f;
				} else {
					float s = (mag - dz) / (1.0f - dz) / mag;
					jx = std::fmax(-1.0f, std::fmin(1.0f, jx * s));
					jy = std::fmax(-1.0f, std::fmin(1.0f, jy * s));
				}
				H.joy_x = jx;
				H.joy_y = jy;
				break;
			}
			case CMD_SKELETON_QUATERNION: {
				const HandQuaternion &q = p->skeletonQuaternion;
				auto cp = [](udcap_quat &d, const BoneQuaternion &s) {
					d.x = s.x;
					d.y = s.y;
					d.z = s.z;
					d.w = s.w;
				};
				auto cf = [&](udcap_finger &df, const FingerQuaternion &sf) {
					cp(df.proximal, sf.proximal);
					cp(df.intermediate, sf.intermediate);
					cp(df.distal, sf.distal);
				};
				cf(H.skel.thumb, q.thumbFinger);
				cf(H.skel.index, q.indexFinger);
				cf(H.skel.middle, q.middleFinger);
				cf(H.skel.ring, q.ringFinger);
				cf(H.skel.little, q.littleFinger);
				H.calibrated = 1;

				// Analog trigger/grip from the configured finger(s) + thresholds.
				// Config lives in the live shm hand (written by the server/GUI),
				// not the shadow H we publish from.
				const udcap_hand &cfg = g_shm->hands[idx];
				float cu[5] = {
				    bone_curl01(H.skel.thumb.proximal), bone_curl01(H.skel.index.proximal),
				    bone_curl01(H.skel.middle.proximal), bone_curl01(H.skel.ring.proximal),
				    bone_curl01(H.skel.little.proximal),
				};
				uint8_t tf = cfg.trigger_finger < 5 ? cfg.trigger_finger : 1;
				H.trigger = remap01(cu[tf], cfg.trigger_min, cfg.trigger_max);
				float gsrc = (cfg.grip_finger == UDCAP_FINGER_GRIP)
				                 ? (cu[2] + cu[3] + cu[4]) / 3.0f
				                 : cu[cfg.grip_finger < 5 ? cfg.grip_finger : 2];
				H.grip = remap01(gsrc, cfg.grip_min, cfg.grip_max);
				break;
			}
			default: break;
			}
			g_pkt_count[idx].fetch_add(1, std::memory_order_relaxed);
			H.fps = g_fps[idx].load(std::memory_order_relaxed);
			H.timestamp_ns = now_ns();
			publish(idx, H);
		});
		unlisten.push_back(u);
		ctxs.push_back(ctx);

		core->mcuGetSerialNum();
		core->mcuGetFirmwareVersion();
		core->mcuGetLinkState();
		core->mcuStartData();
	}

	if (ctxs.empty()) {
		LOGP("[server] no UDCAP receivers found. Exiting.\n");
		g_shm->server_pid = 0; // don't leave a stale "live" segment behind
		return 1;
	}

	// Resolve which core serves which hand (for haptic dispatch).
	LOGP("[server] waiting (up to 20s) for gloves to link...\n");
	for (int i = 0; i < 200 && !g_stop; i++) {
		bool all = true;
		for (auto &c : ctxs)
			all = all && c->linked->load();
		if (all)
			break;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	for (auto &c : ctxs) {
		int idx = hand_index(c->core->getTarget());
		if (idx >= 0) {
			c->idx = idx;
			core_by_hand[idx] = c->core;
		}
	}

	if (do_calibrate) {
		LOGP("\n==================== CALIBRATION ====================\n");
		LOGP("Make the SAME gesture with BOTH hands when prompted.\n");
		for (auto &c : ctxs) {
			try {
				c->core->runCalibration(UDCAP_V1_DEVICE_CALI_TYPE_HAND);
			} catch (const std::exception &e) {
				LOGP("[%s] runCalibration threw: %s\n", hand_str(c->core->getTarget()), e.what());
			}
		}
		std::this_thread::sleep_for(std::chrono::seconds(1));
		struct Step
		{
			const char *msg;
			UdCapV1HandCaliType type;
		};
		Step steps[] = {
		    {"Make a FIST", UDCAP_V1_HAND_CALI_TYPE_FIST},
		    {"Flat hand, fingers SPREAD apart", UDCAP_V1_HAND_CALI_TYPE_ADDUCTION},
		    {"Flat hand, fingers TOGETHER / straight", UDCAP_V1_HAND_CALI_TYPE_PROTRACT},
		};
		for (auto &st : steps) {
			if (g_stop)
				break;
			countdown(st.msg, 6);
			for (auto &c : ctxs) {
				try {
					c->core->captureCalibrationData(st.type);
				} catch (const std::exception &e) {
					LOGP("[%s] capture threw: %s\n", hand_str(c->core->getTarget()), e.what());
				}
			}
		}
		for (auto &c : ctxs) {
			try {
				c->core->completeCalibration(UDCAP_V1_DEVICE_CALI_TYPE_HAND);
			} catch (const std::exception &e) {
				LOGP("[%s] complete threw: %s\n", hand_str(c->core->getTarget()), e.what());
			}
		}
		LOGP("====================================================\n");
		LOGP("[server] calibration done. Publishing to shm. Ctrl-C to stop.\n");
	} else {
		LOGP("[server] skipping calibration (--no-cal). Publishing RAW (skeleton may be invalid).\n");
	}

	// Main loop: haptics (driver -> server), commands (app -> server), FPS.
	uint32_t last_haptic[UDCAP_HAND_COUNT] = {0, 0};
	for (int i = 0; i < UDCAP_HAND_COUNT; i++)
		last_haptic[i] = __atomic_load_n(&g_shm->hands[i].haptic_seq, __ATOMIC_ACQUIRE);

	uint32_t last_cmd = __atomic_load_n(&g_shm->cmd_seq, __ATOMIC_ACQUIRE);
	__atomic_store_n(&g_shm->cmd_ack, last_cmd, __ATOMIC_RELEASE);

	uint64_t last_fps_ns = now_ns();
	uint32_t last_count[UDCAP_HAND_COUNT] = {0, 0};

	// Reconnection watchdog: nudge the stream back if a hand goes quiet.
	uint64_t wd_pkt_ns[UDCAP_HAND_COUNT] = {now_ns(), now_ns()};
	uint32_t wd_count[UDCAP_HAND_COUNT] = {0, 0};
	uint64_t wd_nudge_ns[UDCAP_HAND_COUNT] = {0, 0};
	bool wd_active[UDCAP_HAND_COUNT] = {false, false};

	while (!g_stop) {
		// Haptics.
		for (int i = 0; i < UDCAP_HAND_COUNT; i++) {
			uint32_t s = __atomic_load_n(&g_shm->hands[i].haptic_seq, __ATOMIC_ACQUIRE);
			if (s != last_haptic[i]) {
				last_haptic[i] = s;
				int index = g_shm->hands[i].haptic_index;
				float dur = g_shm->hands[i].haptic_duration_s;
				int strength = g_shm->hands[i].haptic_strength;
				if (core_by_hand[i]) {
					try {
						core_by_hand[i]->mcuSendVibration(index < 0 ? 1 : index, dur,
						                                  strength);
					} catch (const std::exception &e) {
						LOGP("[server] haptic send failed: %s\n", e.what());
					}
				}
			}
		}

		// Control-app commands (calibration).
		uint32_t cs = __atomic_load_n(&g_shm->cmd_seq, __ATOMIC_ACQUIRE);
		if (cs != last_cmd) {
			last_cmd = cs;
			uint32_t code = g_shm->cmd_code;
			if (code == UDCAP_CMD_CALIB_AUTO) {
				start_autocal(ctxs);
			} else {
				if (code == UDCAP_CMD_CALIB_CANCEL) {
					g_autocal_active = false;
				}
				handle_command(code, ctxs);
			}
			__atomic_store_n(&g_shm->cmd_ack, cs, __ATOMIC_RELEASE);
		}

		// Glove power (side) button tap -> start / cancel auto-calibration. The tap
		// is a 1-frame pulse caught in the packet callback; we just consume it.
		if (g_power_tap.exchange(false, std::memory_order_relaxed)) {
			if (g_autocal_active) {
				g_autocal_active = false;
				g_shm->calib_state = UDCAP_CALIB_IDLE;
				LOGP("[server] auto-calibration cancelled (power tap)\n");
			} else {
				start_autocal(ctxs);
			}
		}
		tick_autocal(ctxs);

		// FPS, every ~500ms.
		uint64_t now = now_ns();
		if (now - last_fps_ns >= 500000000ull) {
			double secs = (double)(now - last_fps_ns) / 1.0e9;
			for (int i = 0; i < UDCAP_HAND_COUNT; i++) {
				uint32_t c = g_pkt_count[i].load(std::memory_order_relaxed);
				g_fps[i].store((float)((double)(c - last_count[i]) / secs),
				               std::memory_order_relaxed);
				last_count[i] = c;
			}
			last_fps_ns = now;

			// Reconnection: a hand that was streaming and went quiet for >2.5s gets
			// its data stream re-triggered (handles a dropped 2.4GHz link where the
			// dongle stops sending). The server keeps running, so the shm stays
			// valid and Monado does not need restarting.
			for (int i = 0; i < UDCAP_HAND_COUNT; i++) {
				uint32_t c = g_pkt_count[i].load(std::memory_order_relaxed);
				if (c != wd_count[i]) {
					wd_count[i] = c;
					wd_pkt_ns[i] = now;
					wd_active[i] = true;
				} else if (wd_active[i] && core_by_hand[i] && now - wd_pkt_ns[i] > 2500000000ull &&
				           now - wd_nudge_ns[i] > 1500000000ull) {
					wd_nudge_ns[i] = now;
					LOGP("[server] hand %d quiet, re-triggering stream...\n", i);
					try {
						core_by_hand[i]->mcuGetLinkState();
						core_by_hand[i]->mcuStartData();
					} catch (const std::exception &e) {
						LOGP("[server] reconnect nudge failed: %s\n", e.what());
					}
				}
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	LOGP("[server] stopping ...\n");
	for (int i = 0; i < UDCAP_HAND_COUNT; i++) {
		udcap_hand *H = &g_shm->hands[i];
		udcap_write_begin(H);
		H->present = 0;
		H->link = UDCAP_LINK_NOT_CONNECTED;
		udcap_write_end(H);
	}
	g_shm->server_pid = 0;
	for (auto &u : unlisten)
		u();
	ctxs.clear();
	LOGP("[server] done.\n");
	return 0;
}
