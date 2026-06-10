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
				H.trigger = p->button.trigger;
				H.grip = p->button.grip;
				H.trackpad = p->button.trackpad;
				break;
			case CMD_INPUT_JOYSTICK:
				H.joy_x = p->joystickData.joyX;
				H.joy_y = p->joystickData.joyY;
				break;
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
			handle_command(g_shm->cmd_code, ctxs);
			__atomic_store_n(&g_shm->cmd_ack, cs, __ATOMIC_RELEASE);
		}

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
