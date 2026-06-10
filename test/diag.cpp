// Standalone diagnostic + calibration harness for the UDCAP community driver
// core on Linux.
//
// Why this exists (vs. the upstream test tool):
//   * ALL logging goes to stderr, unbuffered, so nothing is hidden by pipe
//     buffering.
//   * Every phase is logged, so a hang is localized precisely.
//   * Ctrl-C works (atomic flag + poll loop).
//   * It does NOT use UsbEnumerate::refresh() (libusbp enumeration hangs on some
//     Linux systems). It opens the serial path directly -- exactly how the real
//     Monado driver will, since Monado does its own USB probing.
//
// Modes:
//   (default)      enumerate -> probe -> stream RAW sensor angles
//   --calibrate    after link, run the guided fist/spread/flat calibration on
//                  ALL hands together, save the result, then stream the COMPUTED
//                  per-bone skeleton quaternions (what the Monado driver needs)
//   --computed     skip calibration, load saved prefs, stream computed quats
//
// Targets: device paths on argv, else auto-scan /dev/ttyUSB0..31.
//   ./UdCapDiag --calibrate /dev/ttyUSB0 /dev/ttyUSB1

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <UsbEnumerate.h>
#include <PortAccessor.h>
#include <UdCapProbe.h>
#include <UdCapV1Core.h>
#include <CorePref.h>

// Streaming mode controls what the per-packet callback prints.
enum DiagMode
{
	MODE_RAW = 0,      // print [data] raw sensor angles
	MODE_CALIBRATING = 1, // suppress streaming spam, keep prompts readable
	MODE_COMPUTED = 2, // print [quat]/[angle] computed skeleton
};
static std::atomic_int g_mode{MODE_RAW};
static std::atomic_bool g_stop{false};

static void
on_signal(int)
{
	g_stop = true;
}

#define LOGP(...)                                                                                                      \
	do {                                                                                                           \
		fprintf(stderr, __VA_ARGS__);                                                                           \
		fflush(stderr);                                                                                        \
	} while (0)

static const char *
hand_str(UdTarget t)
{
	switch (t) {
	case UD_TARGET_LEFT_HAND: return "L";
	case UD_TARGET_RIGHT_HAND: return "R";
	default: return "?";
	}
}

struct CoreCtx
{
	std::shared_ptr<PortAccessor> pa;
	std::shared_ptr<UdCapV1Core> core;
	std::string recv_sn;
	std::shared_ptr<std::atomic_bool> linked = std::make_shared<std::atomic_bool>(false);
};

// Per-core print state: independent throttles per stream + button change detect,
// so quats aren't masked by angles and buttons/joystick don't spam.
struct Thr
{
	std::chrono::steady_clock::time_point t_data{};
	std::chrono::steady_clock::time_point t_quat{};
	std::chrono::steady_clock::time_point t_joy{};
	bool have_btn = false;
	int bA = 0, bB = 0, bMenu = 0, bJoy = 0, bPwr = 0;
};

static void
countdown(const char *msg, int secs)
{
	LOGP("\n>>> %s  (hold for %ds)\n", msg, secs);
	for (int s = secs; s > 0 && !g_stop; --s) {
		LOGP("    %d...\n", s);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}

int
main(int argc, char **argv)
{
	std::signal(SIGINT, on_signal);
	std::signal(SIGTERM, on_signal);

	bool do_calibrate = false;
	bool start_computed = false;
	std::vector<std::string> ports;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--calibrate") || !strcmp(argv[i], "-c"))
			do_calibrate = true;
		else if (!strcmp(argv[i], "--computed"))
			start_computed = true;
		else
			ports.push_back(argv[i]);
	}

	LOGP("[diag] start (build %s %s) mode=%s\n", __DATE__, __TIME__,
	     do_calibrate ? "calibrate" : (start_computed ? "computed" : "raw"));

	// Persist calibration in a deterministic, app-owned location so it can be
	// reused across runs (and later by the Monado driver).
	{
		const char *home = getenv("HOME");
		std::string pref_dir = std::string(home ? home : ".") + "/.config/udcap";
		CorePref::getInstance().setDefaultPrefPath(pref_dir);
		LOGP("[diag] prefs dir: %s\n", pref_dir.c_str());
	}

	if (ports.empty()) {
		for (int i = 0; i < 32; i++) {
			std::string p = "/dev/ttyUSB" + std::to_string(i);
			if (access(p.c_str(), F_OK) == 0)
				ports.push_back(p);
		}
	}
	LOGP("[diag] target serial port(s): %zu\n", ports.size());
	for (auto &p : ports)
		LOGP("    - %s\n", p.c_str());
	if (ports.empty()) {
		LOGP("[diag] No /dev/ttyUSB* found. Pass a path: ./UdCapDiag /dev/ttyUSB0\n");
		return 1;
	}

	if (start_computed)
		g_mode = MODE_COMPUTED;

	std::vector<std::shared_ptr<CoreCtx>> ctxs;
	std::vector<std::function<void()>> unlisten;

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

		LOGP("[diag] phase: opening + probing '%s' ... (1-4s timeout)\n", path.c_str());
		std::shared_ptr<PortAccessor> pa;
		try {
			pa = std::make_shared<PortAccessor>(d);
		} catch (const std::exception &e) {
			LOGP("[diag] '%s' PortAccessor ctor threw: %s\n", path.c_str(), e.what());
			continue;
		}

		UdCapProbe prober(pa);
		UdCapProbeType r = UDCAP_PROBE_FAILURE;
		try {
			r = prober.probe();
		} catch (const std::exception &e) {
			LOGP("[diag] '%s' probe threw: %s\n", path.c_str(), e.what());
			continue;
		}
		if (r != UDCAP_PROBE_HAND_V1) {
			LOGP("[diag] '%s' -> NOT a UDCAP receiver (no 'UD' reply). Skipping.\n", path.c_str());
			continue;
		}
		LOGP("[diag] '%s' -> UDCAP RECEIVER, SN='%s'\n", path.c_str(), prober.getUDCapSerial().c_str());

		auto ctx = std::make_shared<CoreCtx>();
		ctx->pa = pa;
		ctx->recv_sn = prober.getUDCapSerial();
		ctx->core = std::make_shared<UdCapV1Core>(pa);

		auto linked = ctx->linked;
		auto core = ctx->core;
		auto thr = std::make_shared<Thr>();

		auto u = core->listen([core, linked, thr](std::shared_ptr<UdCapV1MCUPacket> p) {
			const char *h = hand_str(core->getTarget());
			switch (p->commandType) {
			case CMD_LINK_STATE:
				LOGP("[%s][evt] link = %s\n", h,
				     UdCapV1Core::fromUdStateToString(p->udState).c_str());
				if (p->udState == UD_INIT_STATE_LINKED)
					linked->store(true);
				break;
			case CMD_SERIAL:
				LOGP("[%s][evt] glove SN='%s' type=%s\n", h, p->deviceSerialNum.c_str(),
				     p->isEnterprise ? "Enterprise" : "Client");
				break;
			case CMD_GET_CHANNEL: LOGP("[%s][evt] channel = %u\n", h, (unsigned)p->channel); break;
			case CMD_FW_VERSION: LOGP("[%s][evt] fw = %s\n", h, p->fwVersion.c_str()); break;
			case CMD_BATTERY: LOGP("[%s][evt] battery = %u\n", h, (unsigned)p->battery); break;
			case CMD_READY:
				if (p->isReady)
					LOGP("[%s][evt] READY\n", h);
				break;
			case CMD_INPUT_BUTTON: {
				// Print only when a boolean button changes (kills the spam);
				// include current analog trig/grip at that moment.
				int A = p->button.btnA, B = p->button.btnB, M = p->button.btnMenu,
				    J = p->button.btnJoyStick, P = p->button.btnPower;
				if (!thr->have_btn || A != thr->bA || B != thr->bB || M != thr->bMenu ||
				    J != thr->bJoy || P != thr->bPwr) {
					LOGP("[%s][btn] A=%d B=%d Menu=%d Joy=%d Pwr=%d  (trig=%.2f "
					     "grip=%.2f trackpad=%.2f)\n",
					     h, A, B, M, J, P, p->button.trigger, p->button.grip,
					     p->button.trackpad);
					thr->have_btn = true;
					thr->bA = A;
					thr->bB = B;
					thr->bMenu = M;
					thr->bJoy = J;
					thr->bPwr = P;
				}
				break;
			}
			case CMD_INPUT_JOYSTICK: {
				float x = p->joystickData.joyX, y = p->joystickData.joyY;
				if (std::fabs(x) < 0.05f && std::fabs(y) < 0.05f)
					break; // ignore centered stick
				auto now = std::chrono::steady_clock::now();
				if (now - thr->t_joy < std::chrono::milliseconds(300))
					break;
				thr->t_joy = now;
				LOGP("[%s][joy] x=%.3f y=%.3f\n", h, x, y);
				break;
			}
			case CMD_DATA: {
				if (g_mode.load() != MODE_RAW)
					break;
				auto now = std::chrono::steady_clock::now();
				if (now - thr->t_data < std::chrono::milliseconds(250))
					break;
				thr->t_data = now;
				LOGP("[%s][data] raw angle[19]:", h);
				for (auto a : p->angle)
					LOGP(" %d", (int)a);
				LOGP("\n");
				break;
			}
			case CMD_SKELETON_QUATERNION: {
				if (g_mode.load() != MODE_COMPUTED)
					break;
				auto now = std::chrono::steady_clock::now();
				if (now - thr->t_quat < std::chrono::milliseconds(200))
					break;
				thr->t_quat = now;
				auto &i = p->skeletonQuaternion.indexFinger;
				auto &m = p->skeletonQuaternion.middleFinger;
				LOGP("[%s][quat] idx.prox=(%.2f %.2f %.2f %.2f) "
				     "idx.dist=(%.2f %.2f %.2f %.2f) mid.prox=(%.2f %.2f %.2f %.2f)\n",
				     h, i.proximal.x, i.proximal.y, i.proximal.z, i.proximal.w,
				     i.distal.x, i.distal.y, i.distal.z, i.distal.w, m.proximal.x,
				     m.proximal.y, m.proximal.z, m.proximal.w);
				break;
			}
			case CMD_ANGLE: break; // redundant with quaternions; not printed
			default: break;
			}
		});
		unlisten.push_back(u);
		ctxs.push_back(ctx);

		core->mcuGetSerialNum();
		core->mcuGetFirmwareVersion();
		core->mcuGetChannel();
		core->mcuGetLinkState();
		core->mcuStartData();
		// NOTE: tryRestoreHandCalibration() requires UD_INIT_STATE_LINKED, so it
		// must be called AFTER the glove links (handled below), not here.
	}

	if (ctxs.empty()) {
		LOGP("[diag] No UDCAP receivers found/active. Exiting.\n");
		return 1;
	}
	LOGP("[diag] %zu receiver(s) active.\n", ctxs.size());

	// Both calibrate and computed-replay need the gloves linked first.
	if (do_calibrate || start_computed) {
		LOGP("[diag] waiting (up to 20s) for all gloves to link...\n");
		for (int i = 0; i < 200 && !g_stop; i++) {
			bool all = true;
			for (auto &c : ctxs)
				all = all && c->linked->load();
			if (all)
				break;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	if (start_computed) {
		// Load + apply previously-saved calibration so computed quats stream.
		for (auto &c : ctxs) {
			try {
				c->core->loadPref();
				c->core->tryRestoreHandCalibration();
				LOGP("[%s] restored saved calibration\n", hand_str(c->core->getTarget()));
			} catch (const std::exception &e) {
				LOGP("[%s] restore failed: %s (run --calibrate first)\n",
				     hand_str(c->core->getTarget()), e.what());
			}
		}
	}

	if (do_calibrate) {
		g_mode = MODE_CALIBRATING;
		LOGP("\n====================  CALIBRATION  ====================\n");
		LOGP("Do the SAME gesture with BOTH hands when prompted.\n");
		for (auto &c : ctxs) {
			try {
				c->core->runCalibration(UDCAP_V1_DEVICE_CALI_TYPE_HAND);
			} catch (const std::exception &e) {
				LOGP("[%s] runCalibration threw: %s\n", hand_str(c->core->getTarget()),
				     e.what());
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
					LOGP("[%s] capture threw: %s\n",
					     hand_str(c->core->getTarget()), e.what());
				}
			}
			LOGP("    captured.\n");
		}

		for (auto &c : ctxs) {
			try {
				c->core->completeCalibration(UDCAP_V1_DEVICE_CALI_TYPE_HAND);
				bool ok = c->core->savePref();
				LOGP("[%s] calibration complete, savePref=%s\n",
				     hand_str(c->core->getTarget()), ok ? "OK" : "FAILED");
			} catch (const std::exception &e) {
				LOGP("[%s] completeCalibration threw: %s\n",
				     hand_str(c->core->getTarget()), e.what());
			}
		}
		LOGP("======================================================\n");
		LOGP("Calibration done. Now streaming COMPUTED quaternions.\n");
		LOGP("Wiggle fingers, press A/B, move joystick. Ctrl-C to stop.\n\n");
		g_mode = MODE_COMPUTED;
	} else {
		LOGP("[diag] streaming %s. Wiggle fingers / press buttons. Ctrl-C to stop.\n",
		     g_mode.load() == MODE_COMPUTED ? "COMPUTED quaternions" : "RAW angles");
	}

	while (!g_stop)
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

	LOGP("[diag] stopping ...\n");
	for (auto &u : unlisten)
		u();
	ctxs.clear();
	LOGP("[diag] done.\n");
	return 0;
}
