// SPDX-License-Identifier: MIT
//
// udcap-shmdump: read-only viewer for the udcap-server shared-memory segment.
// Shows, per hand, the update AGE (so you can tell if a slot is being written
// live vs frozen) and a per-finger curl readout, plus controller inputs.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <chrono>
#include <ctime>
#include <thread>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "udcap_shm.h"

static volatile sig_atomic_t g_stop = 0;
static void
on_sig(int)
{
	g_stop = 1;
}

static uint64_t
now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

// 0..1 curl estimate from a bone quaternion (matches the driver's mapping).
static float
curl(udcap_quat q)
{
	float w = fabsf(q.w);
	if (w > 1.0f)
		w = 1.0f;
	float c = (2.0f * acosf(w)) / 1.5f;
	if (c < 0.0f)
		c = 0.0f;
	if (c > 1.0f)
		c = 1.0f;
	return c;
}

int
main(void)
{
	std::signal(SIGINT, on_sig);
	std::signal(SIGTERM, on_sig);

	int fd = shm_open(UDCAP_SHM_NAME, O_RDWR, 0);
	if (fd < 0) {
		fprintf(stderr, "shm_open(%s) failed: %s  (is udcap-server running?)\n", UDCAP_SHM_NAME,
		        strerror(errno));
		return 1;
	}
	void *p = mmap(nullptr, sizeof(udcap_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED) {
		fprintf(stderr, "mmap failed: %s\n", strerror(errno));
		return 1;
	}
	udcap_shm *shm = (udcap_shm *)p;
	if (shm->magic != UDCAP_SHM_MAGIC || shm->version != UDCAP_SHM_VERSION) {
		fprintf(stderr, "bad magic/version (got %08x/%u, want %08x/%u). Rebuild server+shmdump.\n",
		        shm->magic, shm->version, UDCAP_SHM_MAGIC, UDCAP_SHM_VERSION);
		return 1;
	}
	fprintf(stderr, "shm ok. server_pid=%u. age=ms since this slot was last written (small=live, growing=frozen).\n",
	        shm->server_pid);

	const char *names[2] = {"L", "R"};
	while (!g_stop) {
		uint64_t t = now_ns();
		for (int i = 0; i < UDCAP_HAND_COUNT; i++) {
			udcap_hand snap;
			uint32_t s;
			do {
				s = udcap_read_begin(&shm->hands[i]);
				snap = shm->hands[i];
			} while (udcap_read_retry(&shm->hands[i], s));

			if (!snap.present) {
				fprintf(stderr, "[%s] absent\n", names[i]);
				continue;
			}
			double age_ms = ((double)t - (double)snap.timestamp_ns) / 1.0e6;
			fprintf(stderr,
			        "[%s] age=%6.0fms link=%u cal=%u | curl T=%.2f I=%.2f M=%.2f R=%.2f P=%.2f | "
			        "A=%u B=%u menu=%u joyBtn=%u joy=(%+.2f %+.2f) trig=%.2f grip=%.2f\n",
			        names[i], age_ms, snap.link, snap.calibrated, curl(snap.skel.thumb.proximal),
			        curl(snap.skel.index.proximal), curl(snap.skel.middle.proximal),
			        curl(snap.skel.ring.proximal), curl(snap.skel.little.proximal), snap.btn_a, snap.btn_b,
			        snap.btn_menu, snap.btn_joy, snap.joy_x, snap.joy_y, snap.trigger, snap.grip);
			// Per-joint breakdown (index & middle) to compare hands: prox/inter/distal.
			fprintf(stderr,
			        "      index[p=%.2f i=%.2f d=%.2f] middle[p=%.2f i=%.2f d=%.2f]\n",
			        curl(snap.skel.index.proximal), curl(snap.skel.index.intermediate),
			        curl(snap.skel.index.distal), curl(snap.skel.middle.proximal),
			        curl(snap.skel.middle.intermediate), curl(snap.skel.middle.distal));
		}
		fprintf(stderr, "----\n");
		fflush(stderr);
		std::this_thread::sleep_for(std::chrono::milliseconds(250));
	}
	return 0;
}
