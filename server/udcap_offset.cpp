// SPDX-License-Identifier: MIT
//
// udcap-offset: read/set the per-hand pose offset in the udcap-server shm. The
// Monado driver reads these every frame, so changes take effect live in VR.
//
//   udcap-offset show
//   udcap-offset <left|right> <px> <py> <pz> <degX> <degY> <degZ>
//
// Defaults the server writes (UDCAP "Space Orientation", Vive Tracker 3.0):
//   left  :  0.10  0.10 -0.05   45  85 0
//   right : -0.10  0.10 -0.05   45 -85 0

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "udcap_shm.h"

static void
print_hand(const char *name, const udcap_hand *h)
{
	printf("%-5s tracker='%s'  pos=(%.3f %.3f %.3f)  deg=(%.1f %.1f %.1f)\n", name, h->tracker_serial,
	       h->offset_pos[0], h->offset_pos[1], h->offset_pos[2], h->offset_rot_deg[0], h->offset_rot_deg[1],
	       h->offset_rot_deg[2]);
}

int
main(int argc, char **argv)
{
	int fd = shm_open(UDCAP_SHM_NAME, O_RDWR, 0);
	if (fd < 0) {
		fprintf(stderr, "shm_open(%s) failed: %s (is udcap-server running?)\n", UDCAP_SHM_NAME,
		        strerror(errno));
		return 1;
	}
	udcap_shm *shm = (udcap_shm *)mmap(nullptr, sizeof(udcap_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (shm == MAP_FAILED) {
		fprintf(stderr, "mmap failed: %s\n", strerror(errno));
		return 1;
	}
	if (shm->magic != UDCAP_SHM_MAGIC || shm->version != UDCAP_SHM_VERSION) {
		fprintf(stderr, "shm version mismatch (rebuild server+tools)\n");
		return 1;
	}

	if (argc < 2 || strcmp(argv[1], "show") == 0) {
		print_hand("left", &shm->hands[UDCAP_HAND_LEFT]);
		print_hand("right", &shm->hands[UDCAP_HAND_RIGHT]);
		return 0;
	}

	int idx = -1;
	if (!strcmp(argv[1], "left") || !strcmp(argv[1], "l"))
		idx = UDCAP_HAND_LEFT;
	else if (!strcmp(argv[1], "right") || !strcmp(argv[1], "r"))
		idx = UDCAP_HAND_RIGHT;

	if (idx < 0 || argc != 8) {
		fprintf(stderr,
		        "usage:\n  %s show\n  %s <left|right> <px> <py> <pz> <degX> <degY> <degZ>\n", argv[0],
		        argv[0]);
		return 1;
	}

	udcap_hand *h = &shm->hands[idx];
	h->offset_pos[0] = (float)atof(argv[2]);
	h->offset_pos[1] = (float)atof(argv[3]);
	h->offset_pos[2] = (float)atof(argv[4]);
	h->offset_rot_deg[0] = (float)atof(argv[5]);
	h->offset_rot_deg[1] = (float)atof(argv[6]);
	h->offset_rot_deg[2] = (float)atof(argv[7]);

	print_hand(idx == UDCAP_HAND_LEFT ? "left" : "right", h);
	return 0;
}
