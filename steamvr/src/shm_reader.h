// Read-only view of the udcap-server shared memory for the SteamVR driver.
// Reuses the exact same contract (udcap_shm.h) the Monado driver reads.
#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "udcap_shm.h" // on the include path via CMake (../shm)

class ShmReader
{
public:
	~ShmReader()
	{
		if (shm_ && shm_ != MAP_FAILED) {
			munmap((void *)shm_, sizeof(udcap_shm));
		}
	}

	// Open /dev/shm/udcap_hands (or reopen if the server (re)started). Cheap;
	// safe to call repeatedly until it succeeds.
	bool open()
	{
		if (shm_) {
			return true;
		}
		int fd = shm_open(UDCAP_SHM_NAME, O_RDONLY, 0);
		if (fd < 0) {
			return false;
		}
		void *m = mmap(nullptr, sizeof(udcap_shm), PROT_READ, MAP_SHARED, fd, 0);
		::close(fd);
		if (m == MAP_FAILED) {
			return false;
		}
		const udcap_shm *s = (const udcap_shm *)m;
		if (s->magic != UDCAP_SHM_MAGIC || s->version != UDCAP_SHM_VERSION) {
			munmap(m, sizeof(udcap_shm));
			return false;
		}
		shm_ = s;
		return true;
	}

	bool live() const { return shm_ && shm_->server_pid != 0; }

	// Seqlock read of one hand's published payload.
	bool read_hand(int i, udcap_hand *out) const
	{
		if (!shm_ || i < 0 || i >= UDCAP_HAND_COUNT) {
			return false;
		}
		const udcap_hand *h = &shm_->hands[i];
		uint32_t s;
		do {
			s = udcap_read_begin(h);
			*out = *h;
		} while (udcap_read_retry(h, s));
		return true;
	}

	// Config (offsets, maps, etc.) lives outside the seqlock payload.
	const udcap_shm *raw() const { return shm_; }

private:
	const udcap_shm *shm_ = nullptr;
};
