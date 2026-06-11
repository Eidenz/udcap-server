# UDCAP SteamVR driver

A SteamVR (OpenVR) driver that presents the UDCAP gloves as **Valve Index
Knuckles**, reading the **same `/dev/shm/udcap_hands`** that `udcap-server`
publishes (so the same backend powers both Monado and SteamVR — only the
consumer differs).

## Build

```bash
cmake -S . -B build
cmake --build build -j
# -> udcap/bin/linux64/driver_udcap.so
```

## Register with SteamVR

Point SteamVR at this `udcap/` directory:

```bash
~/.local/share/Steam/steamapps/common/SteamVR/bin/linux64/vrpathreg \
    adddriver "$(pwd)/udcap"
```

Then start `udcap-server` (e.g. via the udcap-control app) and launch SteamVR;
two **UDCAP-L / UDCAP-R** controllers should appear.

## Status / milestones

- [x] **1. Scaffold** — loadable plugin, registers two Knuckles devices, reads the shm.
- [x] **2. Pose** — copies a configured Lighthouse tracker's pose + offset; battery from shm.
- [x] **3. Inputs** — A/B/system/stick/trigger/grip from the shm, honoring the remap config.
- [x] **4. Skeletal input** — 31-bone hand via the OpenVR hand-sim, curl+splay from our quats.
- [ ] **5. Packaging + app** — own input profile, install/enable from udcap-control, Monado⇄SteamVR
      toggle, and a **separate SteamVR pose offset** (Monado's is applied inverted).

## Tuning notes (first-pass constants, may need adjustment after testing)

- Pose **offset** reuses the Monado-tuned values applied the SteamVR way → orientation will likely
  need re-tuning (M5 gives SteamVR its own offset).
- Skeletal **curl** normalises the joint bend by ~1.4 rad; **splay** by ~0.35 rad × the shm splay gain;
  trigger-click fires at 0.7. Adjust in `glove_device.cpp` if fingers over/under-curl.
