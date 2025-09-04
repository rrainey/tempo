# Tempo-BT (V1) — MVP Build Plan (tiny, testable tasks)

> Each task is a single concern with a crisp end state (“Done when…”).
> Order is linear; you can split across engineers by phase. Commands shown assume `BOARD=nrf5340dk_nrf5340_cpuapp`.

---

## Phase 0 — Bootstrap & Hello World

1. **Create Zephyr app skeleton**

* Do: `CMakeLists.txt`, `prj.conf` (empty), `src/main.c` with `printk("boot\n")`.
* Done when: `west build -t flash` boots and you see `boot` over RTT.

2. **Add board overlay scaffold**

* Do: `boards/.../tempo_v1.overlay` with only CPU & aliases (no peripherals yet).
* Done when: Build still succeeds (no DTS errors).

3. **Turn on RTT logging**

* Do: `CONFIG_LOG=y`, `CONFIG_LOG_BACKEND_RTT=y`.
* Done when: `LOG_INF("hello")` appears in RTT viewer.

---

## Phase 1 — Storage First (QSPI + littlefs)

4. **Enable QSPI NOR in DTS**

* Do: Populate `&qspi { ... jedec,spi-nor ... };`.
* Done when: Boot log shows flash JEDEC detected (INFO from driver).

5. **Define littlefs mount**

* Do: `fs_mount_t` for `/lfs`; mount in `app_init.c`.
* Done when: Mount succeeds and `fs_mkdir("/lfs/logs")` returns 0.

6. **Smoke test file write**

* Do: Append “test\n” to `/lfs/logs/smoke.txt`.
* Done when: Reboot and re-open file, content persists.

7. **Partition file (optional)**

* Do: `partitions.yml` (mcuboot, littlefs). Hook into build.
* Done when: Boot log shows correct partition layout, littlefs still mounts.

---

## Phase 2 — App Plumbing (State, Events, Settings)

8. **Add `app_state` container**

* Do: `include/app/app_state.h`, `src/app_state.c` with `mode=IDLE`.
* Done when: RTT prints current mode on boot.

9. **Event bus**

* Do: Tiny `event_bus` (`k_fifo` + subscriber list).
* Done when: Publish a dummy event and subscriber handler runs.

10. **Settings (NVS)**

* Do: Enable `CONFIG_SETTINGS=y`, create `config/settings.c` with `app/*` keys (e.g., `app/ble_name`).
* Done when: Setting a value, reboot, and value persists.

---

## Phase 3 — Timebase (Monotonic + GNSS tie stub)

11. **Monotonic timer**

* Do: `timebase.c` with `uint64_t time_now_us()`.
* Done when: `time_now_us()` increases monotonically and is \~us scale.

12. **UTC correlation placeholder**

* Do: `utc_from_mono()` returning “unknown” until GNSS present.
* Done when: Aggregator can call it without crash.

---

## Phase 4 — GNSS Ingest (UART + NMEA)

13. **Bring up GNSS UART**

* Do: Enable chosen `&uartX` in overlay; `UART_ASYNC` RX.
* Done when: NMEA lines arrive in a ring buffer (no parsing).

14. **NMEA line framing**

* Do: CR/LF framing, validate `*HH` checksum.
* Done when: Rejects bad lines; accepts valid `$GxGGA/$GPVTG`.

15. **Fix cache + `$PTH` hook**

* Do: Parse GGA (lat/lon/alt/time), VTG (speed/course). Maintain fix cache; expose callback for aggregator to emit `$PTH` after each.
* Done when: RTT prints parsed fields at \~2 Hz.

---

## Phase 5 — IMU (ICM-42688-V on SPI4)

Base all driver code as closely as practical to the Zephyr 42688 drivers and enumeration definitions.  Note: The "who am I" ID code for a -V variant of this chip is different than the -P variant expected by the Zephyr driver.

16. **WHO\_AM\_I**

* Do: SPI read 0x5; expect 0x4C (ICM-42688-P - not used on this PCB - is 0x47)
* Done when: RTT prints chip ID 0x4C

17. **Basic configure (ODR, ranges)**

* Do: Set 200–400 Hz internal ODR, ±2000 dps / ±16 g, FIFO on, INT1.
* Done when: Status regs report FIFO running, INT1 toggles.

18. **FIFO burst read**

* Do: INT-driven SPI burst into buffer; unpack accel/gyro, apply scale.
* Done when: Continuous sample stream at configured rate, no overflow.

19. **Ring buffer producer**

* Do: Push structs `{t_us, ax, ay, az, gx, gy, gz}` to IMU ring.
* Done when: Consumer test drains without loss for ≥10 s.

---

## Phase 6 — Baro (BMP390 on I²C)

20. **WHO\_AM\_I**

* Do: Read chip ID (0x60/0x50 depending on variant).
* Done when: RTT prints expected ID.

21. **Configure + DRDY**

* Do: Set ODR for 4–50 Hz; enable DRDY IRQ (P1.11).
* Done when: DRDY fires; read pressure + temp OK.

22. **Ring buffer producer**

* Do: Push `{t_us, pressure_pa, temp_c}` to BARO ring.
* Done when: Consumer test drains at configured rate.

---

## Phase 7 — (Optional) Magnetometer MMC5983MA

23. **Detect + single conversion**

* Do: Verify comms; take one reading.
* Done when: Bx/By/Bz (µT) printed.

24. **Periodic sampling**

* Do: Timer or INT; scale to µT; push to MAG ring.
* Done when: Stable 10–50 Hz stream.

---

## Phase 8 — Aggregator & `$Pxxx` Lines (CSV)

25. **NMEA checksum helper**

* Do: `uint8_t nmea_checksum(const char* payload, size_t n)`.
* Done when: Unit test passes with known vectors.

26. **`$PVER` builder**

* Do: Compose `$PVER` from git/semver and device kind (Tempo).
* Done when: First line of new log is valid `$PVER*HH`.

27. **`$PSFC` (session file config)**

* Do: Include rates, axes, pps=0; write at file open.
* Done when: Second line is `$PSFC*HH`.

28. **`$PIMU` at 40 Hz**

* Do: Downsample IMU ring to 40 Hz cadence; build line from latest batch within window.
* Done when: File shows \~40 lines/s of `$PIMU*HH`.

29. **`$PIM2` (quaternion)**

* Do: Use open source [Madwick Fusion library](https://github.com/xioTechnologies/Fusion) using IMU data and time delta information to maintain an orientation state.  Note: The orientation frame used to express PIMU2 values is relative to the startup frame of the sensor.  Analysis algorithms will later convert this to an estimated human body frame orientation.
* Done when: `$PIM2*HH` follows each `$PIMU`.

30. **`$PENV` at 4 Hz**

* Do: Use BARO ring latest sample on 250 ms ticks; battery field fixed to -1 (V1).
* Done when: `$PENV*HH` cadence 4 Hz.

31. **`$PTH` after each GGA/VTG**

* Do: Emit `$PTH` with tie info immediately after consuming each GNSS sentence.
* Done when: `$PTH*HH` appears after GGA/VTG pairs.

32. **`$PST` on state changes**

* Do: When `app_state.mode` changes, write a `$PST`.
* Done when: Manual mode toggle produces a `$PST*HH`.

33. **(Optional) `$PMAG`**

* Do: If MAG enabled, add line with Bx/By/Bz µT (rate ≤ IMU).
* Done when: `$PMAG*HH` lines present when enabled; absent otherwise.

---

## Phase 9 — File Writer & Session Lifecycle

34. **Async writer thread**

* Do: `k_msgq` for CSV lines → coalesce 1–4 KB → `fs_write`.
* Done when: Producer can enqueue faster than disk without blocking; writer keeps up.

35. **Flush policy**

* Do: Time-based (e.g., 250 ms) + size-based flush; `fs_sync` on stop.
* Done when: Power-cycle during logging yields intact CSV up to last flush.

36. **Backpressure counters**

* Do: If queue near full, drop least-critical (GNSS→MAG→BARO→IMU), increment counters.
* Done when: Inject artificial FS delay and observe controlled drops.

37. **Session open/close**

* Do: `logger_start()` creates directory + file, writes `$PVER/$PSFC`; `logger_stop()` closes file.
* Done when: Files appear under `/lfs/logs/YYYYMMDD/<UUID>/`.

38. **Start/stop controls**

* Do: Button long-press OR Zephyr shell cmd `log start|stop`.
* Done when: Both paths work and produce `$PST`.

---

## Phase 10 — mcumgr (BLE File Xfer + DFU)

39. **Enable SMP over BLE**

* Do: `CONFIG_MCUMGR_SMP_BT=y`, register FS mgmt group.
* Done when: `mcumgr conn` sees device advertising; `mcumgr -t 10 echo hello` works.

40. **List files**

* Do: `mcumgr fs list /lfs/logs`.
* Done when: Directory structure and CSV file show up.

41. **Read file**

* Do: `mcumgr fs read /lfs/logs/.../flight.csv > host.csv`.
* Done when: Host file matches device file byte-for-byte.

42. **Delete file**

* Do: `mcumgr fs delete /lfs/logs/.../flight.csv`.
* Done when: File disappears; device confirms.

43. **DFU smoke**

* Do: Enable image mgmt; build a dummy second image and upload.
* Done when: `mcumgr image upload` + `image test` + `image confirm` cycle succeeds.

---

## Phase 11 — Health, Safety, & Profiles

44. **Health telemetry**

* Do: Track `lines_written`, bytes, drops per stream; expose via shell `health show`.
* Done when: Values increment and reset per session.

45. **Watchdog**

* Do: Enable hardware watchdog with sane timeout; pet in main loop.
* Done when: Inject hang → watchdog resets device.

46. **Power-fail test**

* Do: Log, yank power, reboot; open CSV and verify last flush intact.
* Done when: No filesystem corruption; littlefs mounts cleanly.

47. **Build profiles**

* Do: `prj_debug.conf` and `prj_flight.conf`.
* Done when: `OVERLAY_CONFIG=prj_flight.conf` reduces logs, enables PM.

---

## Phase 12 — Validation Against Legacy

48. **Golden checksum tests**

* Do: Unit tests (ztest) for `$Pxxx` line construction with fixed inputs.
* Done when: CI passes; checksums match known vectors.

49. **Line-for-line diff**

* Do: Record a short session on V1 Arduino build and on Zephyr build under similar conditions; run `scripts/host_tools/verify_log.py` to compare cadence/shape (not values).
* Done when: All expected sentence types and cadences match; no spurious types.

50. **Throughput margin test**

* Do: Measure sustained end-to-end rate (sensor→writer) and BLE read throughput via mcumgr.
* Done when: Headroom ≥2× over target logging load; BLE ≥100–200 kB/s on desk.

---

## Phase 13 — Nice-to-Have (time-boxed)

51. **Session index sidecar**

* Do: `index.json` mapping times→byte offsets.
* Done when: iOS can `fs read` small index first to seek.

52. **LED/UI status**

* Do: Blink patterns for `IDLE/ARMED/LOGGING/ERROR`.
* Done when: Visual state reflects `app_state.mode`.

53. **Settings shell**

* Do: Commands `set imu_odr 400`, `show settings`, `save`.
* Done when: Reboot persists new values.

---

## How to hand these to an LLM runner

* Execute tasks in order; keep PRs to **one task each**.
* Each task should land with: code, a brief TEST section in the PR description (commands + expected outputs), and a log snippet or file artifact (when applicable).

---

### Quick Test Snippets (copy/paste)

* **Build/flash:**
  `west build -b nrf5340dk_nrf5340_cpuapp -p auto . && west flash`

* **mcumgr (BLE):**
  `mcumgr --conntype ble --connstring peer_name=TempoBT fs list /lfs/logs`
  `mcumgr ... fs read /lfs/logs/20250817/<uuid>/flight.csv > flight.csv`

* **Start/stop via shell:**
  `uart:~$ log start` → check `$PVER/$PSFC` + `$PST`
  `uart:~$ log stop` → file closed, size > 0

---

This sequence gets you to a **usable MVP**: reliable CSV logs to QSPI, correct `$Pxxx` cadence & checksums, and **iOS-ready** transfers via mcumgr/FS—plus DFU. Every step is bite-sized and verifiable on the bench.
