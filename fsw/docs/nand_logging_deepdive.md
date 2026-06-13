# QSPI NAND logging deep-dive — working notes

Goal: determine whether the QSPI NAND freezes/`LFS_ERR_IO` are our *implementation*
(software) vs the chip being bad, by studying the correct way to drive a W25N01
QSPI NAND on a Teensy 4.1, then redesign APEX's flash logging cleanly. Exhaust
software causes before blaming hardware.

Status legend: [ ] todo  [~] in progress  [x] done

## Progress
- [x] Quick fix: SD flush 1 s → 30 s (+ one-shot 10 s into DESCENT, + post-landing dump). config.h `LOG_SD_FLUSH_INTERVAL_MS`/`LOG_SD_POST_APOGEE_FLUSH_MS`, storage.cpp flush_logs decoupled flash/SD. Builds on all 3 envs.
- [~] Deep-dive research (this file)
- [ ] Reference design
- [ ] Refactor

## Hardware facts (from this session)
- Chip: Winbond **W25N01GVZEIG**, 1 Gbit (128 MiB) SPI NAND. On Teensy 4.1 it's
  on **FlexSPI2** (the dedicated QSPI memory footprint on the bottom of the board).
- Driver: Teensy core `LittleFS_QPINAND` (LittleFS/src/LittleFS_QPINAND.* +
  LittleFS_NAND.cpp). page/prog size 2048 B, block 131072 B (128 KiB), progtime
  2000 µs, erasetime 15000 µs. ECC on-chip (0xC0 status, BUF=1/ECC=1 read mode).
- NAND `wait(us)` polls `isReady()`, `yield()`s between polls, returns `LFS_ERR_IO`
  on timeout (prog 2 ms / erase 15 ms).

## Evidence (debug serial, 15:18:01–15:18:16)
- `flush=1000957`, `flush=1001008`, `update=1001025` — ~1.001 s flush stalls that
  SUCCEED (faults=0). Too uniform/long to be the NAND wait (2/15 ms) → that one is
  the **SD** flush (now mitigated by the 30 s change). [confirm: was SD present]
- `flash` write max_us climbing 1488→4869 over ~14 s — QSPI write latency growing.
- `QSPI write detail: requested=132 wrote=-5 (LFS_ERR_IO) pos=4246 size=4246
  used=4992/128512 KB elapsed_us=222059` then `fault 0x0001` — a 132 B write spent
  **222 ms** (≫ 2 ms progtime) then hard-failed. 222 ms = many internal NAND ops
  (retries/relocations/GC), i.e. LittleFS doing a lot of work for one small write.

## Suspicions to investigate (software-first, per user)
1. **`yield()` re-entrancy in NAND wait()**: the driver `yield()`s while polling.
   On Teensy, `yield()` runs EventResponder/serialEvent and **USB MTP** servicing.
   If anything in the yield path touches the NAND while we're mid-op (we're INSIDE
   a NAND wait), that's re-entrant FlexSPI2 access → corruption / IO error. MTP.loop
   now runs every loop iteration (export-mode removal). **Prime suspect.**
2. **Write/flush pattern bad for NAND**: tiny records (51–160 B) + frequent flush
   → partial-page programs + frequent metadata commits → write amplification, GC,
   wear. NAND wants page-aligned (2048 B) writes. We write unaligned small records
   and flush often → LittleFS rewrites metadata pages constantly.
3. **No write batching / buffering**: every record is an individual `File.write`.
4. **ISR preemption**: fusion(200)/baro(50)/mag(25) IntervalTimer ISRs read IMU
   (SPI0) / baro,mag,gps (I2C). Different peripherals than FlexSPI2 → preemption
   should NOT corrupt a FlexSPI2 transaction, only delay it. CONFIRM no shared bus.
5. **LittleFS config**: lookahead/cache size, block_cycles (wear-leveling), and
   whether `block_count` matches the real usable blocks after bad-block reserve.
6. **Bad-block management / ECC**: is the QPINAND driver handling bad blocks +
   ECC correctly, or does an ECC-uncorrectable page just return IO error?

## "It got worse as the system grew" (user's key observation)
The IDLE rate is low (2 Hz) but we added: boot record, event records, the 20 Hz
prelaunch RAM ring (flushed at launch), 1 s flush. The *flush frequency* + many
small writes drive metadata churn. Need to quantify writes/sec and bytes/sec and
compare to NAND-friendly patterns.

## Research findings
- PJRC LittleFS README: for logging, prefer **binary** `write()` (we do), and
  "repeatedly removing and writing files can degrade LittleFS performance."
- LittleFS is known for **high write amplification on small/random writes**;
  random write throughput on NAND is poor (littlefs issues #11, #361, #422;
  STM32 W25N01GVZEIG threads). This is inherent to the FS, not the chip.
- **Interrupt-disable scope (verified in the Teensy driver, not just docs):**
  The "all interrupts disabled during write/erase" caveat applies to the
  **program-flash** backend (`LittleFS_Program`), NOT QSPI NAND. The QSPI NAND
  path (`flexspi2_ip_command`/`flexspi2_ip_read` + `wait()`+`yield()`) busy-waits
  on `FLEXSPI2_INTR` and `yield()`s — **interrupts stay enabled.** Confirmed: no
  `noInterrupts`/`__disable_irq` anywhere in LittleFS_NAND.cpp.
- NAND geometry (W25N01): prog/page size **2048 B**, erase block **131072 B**,
  progtime 2 ms, erasetime 15 ms. LittleFS config uses cache_size = prog_size =
  2048, block_size = 131072.

## Interrupt-safety analysis (the user's core question)
- **Can a sensor ISR corrupt a NAND op?** No. Sensors are on SPI0 (IMU) and
  I2C0/1/2 (baro/highg/mag/gps); the radio on SPI1; the NAND on **FlexSPI2** —
  separate peripherals. An ISR preempting a NAND op runs on its own bus and
  returns; the FlexSPI2 transaction state is hardware-held. Preemption only
  *delays* the NAND op, which is fine (it polls with timeout).
- **Does a NAND op block the ISRs?** No — interrupts stay enabled (above), so the
  200 Hz fusion ISR keeps sampling even during a 222 ms NAND stall. Only the
  **main loop** busy-waits → loop-level freeze (telemetry, buzzer, serial) but
  control/estimation continue. Good for flight safety; explains the visible
  "serial pauses" without losing the estimator.
- **Re-entrancy via `yield()`:** the NAND `wait()` calls `yield()`, which runs
  EventResponder/serialEvent. If anything in that path re-entered the NAND
  (e.g. MTP servicing) *while we're inside a NAND op*, that WOULD corrupt the
  FlexSPI2 transaction. APEX calls `MTP.loop()` from the main loop (not a yield
  hook), so logging and MTP are sequential, not concurrent — currently safe.
  But it's fragile: the driver has **no re-entrancy guard**. Design rule going
  forward: exactly one NAND writer (the main loop), and MTP servicing must not
  overlap an in-progress write (gate it, see below).
- **Verdict:** the freezes are busy-wait stalls (not interrupt corruption), and
  `LFS_ERR_IO` is NAND-level (ECC-uncorrectable / failed program), *accelerated
  by our write amplification*. So: partly the chip wearing, but the wear and the
  stalls are driven by how we write. Implementation IS a real contributor.

## Root cause (synthesis)
Our write pattern is near worst-case for LittleFS-on-NAND:
- Records are 51–160 B; LittleFS caches them in a 2048 B page buffer.
- We `flush()` **every 1 s**. At the 2 Hz IDLE rate that's ~2 records (~300 B)
  per flush → each flush programs a mostly-empty 2048 B page **and** commits
  metadata → ~1 near-wasted page program/sec + metadata churn.
- Result: write amplification → accelerated wear/GC → climbing latency
  (1.5→4.8 ms observed) → eventually an ECC-uncorrectable page → `LFS_ERR_IO`.
This matches "it got worse as the system grew" (more record types + flushes).

## Reference design (clean, APEX-specific flash logger)
Principle: feed LittleFS **page-aligned, batched** writes and flush on a bounded
policy, not per-record. Keep one writer (main loop). Back off on faults.

1. **RAM page buffer (2048 B).** `storage_log_*` append serialized records into
   a page buffer. When it reaches a full page, write the whole 2048 B to the
   LittleFS file in one `write()` → full-page programs, ~zero amplification.
2. **Bounded flush policy.** Flush (fsync) the file when (a) a page was just
   written, or (b) a max interval elapsed (default **2 s**) — whichever first.
   This bounds power-cut data loss to ≤ one partial page + 2 s, while removing
   the per-second partial-page churn. Phase boundaries (BOOST entry, LANDED)
   still force a flush.
3. **Fault back-off.** Count consecutive `LFS_ERR_IO`/short writes; after N
   (e.g. 5) suspend QSPI writes for the session (stop the repeated ~200 ms
   freezes on a dead chip), latch `LOG_FAULT_FLASH_WRITE`, keep buzzer FAULT.
   Re-enabled only by reboot/format.
4. **Single writer + MTP gate.** Never write the NAND from an ISR. Don't service
   MTP while a high-rate log is mid-flight (or at least never inside a NAND op).
5. **(Optional, bigger)** raw page-structured append log bypassing LittleFS for
   the flight black box — eliminates FS write amplification entirely but loses
   MTP/file access. Defer; revisit if batching is insufficient.

Trade-off needing sign-off: batching introduces a **data-loss window** (records
in the RAM page buffer not yet written). Default proposal bounds it to ≤ ~2 s +
one partial page. Acceptable for a black box? (Flight rate fills a page in ~0.1 s
so in-flight loss is tiny; IDLE loss is irrelevant.)

## Refactor plan / where I left off
DONE: SD flush decoupled to 30 s + post-apogee one-shot (config.h + storage.cpp).
NEXT (the flash logger refactor):
1. Add a 2048 B page buffer + append in `write_raw`/`write_record` (QSPI side).
2. Replace per-write flush with the bounded policy (page-fill OR 2 s OR forced).
3. Add the consecutive-fault back-off counter + suspend.
4. Keep SD path as-is (mirror, already de-amplified by rare flush).
5. Build all 3 envs; HIL flight on hardware; pull log, decode, verify integrity;
   watch the debug `Storage dbg` line: flash max_us should drop and stay low,
   no climbing latency, far fewer page programs.
RESOLVED: MTP_Teensy does NOT register a yield/EventResponder hook — `MTP.loop()`
is only invoked explicitly from the main loop. So the NAND `wait()`→`yield()`
cannot re-enter MTP, and logging vs MTP are sequential (single-threaded). No
re-entrancy today; keep the single-writer rule so it stays that way.
