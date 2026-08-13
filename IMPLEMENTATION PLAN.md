# PID Auto-Tune for Servo Dome/Roof Axis

## Context

OCS drives its dome/roof azimuth axis with a DC servo motor + quadrature encoder, closed by a PID loop (`Pid` wrapping the QuickPID library) inside `ServoMotor::poll()`. Gains are currently only set at compile time (`Config.h`/`Extended.config.h`) or by hand through a currently-unwired `:SXA` serial command, with no bounds validation. Getting good P/I/D values today means manual trial-and-error on real hardware, which is slow and error-prone, especially since backlash, friction and inertia vary per installation.

The goal is an on-device auto-tune routine that (a) uses an industry-recognized identification method rather than ad-hoc heuristics, and (b) proves the resulting gains by driving the axis through its actual production motion path — the same `Axis::autoGoto()` trapezoidal goto used by a real `:DS#`/`:Dz#` dome slew.

**Method, and why:** relay feedback (Åström–Hägglund) — the usual textbook "industry standard" — was considered and explicitly rejected: it works by forcing a sustained on/off oscillation through the actuator, and a geared dome/roof drive repeatedly slamming direction at a bounded but nontrivial amplitude risks real mechanical wear (gear lash hammering, bearing/belt stress) that a normal slew never produces. This matters doubly here because the target installation has **substantial mechanical backlash** — every commanded reversal hammers across the lash band. Instead this plan uses **closed-loop step-response ("bump test") tuning with iterative correction**, grounded in two established, non-oscillatory techniques:

- The classical _process reaction curve / bump test_ — apply a move and read process gain, rise time, overshoot, and settling time off the response — is the direct non-oscillatory sibling of relay feedback (Ziegler and Nichols themselves published both an oscillation-based method and an open-loop step/reaction-curve method) and is what most commercial servo-drive auto-tuners use in practice for exactly this reason: it never asks the actuator to do anything a normal move wouldn't.
- **Iterative Feedback Tuning (IFT)** (Hjalmarsson et al.) formalizes adjusting controller parameters directly from data gathered on the real closed-loop system operating under its normal reference signals, with no special identification signal at all. Strictly, formal IFT computes gradient estimates from paired experiments; what this plan implements is an *IFT-inspired* bounded heuristic correction from repeated ordinary experiments — the citation grounds the approach (tune on the real closed loop, under production-like moves, iteratively), not the exact update law.

Concretely: each tuning _iteration_ commands several real, bounded `axis->autoGoto()` moves — identical in character to any slew the dome already performs — and measures overshoot and settling time on each. Because a physical dome/roof drive has real mechanical variation around its travel (uneven track/rail surfaces, bearing friction that varies with position, load/weight distribution shifting the friction torque, wind), a single move's measurement is noisy and not representative on its own. Each iteration therefore repeats the test move several times (spread around different points in the dome's travel, not the same spot every time) and combines the results with a robust statistical filter (below) before feeding a single, denoised overshoot/settle-time pair into the gain-correction rule. Gains are then nudged by a bounded, classical PID-tuning correction rule (reduce Kp/raise Kd on excess overshoot, raise Kp on sluggish response, raise Ki on residual position error) and the process repeats for up to a fixed number of iterations until the response meets acceptance criteria. The dome never does anything but ordinary slews.

Scope: tune only the goto/slewing gain set (`param4-6`); leave the tracking/holding set (`param1-3`) untouched. Gains are reported over serial and require an explicit confirm command to persist — nothing auto-persists. The feature is wired through `Axis::command()`, which also fixes the fact that `:GXA`/`:SXA`/`:GXS`/`:GXU` currently exist in the shared axis library but aren't reachable from `Dome`'s command dispatch in this build. (`Roof` owns no `Axis` at all in OCS — it is relay/direct driven — so no Roof changes are needed; the feature is dome-azimuth `axis1` and, where present, dome-altitude `axis2`.)

**Inherent reset-safety (worth stating explicitly):** candidate gains live only in RAM until the explicit `:SXT[n],2#` confirm writes NV. An MCU reset or power loss mid-tune therefore automatically reverts to the stored gains on boot via `Axis::init()`'s NV read — no half-tuned state can survive a restart.

## Verified codebase assumptions

These were checked against the source and the design depends on them:

1. **Gain scheduling is always in "auto scale" mode in OCS.** `Pid` is constructed without a sensitivity argument (`Dome.axis.cpp:36`), so `autoScaleParameters` is always true and pure `param4-6` are the active gains during a slew (`Servo.cpp:400-406`). If a sensitivity were ever passed, the firmware would switch to continuous velocity-blended gains (`Servo.cpp:407-408`) and the "fixed gains during the test move" model breaks. **Guard this:** auto-tune start must check `feedback->autoScaleParameters` and refuse (status = unsupported-mode) if false.
2. **The dome axis is built with `targetTolerance = 0`** (`Dome.axis.cpp:59` omits the constructor argument), so `Axis::atTarget()` requires an *exact* encoder hit (`Axis.cpp:253-255`). With heavy backlash this is a real operational problem (see §3b) and it means auto-tune must never key its settle logic off `atTarget()`.
3. **Post-slew gain-schedule transition:** after `Axis::poll()` reaches target it calls `motor->setSlewing(false)`; slewing gains then remain selected only for `SERVO_SLEWING_TO_TRACKING_DELAY` (3000 ms default), after which `selectTrackingParameters()` fires and `Pid::poll()` ramps the gains to the tracking set over `PID_SLEWING_TO_TRACKING_TIME_MS` (1000 ms default). Any measurement taken later than ~3 s after arrival reflects `param1-3` — the set we are *not* tuning (§3, AT_MONITOR).
4. **Servo safety shutdowns exist and can be tripped by candidate gains:** stall (≥`SERVO_SAFETY_STALL_POWER` with no motion), runaway (>90% power moving away from target), and oscillation (power swinging below −33% and above +33% within 2 s) all call `enable(false)` and latch `safetyShutdown` (`Servo.cpp:414-448`). Critically, `ServoMotor::enable(true)` *clears* `safetyShutdown` (`Servo.cpp:137`) — so a naive retry would blindly re-arm the motor with the same bad gains. §3 treats shutdown as a first-class event.
5. The between-iteration gain-application mechanism is valid: `ServoMotor::enable(false)` → `feedback->reset()` (`Servo.cpp:137`), and `Pid::reset()` unconditionally reloads `param4-6` into `SetTunings()` and clears accumulated state (`Pid.cpp:38-56`). This is needed because `selectSlewingParameters()` only applies tunings on the tracking→slewing transition (`Pid.cpp:73-88`).
6. No `:SXT`/`:GXT` command collisions exist anywhere in the codebase, and nothing currently routes any command to `axis1.command()` (`Dome.command.cpp:143-144` returns false for all non-`:D` commands).

## Design

### 1. `Pid` — bounds validation only (`src/lib/axis/motor/servo/feedback/Pid/Pid.h`, `Pid.cpp`)

The tuning loop itself needs no relay/oscillation logic inside `Pid`, so this class needs only one change: override `validateParameters()` (currently `Feedback::validateParameters()` at `FeedbackBase.cpp:49-51` always returns `true` — no bounds checking exists today, for manual `:SXA` writes either) to reject NaN/negative gains and enforce a sane upper bound. Reused both for auto-tune's candidate gains and regular manual sets.

### 2. Forcing new gains to take effect between iterations

`Pid::selectSlewingParameters()` (`Pid.cpp:73-88`) only calls `pid->SetTunings()` on the tracking→slewing _transition_ (guarded by `if (trackingSelected)`); calling `motor->setParameters(...)` alone while the axis is still within the post-slew "slewing selected" grace window (`SERVO_SLEWING_TO_TRACKING_DELAY`, default 3000 ms, `Servo.h:23`) would silently not take effect for the next test move. Rather than touching `Pid.cpp`, reuse the existing public `ServoMotor::enable(bool)` (`Servo.cpp:133-139`): disabling calls `feedback->reset()`, which (`Pid.cpp:38-56`) always reloads `param4/5/6` into `pid->SetTunings()` and clears the integral accumulator — exactly what's wanted between iterations (fresh tunings, no residual windup from the previous move). So each iteration's gain update is simply: `motor->setParameters(...)` then `motor->enable(false); motor->enable(true);` before the next `autoGoto()`. No new API needed on `Pid`/`ServoMotor`.

Two caveats now made explicit:

- Because `enable(true)` clears `safetyShutdown`, this toggle must only be executed on the normal iteration path — never as a recovery from a detected safety shutdown without first applying the mandatory gain backoff (§3, safety handling).
- The toggle leaves `Axis::enabled` untouched (still true), so the subsequent `autoGoto()` passes its `!enabled` check. If `AXIS1_POWER_DOWN` is `ON` the axis-level power-down logic may independently disable the motor after prolonged standstill; the per-move toggle immediately before each `autoGoto()` re-enables regardless, but a verification case covers this interaction (§Verification 8).

### 3. Orchestration — `Axis` class (`src/lib/axis/Axis.h`, `Axis.cpp`)

New state machine (guarded by `#ifdef SERVO_MOTOR_PRESENT` and the `SERVO_PID_AUTOTUNE` feature flag), driven from inside `Axis::poll()` (`Axis.cpp:430`) alongside the existing `autoRate` handling. States: `AT_IDLE`, `AT_SPEED` (§3c, once per run), `AT_PRELOAD`, `AT_MOVE`, `AT_MONITOR`, `AT_AGGREGATE`, `AT_ANALYZE`, `AT_DONE_SUCCESS`, `AT_DONE_FAIL`.

Each **iteration** (bounded at `PID_AUTOTUNE_MAX_ITERATIONS`, default 6) runs a **round** of `PID_AUTOTUNE_REPEATS` (default 5) repeated test moves before updating the gains:

1. **AT_PRELOAD** (before *every* measured test move) — command a short, unmeasured slew of `PID_AUTOTUNE_PRELOAD_DISTANCE` in the **same direction as the upcoming test move**, wait for it to complete plus a brief dwell, and only then record the start position for the measured move. Purpose: the gear train enters every measured move with the lash fully taken up in the test direction, so every repeat starts from the identical mechanical state and results are comparable across repeats, rounds, and gain sets. This is done before every repeat — not just after direction changes — because the lash state after a completed move is actually indeterminate: a move that overshoots and gets pulled back by the PID ends with the train loaded in the *reverse* direction, so "the previous move went the same way" is not a guarantee. Whenever the walk direction changes (between rounds with wrap ON, every return leg with wrap OFF), the preload is what re-establishes the known lash state; with the preload unconditional, direction changes need no special casing. The preload also makes the measurements independent of how accurately the firmware's configured backlash compensation (`Axis::setBacklash()`/`backlashSteps`) matches the true mechanical lash — important here, where lash is substantial and the configured value may under-represent it. The preload move is an ordinary bounded `autoGoto()` with its own short timeout; its distance is included in the travel-margin bookkeeping, and it is entirely excluded from measurement.

2. **AT_MOVE** (repeated `PID_AUTOTUNE_REPEATS` times per iteration) — record current position, pick a target `PID_AUTOTUNE_TEST_DISTANCE` away *in the preloaded direction*, and call `setTargetCoordinate()` + `autoGoto(validationSlewRate)` — the exact same pair `Dome::gotoAzimuthTarget()` uses for a live `:DS#` (`Dome.cpp:98-112`). This is the only motion auto-tune ever commands: one ordinary trapezoidal-ramp slew.

   **Backlash-aware test-motion pattern** (this installation has substantial lash, so reversals are both a wear source and a measurement contaminant — every reversal spends time traversing the dead band at `backlashFreq` before the load moves, and `Axis::poll()` even skips its slew-rate logic while `motor->inBacklash`):
   - **When `AXIS1_WRAP == ON` (dome azimuth, typical):** walk *unidirectionally* around the full travel — every test move steps the same direction, wrapping through 360°. No reversals at all during a round, so no backlash traversal ever lands inside a measurement, and the moves naturally sample different track/bearing sections. Alternate the walk direction between *rounds* (not moves) so both directions get tuned coverage across the run while keeping reversals to one per round.
   - **When wrap is OFF (bounded travel):** use out-and-back pairs — measure only the *outbound* move of each pair; the return leg is unmeasured repositioning. Base positions still step along the travel between pairs. All targets must keep a margin of `TEST_DISTANCE` + overshoot headroom (not merely containment) from `settings.limits.min/max`, so late repeats near a limit neither fail `autoGoto()`'s limit check nor overshoot into the motion-error abort.
   - With the unconditional AT_PRELOAD step, a *measured* move never begins with a direction reversal — the preload absorbs it. As belt-and-suspenders (e.g. residual takeup if the preload was short of the true lash), the per-move timers still start only once `motor->inBacklash` is false, so any leftover takeup time stays out of rise/settle measurement.

3. **AT_MONITOR** — while the move is in progress and then through the settle window, stream (no buffering of the trajectory itself — only running peak/timer values) the measurement, using `Axis::getTargetDistance()` against auto-tune's **own settle band** — **never `atTarget()`**, which with the dome's `targetTolerance = 0` demands an exact encoder count and would false-reset the settle dwell on every ±1-count flicker. Details:

   - **Settle band:** `PID_AUTOTUNE_SETTLE_TOLERANCE_COUNTS`, defaulting to `AUTO` = max(configured value, 1.5 × current backlash in steps via `motor->getBacklashSteps()`, 2 encoder counts). With real lash the load physically cannot be positioned more finely than the dead band; a band narrower than the lash guarantees hunting and meaningless settle times.
   - **Overshoot is measured in absolute encoder counts** (peak excursion beyond the target position in the direction of travel), not as a percentage of move distance — percent-of-distance scales inversely with `TEST_DISTANCE` and makes thresholds meaningless when the user overrides the distance. The acceptance threshold is `PID_AUTOTUNE_MAX_OVERSHOOT_COUNTS` (default `AUTO` = 2 × settle band). Note that measurable overshoot is floored/quantized by the lash width: an overshoot smaller than the lash may never be pulled back (the load parks inside the dead band, which is fine and counts as settled).
   - **Settled** = `getTargetDistance()` continuously within the settle band for `PID_AUTOTUNE_SETTLE_CONFIRM_MS`.
   - **Measurement window constraint:** all post-arrival measurement (settle confirm + residual-error read) must complete while the *slewing* gain set is still active. After arrival, `Axis::poll()` calls `motor->setSlewing(false)` and the slewing set survives only `SERVO_SLEWING_TO_TRACKING_DELAY` (3 s), then blends to the untouched tracking set over 1 s (`Servo.cpp:400-406`, `Pid.h:53-64`) — data taken after that would attribute tracking-gain behavior to the gains being tuned. Enforcement: once the goto completes, the state machine calls `motor->setSlewing(true)` to hold the slewing set selected for the remainder of AT_MONITOR (the flag's only servo-side effect is parameter selection and `lastSlewingTime`), and sets it false again when the repeat's measurement ends. This removes the 3-second race entirely; `SETTLE_CONFIRM_MS` no longer has a hidden upper bound.
   - **Residual position error** (read after settle confirm): note this is *frozen following error*, not classical steady-state error — when the goto ends, the commanded `motorSteps` trajectory freezes wherever it is, and the PID thereafter holds the encoder at `motorSteps`, not at `targetSteps`. Raising Ki (param5) shrinks following error during the move and therefore shrinks this frozen gap, so the correction rule is directionally valid — but it must be understood (and commented in code) as following-error reduction so a future maintainer doesn't "fix" it. Residual error smaller than the lash band is treated as zero.
   - **Hunting/limit-cycle detection:** count settle-band crossings after first entry. More than `PID_AUTOTUNE_MAX_BAND_CROSSINGS` (default 4) crossings classifies the repeat as *hunting* — the classic integrator-plus-backlash limit cycle — which feeds AT_ANALYZE as an "oscillatory" outcome (reduce Ki and/or Kp) rather than being misread as a long settle time.
   - On completion, store this one repeat's `(overshootCounts, settleTimeMs, residualErrorCounts, huntingFlag)` into a small fixed-size array sized `PID_AUTOTUNE_REPEATS` (trivial RAM even on a 2KB AVR).
   - A per-move watchdog (`PID_AUTOTUNE_MOVE_TIMEOUT_MS`) plus the existing `motionError()`/`motorFault()` checks abort the run — restoring the pre-tune gains — if anything goes wrong. The watchdog also covers the pathological case where the goto itself never terminates: with `targetTolerance = 0`, `AR_RATE_BY_DISTANCE` only ends on an exact encoder crossing, and a high-lash axis can hunt indefinitely. On watchdog expiry issue `autoSlewAbort()`, then abort or backoff per the safety rules below.

   **Safety shutdown is a first-class event, not a bad sample.** Each poll in AT_MOVE/AT_MONITOR checks `motor->enabled` (the hook already exists — `Axis.cpp:564-568` reacts to the motor disabling itself) and `getDriverStatus().fault`. If the servo safety layer tripped (stall/runaway/oscillation, `Servo.cpp:414-448`):
   - discard the entire round in progress (all repeats),
   - apply a mandatory backoff to the candidate gains — multiply Kp, Ki, Kd by `PID_AUTOTUNE_SAFETY_BACKOFF_PERCENT` (default 50%) — before any re-enable, since `enable(true)` clears `safetyShutdown` and would otherwise re-arm the motor with the exact gains that just tripped it,
   - increment a shutdown counter; on the second shutdown in one run, abort entirely and restore pre-tune gains.

4. **AT_AGGREGATE** (once all repeats in the round are collected) — combine the round's samples into one robust overshoot/settle-time/residual-error triple using the median + modified-Z-score outlier filter (below), instead of a plain average that a single bad reading (surface snag, gust of wind, a moment of unusually high static friction) could skew. Any hunting-flagged repeat forces the round's outcome to "oscillatory" regardless of the numeric aggregate.

5. **AT_ANALYZE** — compare the round's aggregated results to the acceptance thresholds. If within bounds, mark success and stage the current `param4-6` as the validated result. Otherwise apply a bounded classical correction and start the next round at step 1:
   - overshoot too high, or round flagged oscillatory/hunting → reduce Kp and/or raise Kd; hunting specifically also reduces Ki (integrator + lash limit cycle)
   - sluggish, no overshoot, slow settle → raise Kp
   - residual position error beyond the lash band after settling → raise Ki
   - each correction clamped to a maximum per-iteration change (`PID_AUTOTUNE_GAIN_STEP_LIMIT_PERCENT`, default 40%) so the loop can't swing wildly between iterations.

   The correction *magnitude* may be scaled using the classical second-order `%OS = 100·exp(-ζπ/√(1-ζ²))` step-response relation as a bounded heuristic only — it is explicitly *not* an accurate model here, because the PID tracks a decelerating trapezoidal setpoint (`control->set` follows the ramped `motorSteps` trajectory, `Servo.cpp:371`), not a step, and QuickPID runs `dOnMeas`/`pOnError`. The direction rules above carry the real logic; the clamp carries the real safety. A plain proportional correction is an acceptable simplification if the ζ inversion proves noisy on hardware.

6. On reaching the iteration cap without meeting thresholds, report the best-scoring iteration's gains (tracked with simple running-best bookkeeping, no full history buffer) flagged as "did not fully converge" — still requires the same explicit confirm as a full success, never auto-persisted either way.

With the defaults above this is up to 6 iterations × 5 repeats = 30 ordinary bounded measured slews worst case, each preceded by a short preload nudge (plus return legs when wrap is off); the iteration/repeat constants are configurable to trade test duration against confidence.

`autoTuneApply()` re-applies the validated gains live and writes them to NV via the exact path `Axis.command.cpp`'s `:SXA` handler already uses (`nv.updateBytes(NV_AXIS_SETTINGS_BASE + ...)` + `motor->setParameters(...)`, `Axis.command.cpp:129-130`) — no new persistence mechanism.

### 3a. Statistical aggregation and outlier rejection

For combining each round's repeated measurements, the standard small-sample robust-statistics options:

- **Chauvenet's criterion** rejects based on deviation from the _mean_/_standard deviation_ — but mean and standard deviation are themselves not robust: a single bad measurement can inflate the standard deviation enough to mask itself and other outliers ("masking"), and it assumes an approximately normal distribution, a poor fit for 5-10 samples.
- **Grubbs' test** shares the normality assumption, detects only one outlier per pass, and — per the standard guidance — should not be used at all for n ≤ 6, where it frequently over-flags points. A 5-repeat round is squarely inside its exclusion zone.
- **Modified Z-score (Iglewicz & Hoaglin)** — `Mi = 0.6745 × (xi − median) / MAD`, where MAD is the median absolute deviation from the median — uses the median and MAD instead of mean/stdev, so it isn't skewed by the very outliers it's screening for, works well down to very small n, and needs only a median (an O(n log n) sort of ≤10 elements) plus a handful of subtractions — negligible cost on any supported MCU.

**Chosen: modified Z-score with the standard Iglewicz–Hoaglin threshold (reject `|Mi| > 3.5`)**, applied independently to the round's overshoot, settle-time, and residual-error arrays; the surviving samples are averaged. For the degenerate `MAD == 0` case (too few distinct values): the canonical Iglewicz–Hoaglin refinement substitutes the mean absolute deviation (constant 0.7979), but for firmware simplicity we fall back to the plain median of the round — with all-identical samples the two are equivalent anyway, and the simpler path is easier to unit-check.

### 3b. Backlash and encoder-tolerance handling (summary)

The target installation has significant lash and the axis is currently built with `targetTolerance = 0`, so this deserves its own checklist:

- Auto-tune's settle/overshoot logic uses its own counts-based band sized ≥ 1.5 × lash (never `atTarget()`), measures overshoot in absolute counts, treats sub-lash residual error as zero, excludes backlash-takeup time from timing, avoids reversals inside measured moves, and detects lash-induced limit cycling explicitly (§3).
- Every measured move is preceded by an unmeasured same-direction preload nudge (AT_PRELOAD) so the gear train always enters the measurement with lash fully taken up in the test direction — repeats start from an identical mechanical state regardless of how the previous move ended (an overshoot pull-back leaves the train loaded the *wrong* way) and regardless of whether the configured backlash compensation matches the true lash. Any direction change is automatically re-compensated because the preload runs before every measured move.
- **Separately, add an `AXIS1_TARGET_TOLERANCE` config macro (degrees, default 0.0 = exactly current behavior) passed as the `Axis` constructor's `targetTolerance` in `Dome.axis.cpp:59`** (and `AXIS2_TARGET_TOLERANCE` where axis2 exists). This is a general fix, not just for tuning: with tolerance 0 and real lash, every production `:DS#` goto can only terminate on an exact encoder-count crossing and may hunt at the target. High-backlash installs should set this to roughly the lash width so gotos terminate cleanly inside the dead band. Auto-tune works with it at 0 (the watchdog covers non-terminating gotos), but the tuned system behaves far better with it set — the docs for the feature should say so.
- Minimum test distance sanity check at start: `PID_AUTOTUNE_TEST_DISTANCE` must be ≥ 20 × the settle band (in measure units) so measurements have dynamic range against lash + encoder quantization; refuse to start otherwise (status = bad-test-distance).

### 3c. Physical maximum rotation rate measurement (`AT_SPEED`)

Runs once at the very start of every run (opt-out via `PID_AUTOTUNE_SPEED_TEST OFF`), before any tuning round, and serves two purposes: it reports the drive's true capability so `AXIS1_SLEW_RATE_DESIRED` can be configured to something the hardware can actually do, and it protects the tuning itself — if the configured slew rate exceeds the physical maximum, every test move saturates the drive and the overshoot/settle data is meaningless.

Method: after the usual preload nudge, command one long ordinary `autoGoto()` (`PID_AUTOTUNE_SPEED_TEST_DISTANCE`, default 3 × test distance) at a deliberately unattainable rate (`PID_AUTOTUNE_SPEED_TEST_RATE`, default 2 × the test slew rate). The servo output saturates (`velocityPercent` ≥ `PID_AUTOTUNE_SPEED_SATURATION_PERCENT`, default 90%) and the encoder velocity plateaus at the physical ceiling. Encoder position is sampled at ~50 Hz, smoothed with a ~200 ms IIR, and the measured maximum is the smoothed-velocity peak during saturation.

Implementation notes, verified against the code:

- `Axis::setFrequencySlew()` and `setFrequency()` clamp to `maxFreq`, which `Dome::init()` sets to `AXIS1_SLEW_RATE_DESIRED` — an over-speed command would be silently limited to production speed. The speed test therefore temporarily lifts `maxFreq` (saving `maxFreq`/`slewFreq`) and restores both on every exit path: normal completion, watchdog timeout, safety shutdown, abort.
- The move stays a normal trapezoid: for servo axes the goto's rate curve derives from *encoder* distance-to-target, and the commanded `motorSteps` trajectory parks at the target while the PID brings the lagging encoder in — deceleration comes naturally from the shrinking error; no special end-of-move handling is needed.
- The existing servo safety detectors remain correct during saturation: stall requires *no* encoder motion, runaway requires moving *away* from the target — a hard cruise toward the target trips neither.
- If the drive never saturates (its true maximum exceeds the commanded rate), the peak velocity is still reported but flagged `saturated = 0` — meaning "at least this fast", not a ceiling.
- After the measurement, if the tuning slew rate exceeds `PID_AUTOTUNE_SPEED_HEADROOM_PERCENT` (default 85%) of a *saturated* measurement, the run's test rate is clamped to that recommended value (logged); the measured rate itself is reported via `:GXT` so the user can update `AXIS1_SLEW_RATE_DESIRED` in Config.h.

### 4. Serial command interface (`src/lib/axis/Axis.command.cpp`)

New commands alongside the existing `:GXA`/`:SXA`/`:GXS`/`:GXU` (same `index = parameter[1]-'1'` axis-selection convention):

- `:SXT[n],1[,dist]#` — start auto-tune (optional test-distance override, else config default)
- `:SXT[n],0#` — abort, restoring pre-tune gains
- `:SXT[n],2#` — apply/persist last validated (or best-effort) result
- `:GXT[n]#` — status/result: state, iteration count, and once done: current staged `Kp,Ki,Kd`, measured overshoot (counts), settleTimeMs, converged flag, **measured physical maximum rate (deg/s) and a saturated flag** (1 = the drive output saturated, so the value is the true ceiling; 0 = "at least this fast"); distinct status codes for the refusal reasons (parked, interlock, unsupported-mode, bad-test-distance) and failure causes (safety-shutdown, timeout, fault)

**Wiring fix:** `Dome::command()` (`Dome.command.cpp:143-144`) currently ends its axis-unrelated command block with a bare `else return false;`, so `Axis::command()` is never reached. Change that fallthrough to try `axis1.command(...)` (and `axis2.command(...)` where present) before finally returning false — this activates the new `:SXT`/`:GXT` commands and simultaneously fixes the pre-existing orphaned `:GXA`/`:SXA`/`:GXS`/`:GXU`. (Dispatch order in `Observatory.command.cpp` tries `roof.command` before `dome.command`; roof returns false for all `:GX`/`:SX` forms, so no collision.)

**Dome-level safety gating (required):** the auto-tune state machine drives `autoGoto()` from inside `Axis::poll()`, *below* the checks `Dome::gotoAzimuthTarget()` performs — parked state (`CE_SLEW_ERR_IN_PARK`, `Dome.cpp:102`) and the `DOME_SHUTTER_LOCK`/`roof.open()` interlock (`Dome.cpp:99-101`). Without a gate, `:SXT1,1#` would move a parked dome that a normal `:DS#` would refuse to move. Therefore `Dome::command()` intercepts the `:SXT[n],1…` *start* form before delegating: if `settings.park.state >= PS_PARKED`, or the shutter-lock interlock is active, reply with the corresponding refusal and do not forward to `axis1.command()`. Abort/apply/status forms pass through ungated (aborting or reading status must always work). The same pattern applies to axis2 where present.

### 5. Config defaults (`src/Config.defaults.h`, `Extended.config.h`)

New `PID_AUTOTUNE_*` macros next to the existing `PID_*` block in `Pid.h:12-26`:

| Macro | Default | Notes |
|---|---|---|
| `PID_AUTOTUNE_TEST_DISTANCE` | (degrees, per-install) | must be ≥ 20 × settle band; validated at start |
| `PID_AUTOTUNE_VALIDATION_SLEW_RATE` | `AXIS1_SLEW_RATE_DESIRED` | tune at the production slew rate (`Dome.cpp:112`) so gains are validated under real conditions |
| `PID_AUTOTUNE_PRELOAD_DISTANCE` | `AUTO` (= max(2 × backlash, 10 × settle band) in measure units) | unmeasured same-direction nudge before every measured move to zero the lash in the test direction (§3 AT_PRELOAD) |
| `PID_AUTOTUNE_MAX_OVERSHOOT_COUNTS` | `AUTO` (= 2 × settle band) | absolute counts, not percent (see §3 AT_MONITOR) |
| `PID_AUTOTUNE_SETTLE_TOLERANCE_COUNTS` | `AUTO` (= max(1.5 × backlash steps, 2)) | lash-aware settle band |
| `PID_AUTOTUNE_SETTLE_CONFIRM_MS` | 1000 | no longer bounded by the 3 s window — slewing set is held selected during monitor |
| `PID_AUTOTUNE_MOVE_TIMEOUT_MS` | derived from distance/rate × margin | also catches non-terminating gotos (tolerance-0 hunting) |
| `PID_AUTOTUNE_MAX_ITERATIONS` | 6 | |
| `PID_AUTOTUNE_REPEATS` | 5 | samples per round |
| `PID_AUTOTUNE_GAIN_STEP_LIMIT_PERCENT` | 40 | per-iteration clamp |
| `PID_AUTOTUNE_SAFETY_BACKOFF_PERCENT` | 50 | mandatory gain reduction after a servo safety shutdown |
| `PID_AUTOTUNE_MAX_BAND_CROSSINGS` | 4 | hunting/limit-cycle classifier |
| `PID_AUTOTUNE_OUTLIER_MODZ_THRESHOLD` | 3.5 | standard Iglewicz–Hoaglin value |
| `PID_AUTOTUNE_SPEED_TEST` | `ON` | measure the physical maximum rotation rate at the start of each run (§3c) |
| `PID_AUTOTUNE_SPEED_TEST_DISTANCE` | `AUTO` (= 3 × test distance) | speed test move length |
| `PID_AUTOTUNE_SPEED_TEST_RATE` | `AUTO` (= 2 × test slew rate) | deliberately unattainable commanded rate |
| `PID_AUTOTUNE_SPEED_SATURATION_PERCENT` | 90 | drive output % at/above which the drive counts as saturated |
| `PID_AUTOTUNE_SPEED_HEADROOM_PERCENT` | 85 | recommended operating max as % of measured; also clamps the tuning rate |

Plus, independent of the tuner (§3b): `AXIS1_TARGET_TOLERANCE` / `AXIS2_TARGET_TOLERANCE` (degrees, default 0.0 — preserves current behavior exactly) wired into the `Axis` constructors in `Dome.axis.cpp`.

Gate the whole feature behind a new `SERVO_PID_AUTOTUNE ON/OFF` flag (default `OFF`), following the existing `ABSOLUTE_ENCODER_CALIBRATION` opt-in pattern, so constrained AVR builds don't carry the extra state unless enabled. All auto-tune state (sample arrays, counters) lives under this flag.

## Files touched

- `src/lib/axis/motor/servo/feedback/Pid/Pid.h`, `Pid.cpp` — `validateParameters()` bounds only
- `src/lib/axis/Axis.h`, `Axis.cpp` — auto-tune state machine, per-round repeat/aggregate/outlier-reject logic, backlash-aware motion pattern, safety-shutdown handling, slewing-set hold during monitor; uses existing `autoGoto()`/`getTargetDistance()`/`motor->enable()`/`motor->setParameters()`/`motor->setSlewing()`/`motor->getBacklashSteps()` — no new lower-level motor/PID API required
- `src/lib/axis/Axis.command.cpp` — `:SXT`/`:GXT` commands
- `src/observatory/dome/Dome.command.cpp` — wire `axis1.command()`/`axis2.command()` fallback **and** the park/interlock gate on the `:SXT…,1` start form (no Roof changes — Roof owns no Axis in OCS)
- `src/observatory/dome/Dome.axis.cpp` — pass `AXIS1_TARGET_TOLERANCE`/`AXIS2_TARGET_TOLERANCE` to the `Axis` constructors
- `src/Config.defaults.h`, `Extended.config.h` — new `PID_AUTOTUNE_*` defaults, `AXIS…_TARGET_TOLERANCE`, and `SERVO_PID_AUTOTUNE` feature flag

## Verification

1. Compile for both a RAM-constrained target (e.g. Mega2560) and a larger target (ESP32/STM32) with `SERVO_PID_AUTOTUNE` `ON` and `OFF`, confirming no flash/RAM regression when off.
2. On real (or bench-simulated via `VirtualEnc`) hardware: `:SXT1,1#`, poll `:GXT1#` until done; confirm each measured move is a single ordinary bounded slew, that with wrap ON the moves walk unidirectionally around the travel (no mid-round reversals), and that no sustained oscillation/direction-chatter ever occurs.
3. Unit-check the aggregation math in isolation (host-side or a temporary debug command): synthetic round with one extreme value → modified-Z filter excludes it and the aggregate matches the median of the remaining samples; `MAD == 0` (all-identical) falls back sanely; hunting flag forces oscillatory outcome.
4. **Safety gating:** `:SXT1,1#` while parked → refused with the parked status code; with `DOME_SHUTTER_LOCK` active and roof closed → refused; `:SXT1,0#` (abort) and `:GXT1#` (status) still work in both states.
5. **Safety shutdown path:** provoke a servo safety shutdown mid-test (bench: force the oscillation detector with deliberately absurd gains, or disconnect the encoder for the stall detector) and verify the round is discarded, gains are backed off by `PID_AUTOTUNE_SAFETY_BACKOFF_PERCENT` before any re-enable, and a second shutdown aborts the run restoring pre-tune gains.
6. **Backlash behavior:** on the high-lash axis, confirm every measured move is preceded by a same-direction preload nudge (visible in the debug stream / physically as a small pre-move), including immediately after every direction change; confirm settle detection completes (band ≥ lash); confirm repeat-to-repeat spread within a round is visibly tighter with preload than with it disabled (temporarily set `PID_AUTOTUNE_PRELOAD_DISTANCE` to 0 to compare — this is the consistency the preload exists to buy); and confirm deliberately-high Ki produces a hunting classification rather than a long settle time.
7. **Measurement window:** with verbose debug on, confirm the "tracking selected" transition never fires during AT_MONITOR (slewing set held), and does fire normally after the run ends.
8. With `AXIS1_POWER_DOWN ON`, run a full tune and confirm the power-down logic never strands a move (the pre-move enable toggle re-arms the motor).
9. Confirm failure paths: force a fault mid-test (e.g. disconnect encoder) and verify auto-tune aborts and restores original gains rather than leaving the axis in a half-tuned state; power-cycle mid-tune and confirm boot gains come from NV (nothing persisted).
10. Confirm convergence behavior on a few different starting gain sets (deliberately too-soft and too-aggressive `Config.h` values) to sanity-check the correction rule converges within `PID_AUTOTUNE_MAX_ITERATIONS` and doesn't overcorrect/oscillate the gains themselves between iterations.
11. `:SXT1,2#` to persist, then `:GXA1#` (and a power-cycle) to confirm the new gains are read back correctly from NV.
12. Regression: with `SERVO_PID_AUTOTUNE` `OFF` (default), confirm `:Dz`/`:DS#` goto and manual `:SXA`/`:GXA` editing behave exactly as before, including with `AXIS1_TARGET_TOLERANCE` left at its 0.0 default.
13. **Speed measurement:** confirm the run opens with one visibly faster move; `:GXT1#` afterwards reports a plausible max rate with `saturated = 1` (cross-check by timing a full rotation at the reported rate); with `PID_AUTOTUNE_SPEED_TEST_RATE` set *below* the physical max, confirm `saturated = 0` is reported; deliberately set `AXIS1_SLEW_RATE_DESIRED` above the physical max and confirm the tuning moves run at the clamped rate (debug log) rather than saturating; confirm normal `:DS#` gotos after the run still respect the production `maxFreq`/slew rate (ceiling restored), including after an abort mid-speed-test.
