// -----------------------------------------------------------------------------------
// Axis servo PID auto-tune state
//
// Kept out of Axis.h deliberately: the run needs a lot of bookkeeping, and none of it is
// of any interest to the rest of the Axis class or to anything that includes it.  Axis
// carries one AxisAutoTuneState member; the logic lives in Axis.autotune.cpp
#pragma once

#include "../../Common.h"

#ifdef SERVO_PID_AUTOTUNE_PRESENT

// PID auto-tune state machine states (AT_BACKLASH is appended so the codes :GXT already
// reports keep their numbering; it runs first regardless of its position here)
enum AutoTuneState: uint8_t {AT_IDLE, AT_SPEED, AT_PRELOAD, AT_MOVE, AT_MONITOR, AT_AGGREGATE, AT_ANALYZE, AT_DONE_SUCCESS, AT_DONE_FAIL, AT_BACKLASH};

// PID auto-tune result/refusal codes (reported by :GXT)
enum AutoTuneResult: uint8_t {ATR_NONE, ATR_CONVERGED, ATR_BEST_EFFORT, ATR_ABORTED, ATR_TIMEOUT, ATR_FAULT, ATR_SAFETY_SHUTDOWN, ATR_MOTION_ERROR, ATR_BACKLASH_FAIL};

struct AxisAutoTuneState {
  AutoTuneState state = AT_IDLE;
  AutoTuneResult result = ATR_NONE;
  uint16_t step = 0;                    // commanded moves this run, for progress reporting
  uint16_t stepTotal = 0;               // estimated moves in a full run
  uint8_t phase = 0;                    // sub-phase within a state
  uint8_t repeat = 0;                   // measured moves completed this round
  uint8_t shutdownCount = 0;            // servo safety shutdowns this run (stalls, unattributed)
  uint8_t infeasibleCount = 0;          // probes discarded for tripping a detector (own budget)
  uint8_t huntingCount = 0;             // hunting-flagged repeats this round
  int8_t direction = 1;                 // direction of the current test move
  bool holdSlewing = false;             // holding motor->setSlewing(true) during monitor
  bool stagedValid = false;             // autoTuneStaged[] holds a result awaiting :SXT,2

  long settleBandSteps = 2;             // positioning dead band in steps (counts), from the lash
  long wobbleSteps = 2;                 // stability threshold in steps (counts), from the dither.
                                                // deliberately independent of the settle band: the band sizes
                                                // the test geometry, this only decides "has it stopped moving"
  long maxOvershootSteps = 4;           // overshoot acceptance in steps (counts)
  float testDistance = 0.0F;            // measured move distance in measures
  float preloadDistance = 0.0F;         // backlash take-up nudge distance in measures
  float rate = 0.0F;                    // test slew rate in measures/s, always concrete: autoGoto()
                                                // ignores a NAN rate and would silently inherit whatever
                                                // slewFreq a previous leg left behind
  float productionRate = 0.0F;          // slewFreq on entry, restored on every exit path
  unsigned long moveTimeout = 0;        // per-move watchdog period in milliseconds

  // which geometry values came from AUTO and so may be derived from the measured backlash
  bool bandAuto = false;
  bool overshootAuto = false;
  bool preloadAuto = false;
  bool timeoutAuto = false;

  // mechanical backlash measurement
  uint8_t backlashProbe = 0;            // reversal probes attempted
  uint8_t backlashSamples = 0;          // probes that produced a usable reading, and the write
                                                // index into the sample arrays.  a probe whose noise
                                                // floor makes the measurement untrustworthy is dropped
                                                // rather than recorded, so the two counts diverge
  long backlashSteps = 0;               // aggregated measured backlash in steps (counts)
  long backlashStore = 0;               // axis backlash compensation before the probe
  bool backlashOverride = false;        // compensation zeroed (restore on any exit)
  bool backlashDetected = false;        // this probe has seen the load break away
  bool backlashValid = false;           // a measurement completed this run
  float probeDistance = 0.0F;           // probe leg distance in measures
  float probeRate = 0.0F;               // probe slew rate in measures/s
  long probeEncStart = 0;               // encoder position at the reversal instant
  long probeCmdStart = 0;               // commanded position at the reversal instant
  long backlashCeiling = 0;             // per-probe sanity ceiling in steps

  // closed-loop hold dither, sampled while stopped between the probe legs.  the settle test can
  // only ever pass if the band is wider than the noise the loop holds position to, and an
  // encoder mounted motor-side of the gearbox reports no lash at all - this floors the band
  long ditherMin = 0;
  long ditherMax = 0;
  long ditherStartPos = 0;              // position at the start of the sampling window; net drift
                                                // against the peak-to-peak tells settled from still-moving
  uint8_t quietRetry = 0;               // dither windows discarded for drift, this probe
  uint8_t probeRetry = 0;               // safety-trip retries of the current probe
  long ditherSteps = 0;                 // aggregated peak-to-peak hold dither in steps

  // physical maximum rotation rate measurement
  float speedDistance = 0.0F;           // speed test move distance in measures
  float speedRate = 0.0F;               // commanded (over-speed) rate in measures/s
  float maxFreqStore = 0.0F;            // maxFreq before the speed test ceiling lift
  bool maxFreqOverride = false;         // ceiling lift active (restore on any exit)
  bool speedSaturated = false;          // drive output saturated during the speed test
  // the speed test runs once per direction: an axis has to slew both ways, so the usable ceiling
  // is the slower of the two, and the difference exposes a drive that is stronger one way
  uint8_t speedPass = 0;                // 0 = first direction, 1 = second
  float speedPassRate[2] = {0, 0};      // measured maximum per pass, in measures/s
  bool speedPassSat[2] = {false, false};// whether the drive saturated on each pass
  int8_t speedPassDir[2] = {0, 0};      // direction each pass ran in
  float measuredMaxRate = 0.0F;         // measured maximum rate in measures/s (deg/s)
  float peakRate = 0.0F;                // peak smoothed rate regardless of saturation
  float velocityEstimate = 0.0F;        // smoothed velocity estimate in measures/s
  long lastPositionSteps = 0;           // last sampled position for velocity estimation
  unsigned long lastSampleTime = 0;     // last velocity sample time in milliseconds

  // per-repeat measurement working state
  long peakOvershoot = 0;               // peak excursion beyond target in steps
  long stableDistance = 0;              // target distance when stability tracking last reset
  long probeDetectSteps = 0;            // this probe's breakaway threshold, floored by its dither
  float moveDuration = 0.0F;            // travel time of the last measured move, reported only
  unsigned long moveStartTime = 0;      // measured move start (backlash excluded) in milliseconds
  unsigned long watchdogTime = 0;       // watchdog deadline in milliseconds
  unsigned long stableStartTime = 0;    // position stable since, in milliseconds
  unsigned long dwellTime = 0;          // post-move dwell deadline in milliseconds
  unsigned long arriveTime = 0;         // goto completion time in milliseconds (0 = still slewing)
  bool measuring = false;               // true once out of backlash take-up
  bool inBand = false;                  // currently within the settle band
  bool everInBand = false;              // has entered the settle band
  bool hunting = false;                 // this repeat classified as hunting
  uint8_t bandCrossings = 0;            // settle band boundary crossings

  // round sample arrays
  float overshoot[PID_AUTOTUNE_REPEATS];
  float settleTime[PID_AUTOTUNE_REPEATS];
  float residual[PID_AUTOTUNE_REPEATS];
  float backlashSample[PID_AUTOTUNE_BACKLASH_PROBES];
  float ditherSample[PID_AUTOTUNE_BACKLASH_PROBES];

  // round aggregates (also reported by :GXT)
  float overshootResult = 0.0F;
  float settleTimeResult = 0.0F;
  float residualResult = 0.0F;

  // gain sets: original (restore,) candidate (under test,) incumbent (best so far,) staged (result)
  float original[3] = {0, 0, 0};
  float candidate[3] = {0, 0, 0};
  float staged[3] = {0, 0, 0};

  // compass search state.  the incumbent is the centre of the current poll and always holds the
  // best set measured so far; every probe is a step away from it that is either accepted (it
  // becomes the new incumbent) or discarded
  float incumbent[3] = {0, 0, 0};
  float incumbentScore = 0.0F;
  bool incumbentValid = false;          // false until the opening evaluation completes

  // the incumbent's own aggregates, kept because the live ones belong to whichever probe ran
  // last - usually a rejected one.  the poll ordering and the final verdict must both reason
  // about the set being kept, not about the last thing measured
  float incumbentOvershoot = 0.0F;
  float incumbentSettle = 0.0F;
  float incumbentResidual = 0.0F;
  bool incumbentHunting = false;

  float startScore = 0.0F;              // the opening score, reported at the end
  float stepFraction = 0.0F;            // current probe step as a fraction of each gain
  uint8_t repeatsThisEval = 0;          // measured moves this evaluation (coarse vs fine)
  uint16_t evalCount = 0;               // evaluations consumed, against MAX_EVALUATIONS
  int8_t pollOrder[6] = {0, 0, 0, 0, 0, 0}; // probe order, encoded parameter*2 + (0 up, 1 down)
  uint8_t pollCount = 0;                // valid entries in autoTunePollOrder[]
  uint8_t pollIndex = 0;                // probe being evaluated
  int8_t lastMove = -1;                 // last accepted probe, retried first from the new point
  bool kpUpBlocked = false;             // a Kp+ probe from this incumbent hunted or tripped safety
};

#endif
