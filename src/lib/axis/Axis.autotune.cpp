// -----------------------------------------------------------------------------------
// Axis servo PID auto-tune (closed-loop bump-test with iterative correction)
//
// Tunes only the slewing/goto gain set (param4-6.)  Each round commands several
// ordinary bounded autoGoto() moves, each preceded by an unmeasured same-direction
// preload nudge that takes up the mechanical backlash, measures overshoot (counts,)
// settle time and residual error per move, robustly aggregates the round (median +
// modified Z-score outlier filter) and applies a bounded classical correction.
// Results are staged in RAM only; :SXT[n],2# persists them to NV.

#include "Axis.h"

#ifdef SERVO_PID_AUTOTUNE_PRESENT

#include "../nv/Nv.h"

// start the PID auto-tune of the slewing gain set (param4-6)
CommandError Axis::autoTuneStart(float testDistance) {
  if (autoTuneActive()) return CE_SLEW_IN_MOTION;
  if (motor->driverType != SERVO) return CE_CMD_UNKNOWN;
  // the measure/correct model requires pure param4-6 during slews (auto scaled parameter mode)
  if (!((ServoMotor*)motor)->feedbackAutoScales()) return CE_PARAM_FORM;
  if (autoRate != AR_NONE) return CE_SLEW_IN_MOTION;
  if (motorFault()) return CE_SLEW_ERR_HARDWARE_FAULT;

  long backlash = motor->getBacklashSteps();

  // the production slew rate, captured before any leg can disturb it.  autoGoto() only calls
  // setFrequencySlew() for a non-NAN rate, so slewFreq is sticky: a slow backlash probe leg would
  // otherwise become the rate for the rest of the run, and outlive it
  at.productionRate = slewFreq;

  // the AUTO forms of the geometry below are all functions of the mechanical lash, and the
  // configured compensation is often 0 (in OCS the dome value lives in NV with no setter,) which
  // would collapse the settle band onto its 2 count floor.  AT_BACKLASH measures the lash for
  // real at the start of the run and autoTuneDeriveGeometry() re-derives whatever is AUTO from it
  #if PID_AUTOTUNE_BACKLASH_TEST == ON
    const bool measuringBacklash = true;
  #else
    const bool measuringBacklash = false;
  #endif

  // settle band in counts; the load can't be positioned finer than the lash dead band
  long band = (long)(PID_AUTOTUNE_SETTLE_TOLERANCE_COUNTS);
  at.bandAuto = band <= 0;
  if (at.bandAuto) { band = lroundf(backlash*1.5F); if (band < 2) band = 2; }
  at.settleBandSteps = band;

  long maxOvershoot = (long)(PID_AUTOTUNE_MAX_OVERSHOOT_COUNTS);
  at.overshootAuto = maxOvershoot <= 0;
  if (at.overshootAuto) maxOvershoot = band*2;
  at.maxOvershootSteps = maxOvershoot;

  // measured move distance must have dynamic range against lash + encoder quantization.  when the
  // band is still going to be measured this can't be judged yet, so defer it: the derivation raises
  // the distance to suit the measured band rather than failing the run
  float distance = testDistance;
  if (isnan(distance) || distance <= 0.0F) distance = PID_AUTOTUNE_TEST_DISTANCE;
  if (!(measuringBacklash && at.bandAuto) &&
      distance*settings.stepsPerMeasure < (float)(PID_AUTOTUNE_MIN_DISTANCE_BANDS)*band) return CE_PARAM_RANGE;
  at.testDistance = distance;

  // provisional stability threshold.  the dither measurement replaces this with a real figure; with
  // the backlash test off there is nothing to measure, so keep the old band-derived fallback
  at.wobbleSteps = band/4;
  if (at.wobbleSteps < 2) at.wobbleSteps = 2;

  // preload nudge distance.  sized from the lash it exists to take up, never from the settle band
  float preload = (float)(PID_AUTOTUNE_PRELOAD_DISTANCE);
  at.preloadAuto = preload <= 0.0F;
  if (at.preloadAuto) {
    long preloadSteps = backlash*2;
    if (preloadSteps < at.wobbleSteps*4) preloadSteps = at.wobbleSteps*4;
    preload = preloadSteps/settings.stepsPerMeasure;
  }
  at.preloadDistance = preload;

  // test slew rate, always resolved to a concrete value.  a NAN rate makes autoGoto() skip
  // setFrequencySlew() entirely and inherit whatever the previous leg left in slewFreq
  float rate = (float)(PID_AUTOTUNE_VALIDATION_SLEW_RATE);
  if (rate <= 0.0F) rate = at.productionRate;
  if (rate <= 0.0F) rate = 1.0F;
  at.rate = rate;

  // per-move watchdog, also catches a goto that never terminates (exact-count hunting)
  long timeoutMs = (long)(PID_AUTOTUNE_MOVE_TIMEOUT_MS);
  at.timeoutAuto = timeoutMs <= 0;
  if (at.timeoutAuto) timeoutMs = lroundf(((distance + preload)/rate)*4000.0F) + 8000;
  at.moveTimeout = timeoutMs;

  // backlash probe geometry.  a slow probe keeps the following-error term inside the measurement
  // small, since following error scales with commanded velocity
  at.probeDistance = (float)(PID_AUTOTUNE_BACKLASH_PROBE_DISTANCE);
  if (at.probeDistance <= 0.0F) {
    at.probeDistance = distance/5.0F;
    if (at.probeDistance < 1.0F) at.probeDistance = 1.0F;
  }
  at.probeRate = (float)(PID_AUTOTUNE_BACKLASH_PROBE_RATE);
  if (at.probeRate <= 0.0F) at.probeRate = rate*0.25F;
  if (at.probeRate <= 0.0F) at.probeRate = 0.25F;
  at.backlashCeiling = (long)(PID_AUTOTUNE_BACKLASH_MAX_COUNTS);
  if (at.backlashCeiling <= 0) at.backlashCeiling = lroundf((at.probeDistance*settings.stepsPerMeasure)/2.0F);
  at.backlashProbe = 0;
  at.backlashSamples = 0;
  at.backlashSteps = 0;
  at.ditherSteps = 0;
  at.probeRetry = 0;
  at.backlashValid = false;
  at.backlashDetected = false;

  // capture the pre-tune slewing gain set (restored at the end of every run)
  at.original[0] = settings.param4; at.original[1] = settings.param5; at.original[2] = settings.param6;
  for (int i = 0; i < 3; i++) { at.candidate[i] = at.original[i]; at.incumbent[i] = at.original[i]; }
  at.stagedValid = false;
  at.repeat = 0; at.shutdownCount = 0; at.infeasibleCount = 0; at.huntingCount = 0;
  at.overshootResult = 0.0F; at.settleTimeResult = 0.0F; at.residualResult = 0.0F;

  // compass search: the run opens by measuring the Config.h gains themselves, which establishes the
  // incumbent every later probe is judged against
  at.incumbentValid = false;
  at.incumbentScore = 0.0F;
  at.incumbentOvershoot = 0.0F; at.incumbentSettle = 0.0F; at.incumbentResidual = 0.0F;
  at.incumbentHunting = false;
  at.startScore = 0.0F;
  at.stepFraction = PID_AUTOTUNE_STEP_INITIAL_PERCENT/100.0F;
  at.repeatsThisEval = at.stepFraction >= PID_AUTOTUNE_COARSE_STEP_PERCENT/100.0F ?
                            PID_AUTOTUNE_COARSE_REPEATS : PID_AUTOTUNE_REPEATS;
  at.evalCount = 0;
  at.pollCount = 0; at.pollIndex = 0;
  at.lastMove = -1;
  at.kpUpBlocked = false;

  // worst-case move count for the progress line.  a safety shutdown discards and repeats an
  // evaluation, so the real count can exceed this - hence "of ~n" rather than a hard denominator
  at.step = 0;
  at.stepTotal = PID_AUTOTUNE_MAX_EVALUATIONS*PID_AUTOTUNE_REPEATS*2;
  #if PID_AUTOTUNE_BACKLASH_TEST == ON
    at.stepTotal += PID_AUTOTUNE_BACKLASH_PROBES*2;                   // load-up + reversal leg
  #endif
  #if PID_AUTOTUNE_SPEED_TEST == ON
    at.stepTotal += 4;                          // preload + speed move, once per direction
  #endif

  // walk direction: with bounded travel start toward the far limit
  at.direction = 1;
  if (!wrapEnabled) {
    double coordinate = getInstrumentCoordinate();
    if (coordinate - settings.limits.min > settings.limits.max - coordinate) at.direction = -1;
  }

  if (!enabled) enable(true);

  // echo the settings that shape a run, so a pasted log answers "what was this tuned against?"
  // on its own.  the servo driver already prints its acceleration and power range at boot
  VF("MSG:"); V(axisPrefix); VF("auto-tune config: target tolerance "); V(targetTolerance);
  V(unitsStr); VF(", slew rate "); V(at.productionRate); V(unitsStr);
  VF("/s, backlash comp "); V(getBacklash()); V(unitsStr);
  VF(", power down "); V(powerDownStandstill ? "ON" : "OFF");
  VF(", gains P="); V(settings.param4); VF(" I="); V(settings.param5); VF(" D="); VL(settings.param6);

  VF("MSG:"); V(axisPrefix); VF("auto-tune start, band "); V(at.settleBandSteps);
  VF(" counts, test "); V(at.testDistance); VF(", preload "); V(at.preloadDistance);
  #if PID_AUTOTUNE_BACKLASH_TEST == ON
    VLF(" (provisional, pending the backlash measurement)");
  #else
    VLF("");
  #endif

  at.result = ATR_NONE;
  at.phase = 0;

  #if PID_AUTOTUNE_SPEED_TEST == ON
    at.speedDistance = (float)(PID_AUTOTUNE_SPEED_TEST_DISTANCE);
    if (at.speedDistance <= 0.0F) at.speedDistance = at.testDistance*3.0F;
    at.speedRate = (float)(PID_AUTOTUNE_SPEED_TEST_RATE);
    if (at.speedRate <= 0.0F) at.speedRate = at.rate*4.0F;
  #endif
  at.measuredMaxRate = 0.0F;
  at.peakRate = 0.0F;
  at.speedSaturated = false;
  at.speedPass = 0;
  at.speedPassRate[0] = at.speedPassRate[1] = 0.0F;
  at.speedPassSat[0] = at.speedPassSat[1] = false;

  // order of the opening measurements:
  //   AT_BACKLASH - sizes the settle band and preload, which everything downstream depends on
  //   AT_SPEED    - reports the hardware capability and keeps the tuning moves achievable
  #if PID_AUTOTUNE_BACKLASH_TEST == ON
    at.state = AT_BACKLASH;
  #elif PID_AUTOTUNE_SPEED_TEST == ON
    at.state = AT_SPEED;
  #else
    at.state = AT_PRELOAD;
  #endif
  return CE_NONE;
}

// append a probe direction to a poll being built, ignoring one already in it.  the symptom ordering
// and the pattern move can easily name the same direction, and a duplicate is not merely a wasted
// evaluation: the list is bounded at six, so a repeat pushes a real direction off the end and the
// poll silently stops being exhaustive - which is exactly what the step-shrink decision relies on
static void autoTunePollAppend(int8_t *list, uint8_t &count, int8_t move) {
  if (count >= 6) return;
  for (uint8_t i = 0; i < count; i++) if (list[i] == move) return;
  list[count++] = move;
}

// name of an encoded compass probe (parameter*2 + direction), for the log
static const char *autoTuneMoveName(int8_t move) {
  switch (move) {
    case 0: return "Kp+";
    case 1: return "Kp-";
    case 2: return "Ki+";
    case 3: return "Ki-";
    case 4: return "Kd+";
    case 5: return "Kd-";
  }
  return "start";
}

// log one line per commanded move, tagged with the step and round/probe position.  a full run makes
// dozens of short moves and without this the debug stream gives no sense of where it has got to
void Axis::autoTuneProgress(const char *what) {
  at.step++;

  VF("MSG:"); V(axisPrefix); VF("auto-tune step "); V(at.step);
  VF(" of ~"); V(at.stepTotal); VF(" [");
  switch (at.state) {
    case AT_BACKLASH:
      VF("backlash probe "); V(at.backlashProbe + 1); VF("/"); V(PID_AUTOTUNE_BACKLASH_PROBES);
    break;
    case AT_SPEED:
      VF("speed test "); V(at.speedPass + 1); VF("/2");
    break;
    default:
      VF("eval "); V(at.evalCount + 1); VF("/"); V(PID_AUTOTUNE_MAX_EVALUATIONS);
      VF(" "); V(at.incumbentValid ? autoTuneMoveName(at.pollOrder[at.pollIndex]) : "start");
      VF(", move "); V(at.repeat + 1); VF("/"); V(at.repeatsThisEval);
      // the gains this move is actually running, on every line.  a run makes hundreds of these and
      // the probe name alone only says which direction was stepped, not what is under the wheels -
      // so a line pulled out of the middle of a log is otherwise not self-describing
      VF(", P="); V(at.candidate[0]); VF(" I="); V(at.candidate[1]);
      VF(" D="); V(at.candidate[2]);
    break;
  }
  VF("] "); VL(what);
}

// advance out of the opening measurement phases into the first tuning round
void Axis::autoTuneBeginTuning() {
  // nothing the opening measurement phases do may influence the gains the tuning rounds start from
  for (int i = 0; i < 3; i++) at.candidate[i] = at.original[i];
  at.phase = 0;
  #if PID_AUTOTUNE_SPEED_TEST == ON
    at.state = AT_SPEED;
  #else
    at.state = AT_PRELOAD;
  #endif
}

// restore the axis backlash compensation if the backlash probe zeroed it
void Axis::autoTuneRestoreBacklash() {
  if (at.backlashOverride) {
    motor->setBacklashSteps(at.backlashStore);
    at.backlashOverride = false;
  }
}

// size the settle band, overshoot limit, preload and test distance from the measured backlash.
// returns false if the resulting geometry cannot fit the axis travel
bool Axis::autoTuneDeriveGeometry() {
  long lash = at.backlashSteps;

  // stability threshold: how still the axis has to be to count as stopped.  this comes from the hold
  // dither alone and is deliberately NOT folded into the settle band - the band sizes the test
  // geometry, so letting the noise floor drive it inflates the test distance and preload with it
  at.wobbleSteps = at.ditherSteps*(long)(PID_AUTOTUNE_DITHER_WOBBLE_FACTOR);
  if (at.wobbleSteps < 2) at.wobbleSteps = 2;

  if (at.bandAuto) {
    // positioning dead band: the load cannot be held finer than the lash, so a narrower band
    // guarantees hunting and meaningless settle times
    long band = lroundf(lash*1.5F);
    // ...nor finer than the wobble.  a band below the stability threshold asks the axis to hold a
    // position tighter than the settle test can even resolve, which no drive can satisfy
    if (band < at.wobbleSteps) band = at.wobbleSteps;
    if (band < 2) band = 2;
    at.settleBandSteps = band;
  } else
  if (at.settleBandSteps < at.wobbleSteps) {
    // explicit configuration wins, but it cannot be met
    DF("WRN:"); D(axisPrefix); DF("auto-tune configured settle band "); D(at.settleBandSteps);
    DF(" is below the measured wobble "); D(at.wobbleSteps); DLF(" counts, residual can never pass");
  }
  if (at.overshootAuto) {
    // floor the overshoot limit on the dither too, so a zero-lash axis is not handed a limit
    // narrower than the noise it holds position to
    long fromBand = at.settleBandSteps*2;
    long fromDither = at.ditherSteps*4;
    at.maxOvershootSteps = fromBand > fromDither ? fromBand : fromDither;
    if (at.maxOvershootSteps < 4) at.maxOvershootSteps = 4;
  }
  if (at.preloadAuto) {
    // sized from the lash it exists to take up.  the old band-derived floor was a proxy for an
    // unknown lash and, once the band grows, demands absurd nudges
    long preloadSteps = lash*2;
    if (preloadSteps < at.wobbleSteps*4) preloadSteps = at.wobbleSteps*4;
    at.preloadDistance = preloadSteps/settings.stepsPerMeasure;
  }

  // the measured move still needs dynamic range against the now-known dead band; grow it
  // rather than failing the run, which is the whole point of measuring first
  float minDistance = ((float)(PID_AUTOTUNE_MIN_DISTANCE_BANDS)*at.settleBandSteps)/settings.stepsPerMeasure;
  if (at.testDistance < minDistance) {
    float capped = minDistance;
    if (capped > (float)(PID_AUTOTUNE_MAX_TEST_DISTANCE)) capped = (float)(PID_AUTOTUNE_MAX_TEST_DISTANCE);
    VF("MSG:"); V(axisPrefix); VF("auto-tune test distance raised from "); V(at.testDistance);
    VF(" to "); V(capped); VLF(" for the measured dead band");
    if (capped < minDistance) {
      DF("WRN:"); D(axisPrefix); DF("auto-tune wanted "); D(minDistance);
      DF(" but PID_AUTOTUNE_MAX_TEST_DISTANCE caps it at "); D(capped);
      DLF("; overshoot resolution will be reduced");
    }
    at.testDistance = capped;
  }

  // bounded travel has to hold a preload, a measured move and its overshoot headroom
  if (!wrapEnabled) {
    double span = settings.limits.max - settings.limits.min;
    if (at.preloadDistance + at.testDistance*1.25F > span) {
      DF("ERR:"); D(axisPrefix); DF("auto-tune measured backlash "); D(lash);
      DLF(" counts needs more travel than the axis limits allow");
      return false;
    }
  }

  // the speed test move tracks the test distance when it was left at AUTO
  #if PID_AUTOTUNE_SPEED_TEST == ON
    if ((float)(PID_AUTOTUNE_SPEED_TEST_DISTANCE) <= 0.0F) at.speedDistance = at.testDistance*3.0F;
  #endif

  if (at.timeoutAuto) {
    at.moveTimeout = lroundf(((at.testDistance + at.preloadDistance)/at.rate)*4000.0F) + 8000;
  }

  VF("MSG:"); V(axisPrefix); VF("auto-tune geometry: band "); V(at.settleBandSteps);
  VF(" counts, wobble "); V(at.wobbleSteps); VF(" counts, max overshoot ");
  V(at.maxOvershootSteps); VF(" counts, test "); V(at.testDistance);
  VF(", preload "); V(at.preloadDistance);
  VF(", move timeout "); V((long)at.moveTimeout); VLF("ms");

  // rough worst case, so the cost of the run is known before the moves start.  billed at the full
  // repeat count throughout; the coarse regime uses fewer, so a real run comes in under this
  float perPair = (at.testDistance + at.preloadDistance)/at.rate +
                  1.0F + (PID_AUTOTUNE_SETTLE_CONFIRM_MS/1000.0F);
  VF("MSG:"); V(axisPrefix); VF("auto-tune estimated run time up to ");
  V(lroundf((perPair*PID_AUTOTUNE_MAX_EVALUATIONS*PID_AUTOTUNE_REPEATS)/60.0F)); VLF(" minutes");
  return true;
}

// abort a running PID auto-tune, restoring the pre-tune gains
void Axis::autoTuneAbort() {
  if (autoTuneActive()) autoTuneFinish(ATR_ABORTED); else at.state = AT_IDLE;
}

// apply the staged auto-tune result live and persist it to NV (same path as :SXA)
CommandError Axis::autoTuneApply() {
  if (autoTuneActive()) return CE_SLEW_IN_MOTION;
  if (!at.stagedValid) return CE_0;

  settings.param4 = at.staged[0];
  settings.param5 = at.staged[1];
  settings.param6 = at.staged[2];
  nv.updateBytes(NV_AXIS_SETTINGS_BASE + (axisNumber - 1)*AxisStoredSettingsSize, &settings, sizeof(AxisStoredSettings));
  motor->setParameters(settings.param1, settings.param2, settings.param3, settings.param4, settings.param5, settings.param6);
  if (motor->enabled) { motor->enable(false); motor->enable(true); }

  VF("MSG:"); V(axisPrefix); VLF("auto-tune gains applied and saved to NV");
  return CE_NONE;
}

// apply the candidate gains: setParameters() then an enable toggle so Pid::reset()
// reloads param4-6 into SetTunings() and clears any accumulated windup
void Axis::autoTuneSetCandidate() {
  motor->setParameters(settings.param1, settings.param2, settings.param3, at.candidate[0], at.candidate[1], at.candidate[2]);
  motor->enable(false);
  motor->enable(true);
}

// end the run: stop motion, release the slewing-set hold, restore pre-tune gains
void Axis::autoTuneFinish(AutoTuneResult result) {
  if (autoRate != AR_NONE) autoSlewAbort();
  if (at.holdSlewing) { motor->setSlewing(false); at.holdSlewing = false; }

  // restore the frequency ceiling if the speed test lifted it
  if (at.maxFreqOverride) {
    maxFreq = at.maxFreqStore;
    at.maxFreqOverride = false;
  }

  // hand the axis back at the rate it was running before the tune.  probe and test legs each set
  // slewFreq explicitly, and it is sticky, so without this a run leaves production gotos slowed down
  if (at.productionRate > 0.0F) setFrequencySlew(at.productionRate);

  // restore the backlash compensation if the backlash probe zeroed it
  autoTuneRestoreBacklash();

  // restore the pre-tune gains; never re-arm a motor a safety layer disabled
  motor->setParameters(settings.param1, settings.param2, settings.param3, at.original[0], at.original[1], at.original[2]);
  if (motor->enabled) { motor->enable(false); motor->enable(true); }

  at.result = result;

  // an aborted run still has a result if the search got as far as measuring an incumbent.  the
  // incumbent is a complete, aggregated evaluation that behaved - it is the PROBES that trip
  // detectors, never the point the search is sitting on - so discarding it throws away every
  // evaluation the run paid for on account of something that happened to a candidate already
  // rejected.  only ATR_ABORTED discards, because there the operator asked for the axis back
  if (result == ATR_ABORTED || !at.incumbentValid) at.stagedValid = false;

  if (at.stagedValid) {
    // the gain margin, applied only when it is earned.  the search returns a local minimum of the
    // score, which is usually interior - surrounded by measured, well behaved neighbours - and
    // derating that would just move the result off the point the run spent its evaluations finding.
    // but when the Kp+ neighbour hunted or tripped a detector the optimum is sitting against a cliff,
    // and a cliff degrades sharply rather than gracefully, so back Kp off before staging.  Kp only:
    // derating Kd would raise overshoot and derating Ki would slow out the residual error
    if ((float)(PID_AUTOTUNE_GAIN_MARGIN_PERCENT) > 0.0F) {
      if (at.kpUpBlocked) {
        float boundary = at.staged[0];
        at.staged[0] *= 1.0F - (float)(PID_AUTOTUNE_GAIN_MARGIN_PERCENT)/100.0F;
        VF("MSG:"); V(axisPrefix); VF("auto-tune optimum sits against a Kp boundary, applying ");
        V((long)(PID_AUTOTUNE_GAIN_MARGIN_PERCENT)); VF("% gain margin: P=");
        V(at.staged[0]); VF(" (was "); V(boundary); VLF(")");
      } else {
        VF("MSG:"); V(axisPrefix); VLF("auto-tune optimum is interior (the Kp+ neighbour was measured, not a cliff) - staged as found");
      }
    }

    VF("MSG:"); V(axisPrefix); VF("auto-tune done P="); V(at.staged[0]);
    VF(" I="); V(at.staged[1]); VF(" D="); V(at.staged[2]);
    VF(" score "); V(at.incumbentScore); VF(" (start "); V(at.startScore); VF(")");
    if (result == ATR_CONVERGED) { VLF(" (converged)"); } else
    if (result == ATR_BEST_EFFORT) { VLF(" (did not fully converge)"); } else {
      // say plainly that this is a partial result, and why the run stopped, so it is never mistaken
      // for a search that finished on its own terms
      VF(" (run ended early, code "); V(result); VLF(" - best measured set, search incomplete)");
    }
  }

  if (result == ATR_CONVERGED || result == ATR_BEST_EFFORT) {
    at.state = AT_DONE_SUCCESS;
  } else {
    at.state = AT_DONE_FAIL;
    VF("MSG:"); V(axisPrefix); VF("auto-tune ended, code "); VL(result);
  }
}

// handle a servo safety shutdown (stall/runaway/oscillation) or driver fault
// mid-run; returns true if one occurred and was handled
bool Axis::autoTuneSafetyEvent() {
  bool shutdown = ((ServoMotor*)motor)->inSafetyShutdown();
  ServoSafetyCause cause = ((ServoMotor*)motor)->safetyCause();
  bool fault = motorFault();
  if ((!motor->enabled && !poweredDown) || fault) {
    VF("MSG:"); V(axisPrefix); VF("auto-tune servo safety shutdown/fault (");
    switch (cause) {
      case SSC_STALL:       VF("stall"); break;
      case SSC_RUNAWAY:     VF("runaway"); break;
      case SSC_OSCILLATION: VF("oscillation"); break;
      default:              VF("driver fault"); break;
    }
    VLF(")");
    if (at.holdSlewing) { motor->setSlewing(false); at.holdSlewing = false; }

    // restore the frequency ceiling if the speed test had lifted it
    if (at.maxFreqOverride) {
      maxFreq = at.maxFreqStore;
      setFrequencySlew(at.rate);
      at.maxFreqOverride = false;
    }

    // getDriverStatus() reports a latched safety shutdown as .fault too, so only a fault WITHOUT
    // the safety latch is hardware - a broken encoder or a driver error, neither retryable
    if (fault && !shutdown) { autoTuneFinish(ATR_FAULT); return true; }

    // a STALL is not a gain boundary in either direction: the axis did not move at all, and more gain
    // would have pushed harder, not less.  it means something physically stuck, so repeat the round
    // unchanged rather than concluding the sweep is over or backing the gains off
    if (cause == SSC_STALL) {
      at.shutdownCount++;
      VF("MSG:"); V(axisPrefix); VF("auto-tune stall is not a gain limit - repeating at P=");
      V(at.candidate[0]); VF(" (shutdown "); V(at.shutdownCount);
      VF(" of "); V(PID_AUTOTUNE_MAX_SHUTDOWNS); VLF(")");
      if (at.shutdownCount >= PID_AUTOTUNE_MAX_SHUTDOWNS) { autoTuneFinish(ATR_SAFETY_SHUTDOWN); return true; }

      at.repeat = 0; at.huntingCount = 0;
      at.phase = 0;
      if (at.state != AT_BACKLASH && at.state != AT_SPEED) at.state = AT_PRELOAD;
      autoTuneSetCandidate();
      return true;
    }

    // an oscillation or runaway on a probe, with an incumbent to fall back to, is information rather
    // than a failure: this direction is infeasible.  score it as such and carry on polling the others.
    // the search is meant to push each parameter until it stops helping, so finding a hard edge is an
    // expected outcome - ending the run here would let one bad Kd probe throw away every Ki direction
    // still untried, which is what the old refinement-boundary branch did
    if (at.incumbentValid && at.state != AT_BACKLASH && at.state != AT_SPEED) {
      // counted against its OWN budget, not the run's shutdown allowance.  this is the search
      // establishing where the feasible region ends, which is a normal and expected outcome
      at.infeasibleCount++;
      int8_t move = at.pollOrder[at.pollIndex];
      VF("MSG:"); V(axisPrefix); VF("auto-tune probe "); V(autoTuneMoveName(move));
      VF(" is infeasible (P="); V(at.candidate[0]); VF(" I="); V(at.candidate[1]);
      VF(" D="); V(at.candidate[2]); VF("), infeasible probe "); V(at.infeasibleCount);
      VF(" of "); V(PID_AUTOTUNE_MAX_INFEASIBLE_PROBES); VLF("");
      if (at.infeasibleCount >= PID_AUTOTUNE_MAX_INFEASIBLE_PROBES) { autoTuneFinish(ATR_SAFETY_SHUTDOWN); return true; }

      // a Kp+ that trips is exactly the cliff the gain margin exists for
      if (move == 0) at.kpUpBlocked = true;

      // fall back to the incumbent before anything else runs, so a discarded probe can never leak
      // its gains into the next evaluation
      for (int i = 0; i < 3; i++) at.candidate[i] = at.incumbent[i];
      autoTuneSetCandidate();

      // step past this probe.  without this the poll would re-issue the direction that just tripped,
      // and the run would spend its whole shutdown budget rediscovering the same edge
      at.pollIndex++;
      autoTuneAdvancePoll();
      return true;
    }

    #if PID_AUTOTUNE_SPEED_TEST == ON
      // the speed test deliberately drives the output into saturation; a trip here is the drive
      // reporting it has nothing left, which is what the test set out to establish.  take the peak
      // seen so far as the physical maximum and move on rather than failing the run
      if (at.state == AT_SPEED) {
        if (at.peakRate > 0.0F) {
          at.measuredMaxRate = at.peakRate;
          at.speedSaturated = true;
        }
        VF("MSG:"); V(axisPrefix); VF("auto-tune speed test ended by a safety trip, taking ");
        V(at.measuredMaxRate); VLF(" as the physical maximum");

        // apply the same rate clamp the normal completion path would have
        float recommended = at.measuredMaxRate*(PID_AUTOTUNE_SPEED_HEADROOM_PERCENT/100.0F);
        if (recommended > 0.0F && at.rate > recommended) {
          at.rate = recommended;
          VF("MSG:"); V(axisPrefix); VF("auto-tune test rate clamped to "); V(at.rate);
          VLF(" (configured slew rate exceeds the measured maximum)");
        }

        at.repeat = 0; at.huntingCount = 0;
        at.phase = 0;
        at.state = AT_PRELOAD;
        autoTuneSetCandidate();
        return true;
      }
    #endif

    if (at.state == AT_BACKLASH) {
      // a lashy drive stick-slips across the dead band at the slow probe rate and swings the output
      // hard both ways.  it is intermittent - other probes in the same run complete cleanly - so the
      // remedy is simply to retry this probe on the same terms.  changing the terms is worse than
      // useless here: lowering the gains loosens the position hold and triples the measured dither,
      // and raising the rate replaces lash with following error.  completed samples are kept
      bool remedied = false;
      if (at.probeRetry < PID_AUTOTUNE_BACKLASH_PROBE_RETRIES) {
        at.probeRetry++;
        VF("MSG:"); V(axisPrefix); VF("auto-tune retrying backlash probe ");
        V(at.backlashProbe + 1); VF("/"); V(PID_AUTOTUNE_BACKLASH_PROBES);
        VF(" (attempt "); V(at.probeRetry + 1); VLF(", drive oscillates intermittently)");
        remedied = true;
      } else
      if (at.probeRate < at.rate*0.5F) {
        // retrying is not getting through; give the rate one bounded nudge, capped at half the test
        // rate so the measurement keeps some validity, and start the probes over at the new rate
        float raised = at.probeRate*2.0F;
        if (raised > at.rate*0.5F) raised = at.rate*0.5F;
        VF("MSG:"); V(axisPrefix); VF("auto-tune backlash probe rate raised to "); V(raised);
        VLF("/s (retries alone did not get through)");
        at.probeRate = raised;
        at.probeRetry = 0;
        at.backlashProbe = 0;
        at.backlashSamples = 0;   // samples taken at the old rate are not comparable
        remedied = true;
      }

      if (remedied) {
        // a targeted remedy does not spend the run's shutdown budget
        at.backlashDetected = false;
        at.repeat = 0; at.huntingCount = 0;
        at.phase = 0;
        autoTuneSetCandidate();
        return true;
      }
      VF("MSG:"); V(axisPrefix); VLF("auto-tune backlash probe out of remedies");
      at.backlashProbe = 0;
      at.backlashDetected = false;
    }

    at.shutdownCount++;
    VF("MSG:"); V(axisPrefix); VF("auto-tune safety shutdown "); V(at.shutdownCount);
    VF(" of "); V(PID_AUTOTUNE_MAX_SHUTDOWNS); VLF(" tolerated this run");
    if (at.shutdownCount >= PID_AUTOTUNE_MAX_SHUTDOWNS) { autoTuneFinish(ATR_SAFETY_SHUTDOWN); return true; }

    if (at.state != AT_BACKLASH) {
      // a correction round is where the gains really are the thing under test, so back them off
      for (int i = 0; i < 3; i++) at.candidate[i] *= PID_AUTOTUNE_SAFETY_BACKOFF_PERCENT/100.0F;
      VF("MSG:"); V(axisPrefix); VF("auto-tune gains backed off to P="); V(at.candidate[0]);
      VF(" I="); V(at.candidate[1]); VF(" D="); VL(at.candidate[2]);
      at.state = AT_PRELOAD;
    }

    // discard the phase in progress and restart it from its own beginning
    at.repeat = 0; at.huntingCount = 0;
    at.phase = 0;

    // re-arm.  the servo safety layer left the motor disabled, and without clearing that here the
    // next poll sees it still disabled, counts a second shutdown and aborts - so the backoff and
    // retry above could never actually run
    autoTuneSetCandidate();
    return true;
  }
  return false;
}

// robust aggregate of a round's samples: median + modified Z-score (Iglewicz-Hoaglin)
// outlier filter, mean of the survivors; falls back to the median when MAD is zero
float Axis::autoTuneAggregate(float values[], int count) {
  // with no samples every index below is out of range; nothing measured aggregates to nothing
  if (count <= 0) return 0.0F;
  if (count > PID_AUTOTUNE_SAMPLE_MAX) count = PID_AUTOTUNE_SAMPLE_MAX;

  float sorted[PID_AUTOTUNE_SAMPLE_MAX] = {0};
  for (int i = 0; i < count; i++) {
    float v = values[i];
    int j = i;
    while (j > 0 && sorted[j - 1] > v) { sorted[j] = sorted[j - 1]; j--; }
    sorted[j] = v;
  }
  float median = (count % 2) ? sorted[count/2] : (sorted[count/2 - 1] + sorted[count/2])/2.0F;

  float deviations[PID_AUTOTUNE_SAMPLE_MAX] = {0};
  for (int i = 0; i < count; i++) {
    float v = fabsf(values[i] - median);
    int j = i;
    while (j > 0 && deviations[j - 1] > v) { deviations[j] = deviations[j - 1]; j--; }
    deviations[j] = v;
  }
  float mad = (count % 2) ? deviations[count/2] : (deviations[count/2 - 1] + deviations[count/2])/2.0F;

  if (mad < 0.000001F) return median;

  float sum = 0.0F;
  int survivors = 0;
  for (int i = 0; i < count; i++) {
    float modifiedZ = 0.6745F*(values[i] - median)/mad;
    if (fabsf(modifiedZ) <= PID_AUTOTUNE_OUTLIER_MODZ_THRESHOLD) { sum += values[i]; survivors++; }
  }
  if (survivors == 0) return median;
  return sum/survivors;
}

// weighted score of the evaluation just aggregated; lower is better.  each term is a measured value
// over the threshold at which it becomes unacceptable, so the three are commensurate and a score of
// 3.0 is roughly "at every limit at once".  it stays meaningful well below the acceptance thresholds,
// which is what lets the search keep improving a set that already passes
float Axis::autoTuneScore() {
  // hunting counts against a candidate only when it REPRODUCES.  a lashy drive stick-slips
  // intermittently, and one hunting repeat among several is an outlier - the three measurements
  // already treat it as exactly that, via the modified Z-score filter, and come out clean.  a bare
  // "any repeat hunted" test bypasses all of that and vetoes the candidate on the one bad move.
  // measured: the same gain set scored 10.50 with one hunting repeat of three and 0.32 with none,
  // in the same run, and a probe whose other two moves both beat the incumbent was thrown away.
  // so the penalty needs a majority, and scales with how much of the evaluation actually hunted
  float huntFraction = at.repeatsThisEval > 0 ?
                       (float)at.huntingCount/(float)at.repeatsThisEval : 0.0F;
  float huntPenalty = huntFraction > 0.5F ? (float)(PID_AUTOTUNE_HUNTING_PENALTY)*huntFraction : 0.0F;

  return (float)(PID_AUTOTUNE_WEIGHT_OVERSHOOT)*at.overshootResult/(float)at.maxOvershootSteps +
         (float)(PID_AUTOTUNE_WEIGHT_SETTLE)*at.settleTimeResult/(float)PID_AUTOTUNE_MAX_SETTLE_MS +
         (float)(PID_AUTOTUNE_WEIGHT_RESIDUAL)*at.residualResult/(float)at.settleBandSteps +
         huntPenalty;
}

// order the probe directions to try from the current incumbent, best-first.  the ordering is the old
// classical correction rule, repurposed: it no longer decides how far to move - the step schedule does
// that - but it still knows which way is worth trying first, which is most of the value.  a good order
// means the search usually accepts its first or second probe instead of paying for all six
void Axis::autoTuneBuildPoll() {
  // encoded as parameter*2 + direction, 0 = up, 1 = down
  const int8_t KP_UP = 0, KP_DN = 1, KI_UP = 2, KI_DN = 3, KD_UP = 4, KD_DN = 5;

  int8_t preferred[6];
  uint8_t n = 0;

  // the symptoms are the INCUMBENT's, never the live aggregates - by the time a poll is rebuilt those
  // usually describe a probe that was just rejected, and ordering the next search around a set the
  // run has already discarded would send it off in the wrong direction
  if (at.incumbentHunting) {
    // integrator plus backlash limit cycle: take the integrator out first
    autoTunePollAppend(preferred, n, KI_DN); autoTunePollAppend(preferred, n, KP_DN); autoTunePollAppend(preferred, n, KD_UP);
  } else
  if (at.incumbentOvershoot > (float)at.maxOvershootSteps) {
    // under-damped: more damping, or less drive
    autoTunePollAppend(preferred, n, KD_UP); autoTunePollAppend(preferred, n, KP_DN); autoTunePollAppend(preferred, n, KI_DN);
  } else
  if (at.incumbentSettle > (float)PID_AUTOTUNE_MAX_SETTLE_MS) {
    // sluggish: more drive, or less damping holding it back
    autoTunePollAppend(preferred, n, KP_UP); autoTunePollAppend(preferred, n, KD_DN); autoTunePollAppend(preferred, n, KI_UP);
  } else
  if (at.incumbentResidual > (float)at.settleBandSteps) {
    // frozen following error outside the lash band: that is the integrator's job
    autoTunePollAppend(preferred, n, KI_UP); autoTunePollAppend(preferred, n, KP_UP); autoTunePollAppend(preferred, n, KD_DN);
  } else {
    // nothing is over its threshold, so there is no symptom to chase.  retry whatever last worked -
    // the classical pattern move, and cheap when a direction keeps paying off - then push upward
    if (at.lastMove >= 0) autoTunePollAppend(preferred, n, at.lastMove);
    autoTunePollAppend(preferred, n, KP_UP); autoTunePollAppend(preferred, n, KD_UP); autoTunePollAppend(preferred, n, KI_UP);
  }

  // every remaining direction still gets tried, just later.  the poll has to be exhaustive or the
  // step would shrink while an improving direction sat untested
  const int8_t rest[6] = {KP_UP, KP_DN, KD_UP, KD_DN, KI_UP, KI_DN};
  for (uint8_t r = 0; r < 6; r++) autoTunePollAppend(preferred, n, rest[r]);

  // drop the parameters this build is not tuning; with both off the search is Kp only
  at.pollCount = 0;
  for (uint8_t i = 0; i < n; i++) {
    #if PID_AUTOTUNE_TUNE_KI != ON
      if (preferred[i]/2 == 1) continue;
    #endif
    #if PID_AUTOTUNE_TUNE_KD != ON
      if (preferred[i]/2 == 2) continue;
    #endif
    at.pollOrder[at.pollCount++] = preferred[i];
  }
  at.pollIndex = 0;
}

// load at.candidate[] from probe at.pollIndex of the current poll.  false means the probe
// is not worth an evaluation - it landed back on the incumbent, or outside what the feedback accepts
bool Axis::autoTuneApplyProbe() {
  if (at.pollIndex >= at.pollCount) return false;

  int8_t move = at.pollOrder[at.pollIndex];
  uint8_t param = move/2;
  bool up = (move % 2) == 0;

  // the step is a fraction of the gain, floored by an absolute minimum.  without that floor a gain
  // that reached zero could never move again, and both Ki and Kd are legitimately zero
  const float minStep[3] = {(float)(PID_AUTOTUNE_MIN_STEP_KP), (float)(PID_AUTOTUNE_MIN_STEP_KI), (float)(PID_AUTOTUNE_MIN_STEP_KD)};
  float value = at.incumbent[param];
  float step = value*at.stepFraction;
  if (step < minStep[param]) step = minStep[param];

  float probed = up ? value + step : value - step;
  if (probed < 0.0F) probed = 0.0F;

  // Kp at zero is not a controller, it is a disabled axis
  if (param == 0 && probed < minStep[0]) return false;

  // clamped back onto the incumbent: measuring it again would cost an evaluation to learn nothing
  if (fabsf(probed - value) < 0.000001F) return false;

  for (int i = 0; i < 3; i++) at.candidate[i] = at.incumbent[i];
  at.candidate[param] = probed;

  return motor->validateParameters(settings.param1, settings.param2, settings.param3,
                                   at.candidate[0], at.candidate[1], at.candidate[2]);
}

// issue the next probe: the next direction in this poll, or a finer step, or the end of the run
void Axis::autoTuneAdvancePoll() {
  while (true) {
    // out of directions at this step size - the incumbent beat all of its neighbours, so it is a
    // local optimum at this resolution.  halve the step and look again, closer in
    if (at.pollIndex >= at.pollCount) {
      float next = at.stepFraction*(PID_AUTOTUNE_STEP_SHRINK_PERCENT/100.0F);
      if (next < PID_AUTOTUNE_STEP_MIN_PERCENT/100.0F) {
        VF("MSG:"); V(axisPrefix); VF("auto-tune converged at step ");
        V(lroundf(at.stepFraction*100.0F)); VF("% after "); V(at.evalCount);
        VF(" evaluations, score "); V(at.incumbentScore);
        VF(" (start "); V(at.startScore); VLF(")");

        // converged is about the search; whether the answer is good enough is a separate question,
        // and a run that optimized honestly into a drive that still cannot meet the thresholds
        // should say so rather than claim success
        bool acceptable = !at.incumbentHunting &&
                          at.incumbentOvershoot <= (float)at.maxOvershootSteps &&
                          at.incumbentSettle <= (float)PID_AUTOTUNE_MAX_SETTLE_MS &&
                          at.incumbentResidual <= (float)at.settleBandSteps;
        autoTuneFinish(acceptable ? ATR_CONVERGED : ATR_BEST_EFFORT);
        return;
      }

      at.stepFraction = next;
      at.repeatsThisEval = at.stepFraction >= PID_AUTOTUNE_COARSE_STEP_PERCENT/100.0F ?
                                PID_AUTOTUNE_COARSE_REPEATS : PID_AUTOTUNE_REPEATS;
      VF("MSG:"); V(axisPrefix); VF("auto-tune no probe improved, refining step to ");
      V(lroundf(at.stepFraction*100.0F)); VF("% ("); V(at.repeatsThisEval);
      VLF(" moves per evaluation)");
      autoTuneBuildPoll();
      continue;
    }

    // the evaluation budget is what bounds the run time; the incumbent is already staged
    if (at.evalCount >= PID_AUTOTUNE_MAX_EVALUATIONS) {
      VF("MSG:"); V(axisPrefix); VF("auto-tune evaluation budget spent at step ");
      V(lroundf(at.stepFraction*100.0F)); VF("%, score "); V(at.incumbentScore);
      VF(" (start "); V(at.startScore); VLF(")");
      autoTuneFinish(ATR_BEST_EFFORT);
      return;
    }

    if (!autoTuneApplyProbe()) { at.pollIndex++; continue; }

    VF("MSG:"); V(axisPrefix); VF("auto-tune probe ");
    V(autoTuneMoveName(at.pollOrder[at.pollIndex]));
    VF(" at step "); V(lroundf(at.stepFraction*100.0F)); VF("%: P=");
    V(at.candidate[0]); VF(" I="); V(at.candidate[1]);
    VF(" D="); VL(at.candidate[2]);

    // alternate the walk direction between evaluations (one reversal each, absorbed by the next
    // preload) so both directions of travel get covered
    if (wrapEnabled) at.direction = -at.direction;

    at.repeat = 0; at.huntingCount = 0;
    at.phase = 0;
    autoTuneSetCandidate();
    at.state = AT_PRELOAD;
    return;
  }
}

// advance the auto-tune state machine, called from poll() at FRACTIONAL_SEC Hz
void Axis::autoTunePoll() {
  switch (at.state) {

    #if PID_AUTOTUNE_BACKLASH_TEST == ON
    // measure the mechanical lash by reversing and watching how far the commanded trajectory has to
    // travel before the load breaks away: motorSteps is the trajectory and advances at the commanded
    // rate independently of the feedback, so while the drive crosses the dead band the gap between
    // it and the encoder grows by exactly the lash, and the gap at breakaway is the measurement.
    // each probe also samples how tightly the loop holds position while stopped, which floors the
    // settle band independently - a load-side encoder sees the lash, a motor-side one does not, but
    // both see the hold noise that the settle test has to be able to accommodate
    case AT_BACKLASH: {
      if (autoTuneSafetyEvent()) return;

      if (at.phase == 0) {
        // configured backlash compensation would insert its own take-up and mask the measurement
        if (!at.backlashOverride) {
          at.backlashStore = motor->getBacklashSteps();
          motor->setBacklashSteps(0);
          at.backlashOverride = true;
          VF("MSG:"); V(axisPrefix); VF("auto-tune measuring backlash, ");
          V(PID_AUTOTUNE_BACKLASH_PROBES); VF(" probes of "); V(at.probeDistance);
          VF(" at "); V(at.probeRate); VLF("/s");
        }

        // each probe is a load-up leg plus a reversal leg, so keep two legs of travel in hand
        if (!wrapEnabled) {
          double coordinate = getInstrumentCoordinate();
          double need = at.probeDistance*2.25;
          if (at.direction > 0 && coordinate + need > settings.limits.max) at.direction = -1; else
          if (at.direction < 0 && coordinate - need < settings.limits.min) at.direction = 1;
        }

        // load-up leg: take the lash up hard in the current direction
        autoTuneSetCandidate();
        at.quietRetry = 0;
        autoTuneProgress(at.direction > 0 ? "load-up leg, forward" : "load-up leg, reverse");
        double target = getInstrumentCoordinate() + at.direction*at.probeDistance;
        if (wrapEnabled) target = wrap(target);
        setTargetCoordinate(target);
        if (autoGoto(at.probeRate) != CE_NONE) { autoTuneFinish(ATR_MOTION_ERROR); return; }
        at.watchdogTime = millis() + (unsigned long)(lroundf((at.probeDistance/at.probeRate)*4000.0F)) + 8000UL;
        at.phase = 1;
      } else

      if (at.phase == 1) {
        if (autoRate != AR_NONE) {
          if ((long)(millis() - at.watchdogTime) > 0) { autoSlewAbort(); autoTuneFinish(ATR_TIMEOUT); }
          return;
        }
        // hold the slewing gain set for the rest of the probe.  the quiet + dither windows outlast
        // SERVO_SLEWING_TO_TRACKING_DELAY, so without this the dither is measured under param1-3
        // while the wobble derived from it gates a settle test that runs under param4-6
        motor->setSlewing(true); at.holdSlewing = true;

        // the load-up leg has stopped, but the axis is still coasting past target.  wait that out
        // before looking at the hold noise - sampling from here would measure the overshoot instead
        at.dwellTime = millis() + PID_AUTOTUNE_BACKLASH_QUIET_MS;
        at.phase = 2;
      } else

      if (at.phase == 2) {
        if ((long)(millis() - at.dwellTime) <= 0) return;
        // minimum settle delay elapsed: open the first dither sampling window
        at.ditherStartPos = motor->getInstrumentCoordinateSteps();
        at.ditherMin = at.ditherMax = at.ditherStartPos;
        at.dwellTime = millis() + PID_AUTOTUNE_BACKLASH_DITHER_MS;
        at.phase = 3;
      } else

      if (at.phase == 3) {
        long held = motor->getInstrumentCoordinateSteps();
        if (held < at.ditherMin) at.ditherMin = held;
        if (held > at.ditherMax) at.ditherMax = held;
        if ((long)(millis() - at.dwellTime) <= 0) return;

        long dither = at.ditherMax - at.ditherMin;

        // breakaway must be unambiguous against the noise this axis holds position to, or the
        // detector trips on wander while the trajectory has barely moved and reports no lash
        long detect = (long)(PID_AUTOTUNE_BACKLASH_DETECT_COUNTS);
        if (detect < dither*2 + 2) detect = dither*2 + 2;

        // ...but the encoder must still be able to reach it inside one probe leg.  a noisy axis can
        // demand a threshold approaching the leg itself, and lowering it to fit does not rescue the
        // probe - it biases it: an earlier trigger means a smaller "moved" and so a larger
        // "commanded - moved", reading high.  such a probe is dropped, not capped
        long detectMax = lroundf((at.probeDistance*settings.stepsPerMeasure)/4.0F);
        if (detectMax < 4) detectMax = 4;
        bool tooNoisy = detect > detectMax;

        // is the pre-reversal creep big enough to matter?  compare it to the threshold the encoder
        // must cross for breakaway, not to the peak-to-peak: for monotonic motion pk-pk IS the
        // drift, so a drift-vs-pk-pk test is satisfied by any creep at all and never passes
        long drift = labs(held - at.ditherStartPos);
        bool stillMoving = drift > 2 && drift > detect/2;

        // both complaints are usually transient, so re-sample before giving up on the probe
        if ((stillMoving || tooNoisy) && at.quietRetry < PID_AUTOTUNE_BACKLASH_QUIET_RETRIES) {
          at.quietRetry++;
          at.ditherStartPos = held;
          at.ditherMin = at.ditherMax = held;
          at.dwellTime = millis() + PID_AUTOTUNE_BACKLASH_DITHER_MS;
          return;
        }

        if (tooNoisy) {
          DF("WRN:"); D(axisPrefix); DF("auto-tune probe "); D(at.backlashProbe + 1);
          DF(" discarded: dither "); D(dither); DF(" counts needs a breakaway threshold of ");
          D(detect); DF(" over a leg of only ");
          D(lroundf(at.probeDistance*settings.stepsPerMeasure));
          DLF(" counts - raise PID_AUTOTUNE_BACKLASH_PROBE_DISTANCE if this is common");

          // skip the reversal leg entirely and move on; a missing sample beats a biased one
          at.backlashProbe++;
          at.quietRetry = 0;
          at.dwellTime = millis() + 500;
          at.phase = (at.backlashProbe >= PID_AUTOTUNE_BACKLASH_PROBES) ? 5 : 6;
          return;
        }

        if (stillMoving) {
          DF("WRN:"); D(axisPrefix); DF("auto-tune probe "); D(at.backlashProbe + 1);
          DF(" still drifting "); D(drift); DF(" counts after "); D(at.quietRetry);
          DLF(" retries, backlash reading may be low");
        }

        at.ditherSample[at.backlashSamples] = (float)dither;
        at.probeDetectSteps = detect;

        // snapshot both positions at the reversal instant.  the constant offset between them (the
        // frozen following error left by the load-up leg) cancels because both are read as deltas
        at.probeEncStart = motor->getInstrumentCoordinateSteps();
        at.probeCmdStart = motor->getMotorPositionSteps();
        at.backlashDetected = false;

        // release the gain-set hold: the reversal goto selects the slewing set itself
        if (at.holdSlewing) { motor->setSlewing(false); at.holdSlewing = false; }

        at.direction = -at.direction;
        autoTuneProgress(at.direction > 0 ? "reversal leg, forward" : "reversal leg, reverse");
        double target = getInstrumentCoordinate() + at.direction*at.probeDistance;
        if (wrapEnabled) target = wrap(target);
        setTargetCoordinate(target);
        if (autoGoto(at.probeRate) != CE_NONE) { autoTuneFinish(ATR_MOTION_ERROR); return; }
        at.watchdogTime = millis() + (unsigned long)(lroundf((at.probeDistance/at.probeRate)*4000.0F)) + 8000UL;
        at.phase = 4;
      } else

      if (at.phase == 4) {
        if ((long)(millis() - at.watchdogTime) > 0) {
          autoSlewAbort();
          // a probe that never breaks away means the lash is larger than the probe leg
          if (!at.backlashDetected) {
            DF("ERR:"); D(axisPrefix); DF("auto-tune backlash exceeds the probe distance (");
            D(at.probeDistance); DLF("), raise PID_AUTOTUNE_BACKLASH_PROBE_DISTANCE");
            autoTuneFinish(ATR_BACKLASH_FAIL);
          } else autoTuneFinish(ATR_TIMEOUT);
          return;
        }

        if (!at.backlashDetected) {
          // signed: only motion in the reversal sense counts.  an unsigned test is satisfied by
          // hold dither wandering either way, which reports a lash of zero
          long moved = (motor->getInstrumentCoordinateSteps() - at.probeEncStart)*at.direction;
          if (moved >= at.probeDetectSteps) {
            long commanded = labs(motor->getMotorPositionSteps() - at.probeCmdStart);
            long lash = commanded - moved;
            if (lash < 0) lash = 0;
            if (lash > at.backlashCeiling) lash = at.backlashCeiling;
            at.backlashSample[at.backlashSamples] = (float)lash;
            at.backlashDetected = true;
            VF("MSG:"); V(axisPrefix); VF("auto-tune backlash probe "); V(at.backlashProbe + 1);
            VF("/"); V(PID_AUTOTUNE_BACKLASH_PROBES); VF(" = "); V(lash); VF(" counts (dither ");
            V((long)at.ditherSample[at.backlashSamples]); VF(", breakaway at ");
            V(at.probeDetectSteps); VF(", settle retries "); V(at.quietRetry); VLF(")");
          }
        }

        if (autoRate != AR_NONE) return;  // reversal leg still in progress

        if (!at.backlashDetected) {
          long moved = (motor->getInstrumentCoordinateSteps() - at.probeEncStart)*at.direction;
          DF("ERR:"); D(axisPrefix); DF("auto-tune backlash probe saw only "); D(moved);
          DF(" counts of reversal against a breakaway threshold of "); D(at.probeDetectSteps);
          DF(" over a "); D(at.probeDistance); DLF(" leg");
          autoTuneFinish(ATR_BACKLASH_FAIL);
          return;
        }

        at.backlashProbe++;
        at.backlashSamples++;  // this attempt produced a usable reading
        at.probeRetry = 0;     // each probe gets its own retry budget
        at.dwellTime = millis() + 500;
        at.phase = (at.backlashProbe >= PID_AUTOTUNE_BACKLASH_PROBES) ? 5 : 6;
      } else

      if (at.phase == 5) {
        if ((long)(millis() - at.dwellTime) <= 0) return;

        // every probe may have been discarded as too noisy to trust
        if (at.backlashSamples == 0) {
          DF("ERR:"); D(axisPrefix); DF("auto-tune discarded all ");
          D(PID_AUTOTUNE_BACKLASH_PROBES); DF(" backlash probes as too noisy to measure over a ");
          D(at.probeDistance); DLF(" leg - raise PID_AUTOTUNE_BACKLASH_PROBE_DISTANCE");
          autoTuneRestoreBacklash();
          autoTuneFinish(ATR_BACKLASH_FAIL);
          return;
        }

        at.backlashSteps = lroundf(autoTuneAggregate(at.backlashSample, at.backlashSamples));
        at.ditherSteps = lroundf(autoTuneAggregate(at.ditherSample, at.backlashSamples));
        at.backlashValid = true;
        autoTuneRestoreBacklash();

        // the spread says whether to trust the aggregate: a tight cluster is a real measurement,
        // a wide one is either noise or genuine position-dependent lash around the travel
        long spreadLow = lroundf(at.backlashSample[0]), spreadHigh = spreadLow;
        for (int i = 1; i < at.backlashSamples; i++) {
          long s = lroundf(at.backlashSample[i]);
          if (s < spreadLow) spreadLow = s;
          if (s > spreadHigh) spreadHigh = s;
        }

        VF("MSG:"); V(axisPrefix); VF("auto-tune measured backlash "); V(at.backlashSteps);
        VF(" counts ("); V(at.backlashSteps/settings.stepsPerMeasure); V(unitsStr);
        VF("), spread "); V(spreadLow); VF(".."); V(spreadHigh);
        VF(" from "); V(at.backlashSamples); VF("/"); V(PID_AUTOTUNE_BACKLASH_PROBES);
        VF(" probes, hold dither "); V(at.ditherSteps); VLF(" counts pk-pk");

        if (!autoTuneDeriveGeometry()) { autoTuneFinish(ATR_BACKLASH_FAIL); return; }
        autoTuneBeginTuning();
      } else {
        // between probes
        if ((long)(millis() - at.dwellTime) > 0) at.phase = 0;
      }
    } break;
    #endif

    #if PID_AUTOTUNE_SPEED_TEST == ON
    case AT_SPEED: {
      if (autoTuneSafetyEvent()) return;

      if (at.phase == 0) {
        // with bounded travel keep preload + speed move inside the limits
        if (!wrapEnabled) {
          double coordinate = getInstrumentCoordinate();
          double need = at.preloadDistance + at.speedDistance*1.25F;
          if (at.direction > 0 && coordinate + need > settings.limits.max) at.direction = -1; else
          if (at.direction < 0 && coordinate - need < settings.limits.min) at.direction = 1;
        }

        // fresh gains/integrator, then the usual unmeasured backlash take-up nudge
        autoTuneSetCandidate();
        autoTuneProgress(at.direction > 0 ? "preload nudge, forward" : "preload nudge, reverse");
        double target = getInstrumentCoordinate() + at.direction*at.preloadDistance;
        if (wrapEnabled) target = wrap(target);
        setTargetCoordinate(target);
        if (autoGoto(at.rate) != CE_NONE) { autoTuneFinish(ATR_MOTION_ERROR); return; }
        at.watchdogTime = millis() + at.moveTimeout;
        at.phase = 1;
      } else

      if (at.phase == 1) {
        if (autoRate != AR_NONE) {
          if ((long)(millis() - at.watchdogTime) > 0) { autoSlewAbort(); autoTuneFinish(ATR_TIMEOUT); }
          return;
        }
        at.dwellTime = millis() + 500;
        at.phase = 2;
      } else

      if (at.phase == 2) {
        if ((long)(millis() - at.dwellTime) <= 0) return;

        autoTuneProgress(at.direction > 0 ? "over-speed move, forward" : "over-speed move, reverse");

        // temporarily lift the axis frequency ceiling: setFrequencySlew()/setFrequency()
        // clamp to maxFreq, which the dome sets to the production slew rate, so an
        // over-speed command would otherwise be silently limited to production speed
        at.maxFreqStore = maxFreq;
        maxFreq = at.speedRate;
        at.maxFreqOverride = true;

        double target = getInstrumentCoordinate() + at.direction*at.speedDistance;
        if (wrapEnabled) target = wrap(target);
        setTargetCoordinate(target);
        if (autoGoto(at.speedRate) != CE_NONE) {
          maxFreq = at.maxFreqStore; setFrequencySlew(at.rate); at.maxFreqOverride = false;
          autoTuneFinish(ATR_MOTION_ERROR);
          return;
        }

        // watchdog sized from the production rate: the axis may be much slower than commanded
        float productionRate = at.maxFreqStore > 0.0F ? at.maxFreqStore : 1.0F;
        at.watchdogTime = millis() + (unsigned long)(lroundf((at.speedDistance/productionRate)*4000.0F)) + 8000UL;

        at.lastPositionSteps = motor->getInstrumentCoordinateSteps();
        at.lastSampleTime = millis();
        at.velocityEstimate = 0.0F;
        at.phase = 3;
      } else

      if (at.phase == 3) {
        if ((long)(millis() - at.watchdogTime) > 0) {
          autoSlewAbort();
          autoTuneFinish(ATR_TIMEOUT);  // finish restores the lifted ceiling
          return;
        }

        // sample encoder velocity ~50Hz, smooth with an IIR (tau ~200ms,) and record
        // the plateau: the physical maximum is the smoothed rate while the drive
        // output is saturated (it can push no harder)
        unsigned long now = millis();
        long dt = (long)(now - at.lastSampleTime);
        if (dt >= 20) {
          long position = motor->getInstrumentCoordinateSteps();
          float rate = fabsf((float)(position - at.lastPositionSteps))/settings.stepsPerMeasure/(dt/1000.0F);
          at.lastPositionSteps = position;
          at.lastSampleTime = now;
          at.velocityEstimate += (rate - at.velocityEstimate)*0.1F;

          // full output while the axis is SLOWING is the drive braking, not the drive running out of
          // capability, and the speed it happens to be doing at that moment is one momentum carried
          // it to - not one it can hold.  only count a sample while still gaining or holding speed
          bool speedingUp = at.velocityEstimate >= at.peakRate*0.98F;
          bool saturated = speedingUp &&
                           fabsf(((ServoMotor*)motor)->velocityPercent) >= PID_AUTOTUNE_SPEED_SATURATION_PERCENT;
          if (saturated) at.speedSaturated = true;
          if (at.velocityEstimate > at.peakRate) at.peakRate = at.velocityEstimate;
          if (saturated && at.velocityEstimate > at.measuredMaxRate) at.measuredMaxRate = at.velocityEstimate;
        }

        if (autoRate != AR_NONE) return;  // move still in progress

        // move done: without saturation the peak only shows how fast the move went
        // (the true maximum is higher than the commanded rate)
        if (!at.speedSaturated || at.measuredMaxRate <= 0.0F) at.measuredMaxRate = at.peakRate;

        maxFreq = at.maxFreqStore;
        setFrequencySlew(at.rate);
        at.maxFreqOverride = false;

        at.speedPassRate[at.speedPass] = at.measuredMaxRate;
        at.speedPassSat[at.speedPass] = at.speedSaturated;
        at.speedPassDir[at.speedPass] = at.direction;

        VF("MSG:"); V(axisPrefix); VF("auto-tune max rate ");
        V(at.direction > 0 ? "forward " : "reverse ");
        V(at.measuredMaxRate);
        if (at.speedSaturated) { VLF(" (drive saturated while accelerating: physical maximum)"); }
        else {
          VF(" (NOT saturated at a commanded "); V(at.speedRate);
          VLF(" - this is a lower bound, raise PID_AUTOTUNE_SPEED_TEST_RATE to find the real ceiling)");
        }

        // repeat in the other direction.  a drive can be much stronger one way than the other, and
        // the axis has to slew both, so a single-direction measurement can overstate the ceiling by
        // a wide margin - and every move the wrong way then runs saturated
        if (at.speedPass == 0) {
          at.speedPass = 1;
          at.direction = -at.direction;
          at.measuredMaxRate = 0.0F;
          at.peakRate = 0.0F;
          at.speedSaturated = false;
          at.dwellTime = millis() + 500;
          at.phase = 5;
          return;
        }

        // both directions measured: the usable ceiling is the slower one
        int slow = (at.speedPassRate[1] < at.speedPassRate[0]) ? 1 : 0;
        at.measuredMaxRate = at.speedPassRate[slow];
        at.speedSaturated = at.speedPassSat[slow];

        float fast = at.speedPassRate[1 - slow];
        VF("MSG:"); V(axisPrefix); VF("auto-tune max rate both ways: ");
        V(at.speedPassRate[0]); VF(" / "); V(at.speedPassRate[1]);
        VF(", usable ceiling "); V(at.measuredMaxRate); VLF(" (the slower direction)");

        // only compare the two when both are real ceilings; an unsaturated pass is a lower bound and
        // could read low simply because it was never pushed hard enough
        if (fast > 0.0F && at.measuredMaxRate < fast*0.8F &&
            at.speedPassSat[0] && at.speedPassSat[1]) {
          DF("WRN:"); D(axisPrefix); DF("auto-tune drive is asymmetric: one direction reaches only ");
          D(lroundf((at.measuredMaxRate/fast)*100.0F));
          DLF("% of the other. often a physical characteristic rather than a fault, but the slew rate");
          DF("WRN:"); D(axisPrefix); DLF("must come from the slower figure or every move that way saturates");
        }

        // keep the tuning moves physically achievable: clamp the test slew rate
        // to the recommended fraction of the measured maximum
        float recommended = at.measuredMaxRate*(PID_AUTOTUNE_SPEED_HEADROOM_PERCENT/100.0F);
        if (at.speedSaturated && recommended > 0.0F && at.rate > recommended) {
          at.rate = recommended;
          VF("MSG:"); V(axisPrefix); VF("auto-tune test rate clamped to "); V(at.rate);
          VLF(" (configured slew rate exceeds the measured maximum)");
        }

        at.dwellTime = millis() + 500;
        at.phase = 4;
      } else

      if (at.phase == 4) {
        if ((long)(millis() - at.dwellTime) > 0) { at.phase = 0; at.state = AT_PRELOAD; }
      } else {
        // between the two direction passes
        if ((long)(millis() - at.dwellTime) > 0) at.phase = 0;
      }
    } break;
    #endif

    case AT_PRELOAD: {
      if (autoTuneSafetyEvent()) return;

      if (at.phase == 0) {
        // with bounded travel keep preload + move + overshoot headroom inside the limits
        if (!wrapEnabled) {
          double coordinate = getInstrumentCoordinate();
          double need = at.preloadDistance + at.testDistance*1.25F;
          if (at.direction > 0 && coordinate + need > settings.limits.max) at.direction = -1; else
          if (at.direction < 0 && coordinate - need < settings.limits.min) at.direction = 1;
        }

        // fresh candidate gains and a clean integrator for this repeat
        autoTuneSetCandidate();

        // unmeasured same-direction nudge: enter the measured move with the lash
        // fully taken up in the test direction, whatever the last move left behind
        autoTuneProgress(at.direction > 0 ? "preload nudge, forward" : "preload nudge, reverse");
        double target = getInstrumentCoordinate() + at.direction*at.preloadDistance;
        if (wrapEnabled) target = wrap(target);
        setTargetCoordinate(target);
        if (autoGoto(at.rate) != CE_NONE) { autoTuneFinish(ATR_MOTION_ERROR); return; }
        at.watchdogTime = millis() + at.moveTimeout;
        at.phase = 1;
      } else

      if (at.phase == 1) {
        if (autoRate != AR_NONE) {
          if ((long)(millis() - at.watchdogTime) > 0) { autoSlewAbort(); autoTuneFinish(ATR_TIMEOUT); }
          return;
        }
        at.dwellTime = millis() + 500;
        at.phase = 2;
      } else {
        if ((long)(millis() - at.dwellTime) > 0) { at.phase = 0; at.state = AT_MOVE; }
      }
    } break;

    case AT_MOVE: {
      if (autoTuneSafetyEvent()) return;

      autoTuneProgress("measured move");
      double target = getInstrumentCoordinate() + at.direction*at.testDistance;
      if (wrapEnabled) target = wrap(target);
      setTargetCoordinate(target);
      if (autoGoto(at.rate) != CE_NONE) { autoTuneFinish(ATR_MOTION_ERROR); return; }

      at.peakOvershoot = 0;
      at.measuring = false;
      at.moveStartTime = millis();  // provisional, reset once clear of any residual backlash
      at.watchdogTime = millis() + at.moveTimeout;
      at.arriveTime = 0;
      at.inBand = false; at.everInBand = false; at.bandCrossings = 0; at.hunting = false;
      at.stableDistance = motor->getTargetDistanceSteps();
      at.stableStartTime = millis();
      at.state = AT_MONITOR;
    } break;

    case AT_MONITOR: {
      if (autoTuneSafetyEvent()) return;

      if ((long)(millis() - at.watchdogTime) > 0) {
        // BEFORE arrival the goto itself never terminated - the motion system is not doing what it
        // was told, and nothing about the gains explains that, so the run ends.  AFTER arrival the
        // goto completed and the axis simply never held still: that is a fact about the gain set
        // under test, and the most informative one there is.  ending the whole run on it throws away
        // every evaluation already paid for, over a single repeat that was telling us the answer
        if (at.arriveTime == 0) { autoSlewAbort(); autoTuneFinish(ATR_TIMEOUT); return; }

        VF("MSG:"); V(axisPrefix); VF("auto-tune repeat never settled inside the ");
        V((long)at.moveTimeout); VLF("ms watchdog - recording it as hunting");
        at.hunting = true;
      }

      // exclude residual backlash take-up from the timing
      if (!at.measuring && !motor->inBacklash) { at.measuring = true; at.moveStartTime = millis(); }

      long distanceSteps = motor->getTargetDistanceSteps();

      // peak excursion beyond the target in the direction of travel, in counts
      if (at.direction > 0 ? distanceSteps < 0 : distanceSteps > 0) {
        long overshoot = labs(distanceSteps);
        if (overshoot > at.peakOvershoot) at.peakOvershoot = overshoot;
      }

      if (autoRate != AR_NONE) return;  // trajectory still in progress

      if (at.arriveTime == 0) {
        at.arriveTime = millis();
        // a goto that ended far from the target was aborted (limit sense/motion error,) not completed
        if (labs(distanceSteps) > lroundf((at.testDistance*settings.stepsPerMeasure)/2.0F)) {
          autoTuneFinish(ATR_MOTION_ERROR);
          return;
        }
        // hold the slewing gain set selected so the whole measurement reflects the
        // set being tuned (otherwise it decays to the tracking set after 3s)
        motor->setSlewing(true); at.holdSlewing = true;
        at.stableDistance = distanceSteps; at.stableStartTime = millis();
      }

      // settle band crossings after first entry classify lash/integrator limit cycling
      bool nowInBand = labs(distanceSteps) <= at.settleBandSteps;
      if (nowInBand != at.inBand) {
        if (at.everInBand) at.bandCrossings++;
        if (nowInBand) at.everInBand = true;
        at.inBand = nowInBand;
      }
      if (at.bandCrossings > PID_AUTOTUNE_MAX_BAND_CROSSINGS) at.hunting = true;

      // settled = position stable for the confirm period, in or out of the band
      // (out-of-band stability is a real result: frozen following error, drives Ki).
      // the threshold is the measured hold dither, not a fraction of the settle band
      if (labs(distanceSteps - at.stableDistance) > at.wobbleSteps) {
        at.stableDistance = distanceSteps;
        at.stableStartTime = millis();
      }
      bool settled = (long)(millis() - at.stableStartTime) >= PID_AUTOTUNE_SETTLE_CONFIRM_MS;
      if (!settled && !at.hunting) return;

      // record this repeat.  the RAW standing error is kept, not a copy zeroed for being inside the
      // lash band: zeroing per sample throws away the only signal Ki has.  a band wide enough to
      // swallow every candidate's residual - which is what a high-lash drive produces - would leave
      // the residual term reading 0.00 for every gain set alike, and the search could not tell one
      // Ki from another.  acceptance still asks "is the residual within the band", it just asks it
      // of the aggregate, which means the same thing and keeps the gradient intact
      long residual = labs(distanceSteps);
      at.overshoot[at.repeat] = (float)at.peakOvershoot;

      // settle time is measured from ARRIVAL, not from the start of the move.  measured from move
      // start it is dominated by travel time, which scales with the test distance, so a fixed
      // PID_AUTOTUNE_MAX_SETTLE_MS could never be met and every round read as "sluggish"
      at.settleTime[at.repeat] = (float)((at.hunting ? millis() : at.stableStartTime) - at.arriveTime);
      at.moveDuration = (float)(at.arriveTime - at.moveStartTime);
      at.residual[at.repeat] = (float)residual;
      if (at.hunting) at.huntingCount++;

      motor->setSlewing(false); at.holdSlewing = false;

      VF("MSG:"); V(axisPrefix); VF("auto-tune   eval "); V(at.evalCount + 1);
      VF("/"); V(PID_AUTOTUNE_MAX_EVALUATIONS);
      VF(" "); V(at.incumbentValid ? autoTuneMoveName(at.pollOrder[at.pollIndex]) : "start");
      VF(", move "); V(at.repeat + 1);
      VF("/"); V(at.repeatsThisEval); VF(": overshoot "); V(at.peakOvershoot);
      VF("/"); V(at.maxOvershootSteps); VF(" counts, settle ");
      V(at.settleTime[at.repeat]); VF("/"); V((long)PID_AUTOTUNE_MAX_SETTLE_MS);
      VF(" ms, residual "); V(residual); VF("/");
      V(at.settleBandSteps); VF(" counts, travel ");
      V(lroundf(at.moveDuration)); VF("ms");
      if (at.hunting) { VLF(", HUNTING"); } else { VLF(""); }

      at.repeat++;
      at.phase = 0;
      if (at.repeat >= at.repeatsThisEval) at.state = AT_AGGREGATE; else at.state = AT_PRELOAD;
    } break;

    case AT_AGGREGATE: {
      at.overshootResult = autoTuneAggregate(at.overshoot, at.repeatsThisEval);
      at.settleTimeResult = autoTuneAggregate(at.settleTime, at.repeatsThisEval);
      at.residualResult = autoTuneAggregate(at.residual, at.repeatsThisEval);
      at.state = AT_ANALYZE;
    } break;

    case AT_ANALYZE: {
      // "this gain set hunts", on the same majority rule autoTuneScore() applies - an isolated
      // hunting repeat is drive noise, not a property of the gains
      bool hunting = at.huntingCount*2 > at.repeatsThisEval;
      float score = autoTuneScore();
      at.evalCount++;

      VF("MSG:"); V(axisPrefix); VF("auto-tune eval "); V(at.evalCount);
      VF("/"); V(PID_AUTOTUNE_MAX_EVALUATIONS);
      VF(" aggregate: overshoot "); V(at.overshootResult);
      VF("/"); V(at.maxOvershootSteps); VF(" counts, settle "); V(at.settleTimeResult);
      VF("/"); V((long)PID_AUTOTUNE_MAX_SETTLE_MS); VF(" ms, residual "); V(at.residualResult);
      // the raw count, not just the verdict: "hunting 1/3" reads as an intermittent drive, while
      // "hunting 3/3" is the gain set, and the two want completely different responses from a reader
      VF("/"); V(at.settleBandSteps); VF(" counts, hunting "); V(at.huntingCount);
      VF("/"); V(at.repeatsThisEval);
      if (at.huntingCount > 0 && !hunting) { VF(" (isolated, not penalized)"); }
      VF(", score "); VL(score);

      // the opening evaluation measures the Config.h gains themselves and becomes the incumbent that
      // every probe is judged against.  without it the search would have no reference and the first
      // probe would be accepted unconditionally
      if (!at.incumbentValid) {
        for (int i = 0; i < 3; i++) { at.incumbent[i] = at.candidate[i]; at.staged[i] = at.candidate[i]; }
        at.incumbentScore = score;
        at.incumbentOvershoot = at.overshootResult;
        at.incumbentSettle = at.settleTimeResult;
        at.incumbentResidual = at.residualResult;
        at.incumbentHunting = hunting;
        at.startScore = score;
        at.incumbentValid = true;
        at.stagedValid = true;
        VF("MSG:"); V(axisPrefix); VF("auto-tune starting point P="); V(at.incumbent[0]);
        VF(" I="); V(at.incumbent[1]); VF(" D="); V(at.incumbent[2]);
        VF(" score "); V(score); VF(", searching at step ");
        V(lroundf(at.stepFraction*100.0F)); VLF("%");
        autoTuneBuildPoll();
        autoTuneAdvancePoll();
        return;
      }

      // accept only a clear improvement.  repeat-to-repeat scatter is real and a bare < would let the
      // search wander sideways through noise forever, never converging and never actually improving
      float threshold = at.incumbentScore*(1.0F - PID_AUTOTUNE_IMPROVE_PERCENT/100.0F);
      if (score < threshold) {
        at.lastMove = at.pollOrder[at.pollIndex];
        for (int i = 0; i < 3; i++) { at.incumbent[i] = at.candidate[i]; at.staged[i] = at.candidate[i]; }
        float was = at.incumbentScore;
        at.incumbentScore = score;
        at.incumbentOvershoot = at.overshootResult;
        at.incumbentSettle = at.settleTimeResult;
        at.incumbentResidual = at.residualResult;
        at.incumbentHunting = hunting;
        at.stagedValid = true;

        // this incumbent is new ground, so what was known about its neighbours no longer applies
        at.kpUpBlocked = false;

        VF("MSG:"); V(axisPrefix); VF("auto-tune improved, incumbent now P="); V(at.incumbent[0]);
        VF(" I="); V(at.incumbent[1]); VF(" D="); V(at.incumbent[2]);
        VF(" score "); V(score); VF(" (was "); V(was); VLF(")");

        // opportunistic: move now rather than finishing the poll.  a full poll costs six evaluations
        // and the remaining five are about a point the search has already left
        autoTuneBuildPoll();
        autoTuneAdvancePoll();
        return;
      }

      // this direction is no better.  a Kp+ that hunts is the same cliff a safety trip would be, and
      // the gain margin at the end keys off exactly this
      if (at.pollOrder[at.pollIndex] == 0 && hunting) at.kpUpBlocked = true;

      VF("MSG:"); V(axisPrefix); VF("auto-tune probe ");
      V(autoTuneMoveName(at.pollOrder[at.pollIndex]));
      VF(" no better ("); V(score); VF(" vs "); V(at.incumbentScore); VLF("), reverting");

      // restore the incumbent before the next probe steps off it
      for (int i = 0; i < 3; i++) at.candidate[i] = at.incumbent[i];
      at.pollIndex++;
      autoTuneAdvancePoll();
    } break;

    default: break;
  }
}

#endif
