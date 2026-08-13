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

  // settle band in counts; the load can't be positioned finer than the lash dead band
  long band = (long)(PID_AUTOTUNE_SETTLE_TOLERANCE_COUNTS);
  if (band <= 0) { band = lroundf(backlash*1.5F); if (band < 2) band = 2; }
  autoTuneSettleBandSteps = band;

  long maxOvershoot = (long)(PID_AUTOTUNE_MAX_OVERSHOOT_COUNTS);
  if (maxOvershoot <= 0) maxOvershoot = band*2;
  autoTuneMaxOvershootSteps = maxOvershoot;

  // measured move distance must have dynamic range against lash + encoder quantization
  float distance = testDistance;
  if (isnan(distance) || distance <= 0.0F) distance = PID_AUTOTUNE_TEST_DISTANCE;
  if (distance*settings.stepsPerMeasure < 20.0F*band) return CE_PARAM_RANGE;
  autoTuneTestDistance = distance;

  // preload nudge distance
  float preload = (float)(PID_AUTOTUNE_PRELOAD_DISTANCE);
  if (preload <= 0.0F) {
    long preloadSteps = backlash*2;
    if (preloadSteps < band*10) preloadSteps = band*10;
    preload = preloadSteps/settings.stepsPerMeasure;
  }
  autoTunePreloadDistance = preload;

  // test slew rate (NAN = keep the axis production rate)
  float rate = (float)(PID_AUTOTUNE_VALIDATION_SLEW_RATE);
  if (rate <= 0.0F) rate = NAN;
  autoTuneRate = rate;

  // per-move watchdog, also catches a goto that never terminates (exact-count hunting)
  long timeoutMs = (long)(PID_AUTOTUNE_MOVE_TIMEOUT_MS);
  if (timeoutMs <= 0) {
    float expectedRate = !isnan(rate) ? rate : slewFreq;
    if (expectedRate <= 0.0F) expectedRate = 1.0F;
    timeoutMs = lroundf(((distance + preload)/expectedRate)*4000.0F) + 8000;
  }
  autoTuneMoveTimeout = timeoutMs;

  // capture the pre-tune slewing gain set (restored at the end of every run)
  autoTuneOriginal[0] = settings.param4; autoTuneOriginal[1] = settings.param5; autoTuneOriginal[2] = settings.param6;
  for (int i = 0; i < 3; i++) { autoTuneCandidate[i] = autoTuneOriginal[i]; autoTuneBest[i] = autoTuneOriginal[i]; }
  autoTuneBestScore = 0.0F;
  autoTuneStagedValid = false;
  autoTuneIteration = 0; autoTuneRepeat = 0; autoTuneShutdownCount = 0; autoTuneHuntingCount = 0;
  autoTuneOvershootResult = 0.0F; autoTuneSettleTimeResult = 0.0F; autoTuneResidualResult = 0.0F;

  // walk direction: with bounded travel start toward the far limit
  autoTuneDirection = 1;
  if (!wrapEnabled) {
    double coordinate = getInstrumentCoordinate();
    if (coordinate - settings.limits.min > settings.limits.max - coordinate) autoTuneDirection = -1;
  }

  if (!enabled) enable(true);

  VF("MSG:"); V(axisPrefix); VF("auto-tune start, band "); V(autoTuneSettleBandSteps);
  VF(" counts, test "); V(autoTuneTestDistance); VF(", preload "); VL(autoTunePreloadDistance);

  autoTuneResult = ATR_NONE;
  autoTunePhase = 0;

  // physical maximum rotation rate measurement runs first: it both reports the
  // hardware's capability and keeps the tuning moves physically achievable
  #if PID_AUTOTUNE_SPEED_TEST == ON
    autoTuneSpeedDistance = (float)(PID_AUTOTUNE_SPEED_TEST_DISTANCE);
    if (autoTuneSpeedDistance <= 0.0F) autoTuneSpeedDistance = autoTuneTestDistance*3.0F;
    autoTuneSpeedRate = (float)(PID_AUTOTUNE_SPEED_TEST_RATE);
    if (autoTuneSpeedRate <= 0.0F) autoTuneSpeedRate = (!isnan(autoTuneRate) ? autoTuneRate : slewFreq)*2.0F;
    autoTuneMeasuredMaxRate = 0.0F;
    autoTunePeakRate = 0.0F;
    autoTuneSpeedSaturated = false;
    autoTuneState = AT_SPEED;
  #else
    autoTuneState = AT_PRELOAD;
  #endif
  return CE_NONE;
}

// abort a running PID auto-tune, restoring the pre-tune gains
void Axis::autoTuneAbort() {
  if (autoTuneActive()) autoTuneFinish(ATR_ABORTED); else autoTuneState = AT_IDLE;
}

// apply the staged auto-tune result live and persist it to NV (same path as :SXA)
CommandError Axis::autoTuneApply() {
  if (autoTuneActive()) return CE_SLEW_IN_MOTION;
  if (!autoTuneStagedValid) return CE_0;

  settings.param4 = autoTuneStaged[0];
  settings.param5 = autoTuneStaged[1];
  settings.param6 = autoTuneStaged[2];
  nv.updateBytes(NV_AXIS_SETTINGS_BASE + (axisNumber - 1)*AxisStoredSettingsSize, &settings, sizeof(AxisStoredSettings));
  motor->setParameters(settings.param1, settings.param2, settings.param3, settings.param4, settings.param5, settings.param6);
  if (motor->enabled) { motor->enable(false); motor->enable(true); }

  VF("MSG:"); V(axisPrefix); VLF("auto-tune gains applied and saved to NV");
  return CE_NONE;
}

// apply the candidate gains: setParameters() then an enable toggle so Pid::reset()
// reloads param4-6 into SetTunings() and clears any accumulated windup
void Axis::autoTuneSetCandidate() {
  motor->setParameters(settings.param1, settings.param2, settings.param3, autoTuneCandidate[0], autoTuneCandidate[1], autoTuneCandidate[2]);
  motor->enable(false);
  motor->enable(true);
}

// end the run: stop motion, release the slewing-set hold, restore pre-tune gains
void Axis::autoTuneFinish(AutoTuneResult result) {
  if (autoRate != AR_NONE) autoSlewAbort();
  if (autoTuneHoldSlewing) { motor->setSlewing(false); autoTuneHoldSlewing = false; }

  // restore the frequency ceiling and slew rate if the speed test lifted them
  if (autoTuneMaxFreqOverride) {
    maxFreq = autoTuneMaxFreqStore;
    setFrequencySlew(autoTuneSlewFreqStore);
    autoTuneMaxFreqOverride = false;
  }

  // restore the pre-tune gains; never re-arm a motor a safety layer disabled
  motor->setParameters(settings.param1, settings.param2, settings.param3, autoTuneOriginal[0], autoTuneOriginal[1], autoTuneOriginal[2]);
  if (motor->enabled) { motor->enable(false); motor->enable(true); }

  autoTuneResult = result;
  if (result == ATR_CONVERGED || result == ATR_BEST_EFFORT) {
    autoTuneState = AT_DONE_SUCCESS;
    VF("MSG:"); V(axisPrefix); VF("auto-tune done P="); V(autoTuneStaged[0]);
    VF(" I="); V(autoTuneStaged[1]); VF(" D="); V(autoTuneStaged[2]);
    if (result == ATR_BEST_EFFORT) { VLF(" (did not fully converge)"); } else { VLF(" (converged)"); }
  } else {
    autoTuneStagedValid = false;
    autoTuneState = AT_DONE_FAIL;
    VF("MSG:"); V(axisPrefix); VF("auto-tune ended, code "); VL(result);
  }
}

// handle a servo safety shutdown (stall/runaway/oscillation) or driver fault
// mid-run; returns true if one occurred and was handled
bool Axis::autoTuneSafetyEvent() {
  if ((!motor->enabled && !poweredDown) || motorFault()) {
    VF("MSG:"); V(axisPrefix); VLF("auto-tune servo safety shutdown/fault");
    if (autoTuneHoldSlewing) { motor->setSlewing(false); autoTuneHoldSlewing = false; }

    // restore the frequency ceiling if the speed test had lifted it
    if (autoTuneMaxFreqOverride) {
      maxFreq = autoTuneMaxFreqStore;
      setFrequencySlew(autoTuneSlewFreqStore);
      autoTuneMaxFreqOverride = false;
    }

    autoTuneShutdownCount++;
    if (autoTuneShutdownCount >= 2) { autoTuneFinish(ATR_SAFETY_SHUTDOWN); return true; }

    // mandatory gain backoff BEFORE the motor is re-armed (enable(true) clears the
    // servo safetyShutdown latch, so a blind retry would re-trip with the same gains)
    for (int i = 0; i < 3; i++) autoTuneCandidate[i] *= PID_AUTOTUNE_SAFETY_BACKOFF_PERCENT/100.0F;

    // discard the phase in progress and restart it with the backed-off gains
    // (a speed test restarts from its own beginning and re-lifts the ceiling)
    autoTuneRepeat = 0; autoTuneHuntingCount = 0;
    autoTunePhase = 0;
    if (autoTuneState != AT_SPEED) autoTuneState = AT_PRELOAD;
    return true;
  }
  return false;
}

// robust aggregate of a round's samples: median + modified Z-score (Iglewicz-Hoaglin)
// outlier filter, mean of the survivors; falls back to the median when MAD is zero
float Axis::autoTuneAggregate(float values[], int count) {
  float sorted[PID_AUTOTUNE_REPEATS];
  for (int i = 0; i < count; i++) {
    float v = values[i];
    int j = i;
    while (j > 0 && sorted[j - 1] > v) { sorted[j] = sorted[j - 1]; j--; }
    sorted[j] = v;
  }
  float median = (count % 2) ? sorted[count/2] : (sorted[count/2 - 1] + sorted[count/2])/2.0F;

  float deviations[PID_AUTOTUNE_REPEATS];
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

// advance the auto-tune state machine, called from poll() at FRACTIONAL_SEC Hz
void Axis::autoTunePoll() {
  switch (autoTuneState) {

    #if PID_AUTOTUNE_SPEED_TEST == ON
    case AT_SPEED: {
      if (autoTuneSafetyEvent()) return;

      if (autoTunePhase == 0) {
        // with bounded travel keep preload + speed move inside the limits
        if (!wrapEnabled) {
          double coordinate = getInstrumentCoordinate();
          double need = autoTunePreloadDistance + autoTuneSpeedDistance*1.25F;
          if (autoTuneDirection > 0 && coordinate + need > settings.limits.max) autoTuneDirection = -1; else
          if (autoTuneDirection < 0 && coordinate - need < settings.limits.min) autoTuneDirection = 1;
        }

        // fresh gains/integrator, then the usual unmeasured backlash take-up nudge
        autoTuneSetCandidate();
        double target = getInstrumentCoordinate() + autoTuneDirection*autoTunePreloadDistance;
        if (wrapEnabled) target = wrap(target);
        setTargetCoordinate(target);
        if (autoGoto(autoTuneRate) != CE_NONE) { autoTuneFinish(ATR_MOTION_ERROR); return; }
        autoTuneWatchdogTime = millis() + autoTuneMoveTimeout;
        autoTunePhase = 1;
      } else

      if (autoTunePhase == 1) {
        if (autoRate != AR_NONE) {
          if ((long)(millis() - autoTuneWatchdogTime) > 0) { autoSlewAbort(); autoTuneFinish(ATR_TIMEOUT); }
          return;
        }
        autoTuneDwellTime = millis() + 500;
        autoTunePhase = 2;
      } else

      if (autoTunePhase == 2) {
        if ((long)(millis() - autoTuneDwellTime) <= 0) return;

        // temporarily lift the axis frequency ceiling: setFrequencySlew()/setFrequency()
        // clamp to maxFreq, which the dome sets to the production slew rate, so an
        // over-speed command would otherwise be silently limited to production speed
        autoTuneMaxFreqStore = maxFreq;
        autoTuneSlewFreqStore = slewFreq;
        maxFreq = autoTuneSpeedRate;
        autoTuneMaxFreqOverride = true;

        double target = getInstrumentCoordinate() + autoTuneDirection*autoTuneSpeedDistance;
        if (wrapEnabled) target = wrap(target);
        setTargetCoordinate(target);
        if (autoGoto(autoTuneSpeedRate) != CE_NONE) {
          maxFreq = autoTuneMaxFreqStore; setFrequencySlew(autoTuneSlewFreqStore); autoTuneMaxFreqOverride = false;
          autoTuneFinish(ATR_MOTION_ERROR);
          return;
        }

        // watchdog sized from the production rate: the axis may be much slower than commanded
        float productionRate = autoTuneMaxFreqStore > 0.0F ? autoTuneMaxFreqStore : 1.0F;
        autoTuneWatchdogTime = millis() + (unsigned long)(lroundf((autoTuneSpeedDistance/productionRate)*4000.0F)) + 8000UL;

        autoTuneLastPositionSteps = motor->getInstrumentCoordinateSteps();
        autoTuneLastSampleTime = millis();
        autoTuneVelocityEstimate = 0.0F;
        autoTunePhase = 3;
      } else

      if (autoTunePhase == 3) {
        if ((long)(millis() - autoTuneWatchdogTime) > 0) {
          autoSlewAbort();
          autoTuneFinish(ATR_TIMEOUT);  // finish restores the lifted ceiling
          return;
        }

        // sample encoder velocity ~50Hz, smooth with an IIR (tau ~200ms,) and record
        // the plateau: the physical maximum is the smoothed rate while the drive
        // output is saturated (it can push no harder)
        unsigned long now = millis();
        long dt = (long)(now - autoTuneLastSampleTime);
        if (dt >= 20) {
          long position = motor->getInstrumentCoordinateSteps();
          float rate = fabsf((float)(position - autoTuneLastPositionSteps))/settings.stepsPerMeasure/(dt/1000.0F);
          autoTuneLastPositionSteps = position;
          autoTuneLastSampleTime = now;
          autoTuneVelocityEstimate += (rate - autoTuneVelocityEstimate)*0.1F;

          bool saturated = fabsf(((ServoMotor*)motor)->velocityPercent) >= PID_AUTOTUNE_SPEED_SATURATION_PERCENT;
          if (saturated) autoTuneSpeedSaturated = true;
          if (autoTuneVelocityEstimate > autoTunePeakRate) autoTunePeakRate = autoTuneVelocityEstimate;
          if (saturated && autoTuneVelocityEstimate > autoTuneMeasuredMaxRate) autoTuneMeasuredMaxRate = autoTuneVelocityEstimate;
        }

        if (autoRate != AR_NONE) return;  // move still in progress

        // move done: without saturation the peak only shows how fast the move went
        // (the true maximum is higher than the commanded rate)
        if (!autoTuneSpeedSaturated || autoTuneMeasuredMaxRate <= 0.0F) autoTuneMeasuredMaxRate = autoTunePeakRate;

        maxFreq = autoTuneMaxFreqStore;
        setFrequencySlew(autoTuneSlewFreqStore);
        autoTuneMaxFreqOverride = false;

        VF("MSG:"); V(axisPrefix); VF("auto-tune measured max rate "); V(autoTuneMeasuredMaxRate);
        if (autoTuneSpeedSaturated) { VLF(" (drive saturated: physical maximum)"); }
        else { VLF(" (drive NOT saturated: true maximum is higher)"); }

        // keep the tuning moves physically achievable: clamp the test slew rate
        // to the recommended fraction of the measured maximum
        float recommended = autoTuneMeasuredMaxRate*(PID_AUTOTUNE_SPEED_HEADROOM_PERCENT/100.0F);
        float testRate = !isnan(autoTuneRate) ? autoTuneRate : slewFreq;
        if (autoTuneSpeedSaturated && recommended > 0.0F && testRate > recommended) {
          autoTuneRate = recommended;
          VF("MSG:"); V(axisPrefix); VF("auto-tune test rate clamped to "); V(autoTuneRate);
          VLF(" (configured slew rate exceeds the measured maximum)");
        }

        autoTuneDwellTime = millis() + 500;
        autoTunePhase = 4;
      } else {
        if ((long)(millis() - autoTuneDwellTime) > 0) { autoTunePhase = 0; autoTuneState = AT_PRELOAD; }
      }
    } break;
    #endif

    case AT_PRELOAD: {
      if (autoTuneSafetyEvent()) return;

      if (autoTunePhase == 0) {
        // with bounded travel keep preload + move + overshoot headroom inside the limits
        if (!wrapEnabled) {
          double coordinate = getInstrumentCoordinate();
          double need = autoTunePreloadDistance + autoTuneTestDistance*1.25F;
          if (autoTuneDirection > 0 && coordinate + need > settings.limits.max) autoTuneDirection = -1; else
          if (autoTuneDirection < 0 && coordinate - need < settings.limits.min) autoTuneDirection = 1;
        }

        // fresh candidate gains and a clean integrator for this repeat
        autoTuneSetCandidate();

        // unmeasured same-direction nudge: enter the measured move with the lash
        // fully taken up in the test direction, whatever the last move left behind
        double target = getInstrumentCoordinate() + autoTuneDirection*autoTunePreloadDistance;
        if (wrapEnabled) target = wrap(target);
        setTargetCoordinate(target);
        if (autoGoto(autoTuneRate) != CE_NONE) { autoTuneFinish(ATR_MOTION_ERROR); return; }
        autoTuneWatchdogTime = millis() + autoTuneMoveTimeout;
        autoTunePhase = 1;
      } else

      if (autoTunePhase == 1) {
        if (autoRate != AR_NONE) {
          if ((long)(millis() - autoTuneWatchdogTime) > 0) { autoSlewAbort(); autoTuneFinish(ATR_TIMEOUT); }
          return;
        }
        autoTuneDwellTime = millis() + 500;
        autoTunePhase = 2;
      } else {
        if ((long)(millis() - autoTuneDwellTime) > 0) { autoTunePhase = 0; autoTuneState = AT_MOVE; }
      }
    } break;

    case AT_MOVE: {
      if (autoTuneSafetyEvent()) return;

      double target = getInstrumentCoordinate() + autoTuneDirection*autoTuneTestDistance;
      if (wrapEnabled) target = wrap(target);
      setTargetCoordinate(target);
      if (autoGoto(autoTuneRate) != CE_NONE) { autoTuneFinish(ATR_MOTION_ERROR); return; }

      autoTunePeakOvershoot = 0;
      autoTuneMeasuring = false;
      autoTuneMoveStartTime = millis();  // provisional, reset once clear of any residual backlash
      autoTuneWatchdogTime = millis() + autoTuneMoveTimeout;
      autoTuneArriveTime = 0;
      autoTuneInBand = false; autoTuneEverInBand = false; autoTuneBandCrossings = 0; autoTuneHunting = false;
      autoTuneStableDistance = motor->getTargetDistanceSteps();
      autoTuneStableStartTime = millis();
      autoTuneState = AT_MONITOR;
    } break;

    case AT_MONITOR: {
      if (autoTuneSafetyEvent()) return;
      if ((long)(millis() - autoTuneWatchdogTime) > 0) { autoSlewAbort(); autoTuneFinish(ATR_TIMEOUT); return; }

      // exclude residual backlash take-up from the timing
      if (!autoTuneMeasuring && !motor->inBacklash) { autoTuneMeasuring = true; autoTuneMoveStartTime = millis(); }

      long distanceSteps = motor->getTargetDistanceSteps();

      // peak excursion beyond the target in the direction of travel, in counts
      if (autoTuneDirection > 0 ? distanceSteps < 0 : distanceSteps > 0) {
        long overshoot = labs(distanceSteps);
        if (overshoot > autoTunePeakOvershoot) autoTunePeakOvershoot = overshoot;
      }

      if (autoRate != AR_NONE) return;  // trajectory still in progress

      if (autoTuneArriveTime == 0) {
        autoTuneArriveTime = millis();
        // a goto that ended far from the target was aborted (limit sense/motion error,) not completed
        if (labs(distanceSteps) > lroundf((autoTuneTestDistance*settings.stepsPerMeasure)/2.0F)) {
          autoTuneFinish(ATR_MOTION_ERROR);
          return;
        }
        // hold the slewing gain set selected so the whole measurement reflects the
        // set being tuned (otherwise it decays to the tracking set after 3s)
        motor->setSlewing(true); autoTuneHoldSlewing = true;
        autoTuneStableDistance = distanceSteps; autoTuneStableStartTime = millis();
      }

      // settle band crossings after first entry classify lash/integrator limit cycling
      bool nowInBand = labs(distanceSteps) <= autoTuneSettleBandSteps;
      if (nowInBand != autoTuneInBand) {
        if (autoTuneEverInBand) autoTuneBandCrossings++;
        if (nowInBand) autoTuneEverInBand = true;
        autoTuneInBand = nowInBand;
      }
      if (autoTuneBandCrossings > PID_AUTOTUNE_MAX_BAND_CROSSINGS) autoTuneHunting = true;

      // settled = position stable for the confirm period, in or out of the band
      // (out-of-band stability is a real result: frozen following error, drives Ki)
      long wobble = autoTuneSettleBandSteps/4;
      if (wobble < 2) wobble = 2;
      if (labs(distanceSteps - autoTuneStableDistance) > wobble) {
        autoTuneStableDistance = distanceSteps;
        autoTuneStableStartTime = millis();
      }
      bool settled = (long)(millis() - autoTuneStableStartTime) >= PID_AUTOTUNE_SETTLE_CONFIRM_MS;
      if (!settled && !autoTuneHunting) return;

      // record this repeat; residual within the lash band is physically zero
      long residual = labs(distanceSteps);
      if (residual <= autoTuneSettleBandSteps) residual = 0;
      autoTuneOvershoot[autoTuneRepeat] = (float)autoTunePeakOvershoot;
      autoTuneSettleTime[autoTuneRepeat] = (float)((autoTuneHunting ? millis() : autoTuneStableStartTime) - autoTuneMoveStartTime);
      autoTuneResidual[autoTuneRepeat] = (float)residual;
      if (autoTuneHunting) autoTuneHuntingCount++;

      motor->setSlewing(false); autoTuneHoldSlewing = false;

      autoTuneRepeat++;
      autoTunePhase = 0;
      if (autoTuneRepeat >= PID_AUTOTUNE_REPEATS) autoTuneState = AT_AGGREGATE; else autoTuneState = AT_PRELOAD;
    } break;

    case AT_AGGREGATE: {
      autoTuneOvershootResult = autoTuneAggregate(autoTuneOvershoot, PID_AUTOTUNE_REPEATS);
      autoTuneSettleTimeResult = autoTuneAggregate(autoTuneSettleTime, PID_AUTOTUNE_REPEATS);
      autoTuneResidualResult = autoTuneAggregate(autoTuneResidual, PID_AUTOTUNE_REPEATS);
      autoTuneState = AT_ANALYZE;
    } break;

    case AT_ANALYZE: {
      bool hunting = autoTuneHuntingCount > 0;

      VF("MSG:"); V(axisPrefix); VF("auto-tune round "); V(autoTuneIteration + 1);
      VF(" overshoot "); V(autoTuneOvershootResult); VF(" counts, settle "); V(autoTuneSettleTimeResult);
      VF(" ms, residual "); V(autoTuneResidualResult); VF(" counts, hunting "); VL(hunting);

      // running-best bookkeeping (fallback if the iteration cap is reached)
      float score = autoTuneOvershootResult/(float)autoTuneMaxOvershootSteps +
                    autoTuneSettleTimeResult/(float)PID_AUTOTUNE_MAX_SETTLE_MS +
                    autoTuneResidualResult/(float)autoTuneSettleBandSteps +
                    (hunting ? 10.0F : 0.0F);
      if (autoTuneIteration == 0 || score < autoTuneBestScore) {
        autoTuneBestScore = score;
        for (int i = 0; i < 3; i++) autoTuneBest[i] = autoTuneCandidate[i];
      }

      // acceptance
      if (!hunting &&
          autoTuneOvershootResult <= (float)autoTuneMaxOvershootSteps &&
          autoTuneSettleTimeResult <= (float)PID_AUTOTUNE_MAX_SETTLE_MS &&
          autoTuneResidualResult <= (float)autoTuneSettleBandSteps) {
        for (int i = 0; i < 3; i++) autoTuneStaged[i] = autoTuneCandidate[i];
        autoTuneStagedValid = true;
        autoTuneFinish(ATR_CONVERGED);
        return;
      }

      autoTuneIteration++;
      if (autoTuneIteration >= PID_AUTOTUNE_MAX_ITERATIONS) {
        for (int i = 0; i < 3; i++) autoTuneStaged[i] = autoTuneBest[i];
        autoTuneStagedValid = true;
        autoTuneFinish(ATR_BEST_EFFORT);
        return;
      }

      // bounded classical correction: the direction rules carry the logic,
      // the per-round clamp carries the safety
      float limit = PID_AUTOTUNE_GAIN_STEP_LIMIT_PERCENT/100.0F;
      float up = 1.0F + limit, down = 1.0F - limit;

      if (hunting) {
        // integrator + backlash limit cycle: Ki down a full step, Kp down a half step
        autoTuneCandidate[0] *= 1.0F - limit*0.5F;
        autoTuneCandidate[1] *= down;
      } else
      if (autoTuneOvershootResult > (float)autoTuneMaxOvershootSteps) {
        // under-damped: Kp down proportional to the excess, Kd up
        float f = (float)autoTuneMaxOvershootSteps/autoTuneOvershootResult;
        if (f < down) f = down;
        autoTuneCandidate[0] *= f;
        float g = autoTuneOvershootResult/(float)autoTuneMaxOvershootSteps;
        if (g > up) g = up;
        if (autoTuneCandidate[2] > 0.01F) autoTuneCandidate[2] *= g; else autoTuneCandidate[2] = 0.05F;
      } else
      if (autoTuneSettleTimeResult > (float)PID_AUTOTUNE_MAX_SETTLE_MS) {
        // sluggish: Kp up proportional to the shortfall
        float f = autoTuneSettleTimeResult/(float)PID_AUTOTUNE_MAX_SETTLE_MS;
        if (f > up) f = up;
        autoTuneCandidate[0] *= f;
      }
      if (!hunting && autoTuneResidualResult > (float)autoTuneSettleBandSteps) {
        // frozen following error beyond the lash band: Ki up a half step
        if (autoTuneCandidate[1] > 0.01F) autoTuneCandidate[1] *= 1.0F + limit*0.5F; else autoTuneCandidate[1] = 0.05F;
      }

      if (!motor->validateParameters(settings.param1, settings.param2, settings.param3, autoTuneCandidate[0], autoTuneCandidate[1], autoTuneCandidate[2])) {
        for (int i = 0; i < 3; i++) autoTuneStaged[i] = autoTuneBest[i];
        autoTuneStagedValid = true;
        autoTuneFinish(ATR_BEST_EFFORT);
        return;
      }

      VF("MSG:"); V(axisPrefix); VF("auto-tune next P="); V(autoTuneCandidate[0]);
      VF(" I="); V(autoTuneCandidate[1]); VF(" D="); VL(autoTuneCandidate[2]);

      // alternate the walk direction between rounds (one reversal per round,
      // absorbed by the next preload) so both directions get coverage
      if (wrapEnabled) autoTuneDirection = -autoTuneDirection;

      autoTuneRepeat = 0; autoTuneHuntingCount = 0;
      autoTunePhase = 0;
      autoTuneState = AT_PRELOAD;
    } break;

    default: break;
  }
}

#endif
