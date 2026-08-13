// -----------------------------------------------------------------------------------
// Axis motion control
#pragma once

#include "../../Common.h"
#include "../sense/Sense.h"

#ifdef MOTOR_PRESENT

// default start of axis class hardware timers
#ifndef AXIS_HARDWARE_TIMER_BASE
#define AXIS_HARDWARE_TIMER_BASE    1
#endif

// default location of settings in NV
#ifndef NV_AXIS_SETTINGS_REVERT
#define NV_AXIS_SETTINGS_REVERT     100    // bytes: 2   , addr:  100.. 101
#endif
#ifndef NV_AXIS_SETTINGS_BASE
#define NV_AXIS_SETTINGS_BASE       102    // bytes: 25  , addr:  102.. 224 (for 9 axes)
#endif

// polling frequency for monitoring axis motion (default 100X/second) 
#ifndef FRACTIONAL_SEC
#define FRACTIONAL_SEC              100.0F
#endif
#define FRACTIONAL_SEC_US           (lround(1000000.0F/FRACTIONAL_SEC))

// time limit in seconds for slew home phases
#ifndef SLEW_HOME_REFINE_TIME_LIMIT
#define SLEW_HOME_REFINE_TIME_LIMIT 120
#endif

// ON blocks all motion when min/max are on the same pin, applies to all axes (mount/rotator/focusers)
#ifndef LIMIT_SENSE_STRICT
#define LIMIT_SENSE_STRICT          OFF
#endif

#include "../../libApp/commands/ProcessCmds.h"
#include "motor/Motor.h"
#include "motor/stepDir/StepDir.h"
#include "motor/servo/Servo.h"
#include "motor/oDrive/ODrive.h"
#include "motor/kTech/KTech.h"

// helpers for step/dir and servo parameters
#define subdivisions param1
#define subdivisionsGoto param2
#define currentHold param3
#define currentRun param4
#define currentGoto param5
#define integral param1
#define porportional param2
#define derivative param3
#define integralGoto param4
#define porportionalGoto param5
#define derivativeGoto param6

#pragma pack(1)
typedef struct AxisLimits {
  float min;                  // unit (radians, microns, etc.) depends on axis settings
  float max;                  // unit (radians, microns, etc.) depends on axis settings
} AxisLimits;

typedef struct AxisSettings {
  double     stepsPerMeasure; // unit (radians, microns, etc.) depends on axis settings
  int8_t     reverse;         // ON or OFF
  AxisLimits limits;
  float      backlashFreq;    // in measures/s
} AxisSettings;

#define AxisStoredSettingsSize 41
typedef struct AxisStoredSettings {
  double     stepsPerMeasure; // unit (radians, microns, etc.) depends on axis settings
  int8_t     reverse;         // ON or OFF
  float      param1, param2, param3, param4, param5, param6; // content depends on axis settings
  AxisLimits limits;
} AxisStoredSettings;
#pragma pack()

typedef struct AxisSense {
  int32_t    homeTrigger;
  int8_t     homeInit;
  float      homeDistLimit;
  int32_t    minTrigger;
  int32_t    maxTrigger;
  int8_t     minMaxInit;
} AxisSense;

typedef struct AxisPins {
  int16_t    min;
  int16_t    home;
  int16_t    max;
  AxisSense  axisSense;
} AxisPins;

typedef struct AxisErrors {
  uint8_t minLimitSensed:1;
  uint8_t maxLimitSensed:1;
} AxisErrors;

enum AutoRate: uint8_t {AR_NONE, AR_RATE_BY_TIME_ABORT, AR_RATE_BY_TIME_END, AR_RATE_BY_DISTANCE, AR_RATE_BY_TIME_FORWARD, AR_RATE_BY_TIME_REVERSE};
enum HomingStage: uint8_t {HOME_NONE, HOME_FINE, HOME_SLOW, HOME_FAST};
enum AxisMeasure: uint8_t {AXIS_MEASURE_UNKNOWN, AXIS_MEASURE_MICRONS, AXIS_MEASURE_DEGREES, AXIS_MEASURE_RADIANS};

#ifdef SERVO_PID_AUTOTUNE_PRESENT
// PID auto-tune state machine states
enum AutoTuneState: uint8_t {AT_IDLE, AT_SPEED, AT_PRELOAD, AT_MOVE, AT_MONITOR, AT_AGGREGATE, AT_ANALYZE, AT_DONE_SUCCESS, AT_DONE_FAIL};
// PID auto-tune result/refusal codes (reported by :GXT)
enum AutoTuneResult: uint8_t {ATR_NONE, ATR_CONVERGED, ATR_BEST_EFFORT, ATR_ABORTED, ATR_TIMEOUT, ATR_FAULT, ATR_SAFETY_SHUTDOWN, ATR_MOTION_ERROR};
#endif

class Axis {
  public:
    // constructor for this motion controller
    // \param axisNumber: from 1 to 9
    // \param *pins: provides pins and states for h/w limits and homing
    // \param *settings: provides axis scale (steps per measure,) s/w limits, reversal, and backlash frequency
    // \param axisMeasure: one of AXIS_MEASURE_MICRONS, _DEGREES, or _RADIANS for conversion to/from steps
    // \param targetTolerance: in "measures" for detecting when slews arrive at their target
    Axis(uint8_t axisNumber, const AxisPins *pins, const AxisSettings *settings, const AxisMeasure axisMeasure, float targetTolerance = 0.0F);

    // process axis commands
    bool command(char *reply, char *command, char *parameter, bool *supressFrame, bool *numericReply, CommandError *commandError);

    // init prepares this motion controller and the provided motor for operation
    // \param motor: associated motor to be controlled
    bool init(Motor *motor);

    // sets the enabled state (and for its associated motor if supported)
    // \param state: true to enable or false to disable
    void enable(bool state);

    // gets the enabled state
    inline bool isEnabled() { return enabled && !motor->calibrating; }

    // sets the automatic power down time
    // \param time: allowed standstill time in milliseconds before power down, or 0 to disable
    void setPowerDownTime(int time);

    // sets the automatic power down override time
    // \param time: time in milliseconds to disable automatic power down, or 0 to disable
    void setPowerDownOverrideTime(int time);

    // get steps per "measure" (radian, micron, etc.)
    inline double getStepsPerMeasure() { return settings.stepsPerMeasure; }

    // get tracking mode steps per slewing mode step
    inline int getStepsPerStepSlewing() { return motor->getStepsPerStepSlewing(); }

    // reset position, also zeros backlash and index
    // \param value: position of motor and target in "measures" (radians, microns, etc.)
    CommandError resetPosition(double value);

    // reset position, also zeros backlash and index
    // \param value: position of motor and target in steps
    CommandError resetPositionSteps(long value);

    // resets the target position to the motor position
    inline void resetTargetToMotorPosition() { motor->resetTargetToMotorPosition(); }

    // get motor position, in "measures" (radians, microns, etc.)
    // \note motor: position based on actual motion taken since startup/reset (including backlash)
    double getMotorPosition();

    // get motor position, in steps
    // \note motor: position based on actual motion taken since startup/reset (including backlash)
    inline long getMotorPositionSteps() { return motor->getMotorPositionSteps(); }

    // get index position, in "measures" (radians, microns, etc.)
    // \note indexPosition: added to motorPositon (-backlash) yeilds an instrumentCoordinate
    double getIndexPosition();

    // get index position, in steps
    // \note indexPosition: added to motorPositon (-backlash) yeilds an instrumentCoordinate
    inline long getIndexPositionSteps() { return motor->getIndexPositionSteps(); }

    // coordinate wrap
    // \param value: one complete rotation in "measures" (radians, microns, etc.), 0.0 disables
    // \note wrap occurs when the coordinate exceeds the min or max limits
    inline void coordinateWrap(double value) { wrapAmount = value; wrapEnabled = fabs(value) > 0.0F; }

    // set instrument coordinate
    // \param value: position in "measures" (radians, microns, etc.)
    void setInstrumentCoordinate(double value);

    // set instrument coordinate
    // \param value: position in steps
    inline void setInstrumentCoordinateSteps(long value) { motor->setInstrumentCoordinateSteps(value); }

    // get instrument coordinate in "measures" (radians, microns, etc.)
    double getInstrumentCoordinate();

    // get instrument coordinate, in steps
    inline long getInstrumentCoordinateSteps() { return motor->getInstrumentCoordinateSteps(); }

    // set instrument coordinate to park at
    // \param value: position in "measures" (radians, microns, etc.)
    // \note with backlash disabled this indexes to the nearest position where the motor wouldn't cog
    void setInstrumentCoordinatePark(double value);

    // set the target coordinate for park
    // \param value: postion in "measures" (degrees, microns, etc.)
    // \note with backlash disabled this moves to the nearest position where the motor doesn't cog
    void setTargetCoordinatePark(double value);

    // set target coordinate for autoGoto()
    // \param value: postion in "measures" (degrees, microns, etc.)
    void setTargetCoordinate(double value);

    // set target coordinate for autoGoto()
    // \param value: postion in steps
    inline void setTargetCoordinateSteps(long value) { motor->setTargetCoordinateSteps(value); }

    // get target coordinate, in "measures" (degrees, microns, etc.)
    double getTargetCoordinate();

    // get target coordinate, in steps
    inline long getTargetCoordinateSteps() { return motor->getTargetCoordinateSteps(); }

    // distance to target in "measures" (degrees, microns, etc.)
    double getTargetDistance();

    // returns true if at the target coordinate (+/- targetTolerance)
    bool atTarget();

    // returns true if within one second of the target coordinate at the backlash takeup rate
    bool nearTarget();

    // sets backlash amount
    // \param value: backlash distance in "measures" (degrees, microns, etc.)
    void setBacklash(float value);

    // sets backlash amount
    // \param value: backlash distance in steps
    inline void setBacklashSteps(long value) { if (autoRate == AR_NONE) motor->setBacklashSteps(value); }

    // get backlash amount in "measures" (radians, microns, etc.)
    float getBacklash();

    // get frequency in "measures" (degrees, microns, etc.) per second
    float getFrequency();

    // get frequency in steps per second
    float getFrequencySteps() { return motor->getFrequencySteps(); }

    // get direction
    Direction getDirection() { return motor->getDirection(); }

    // gets backlash frequency in "measures" (degrees, microns, etc.) per second
    float getBacklashFrequency();

    // sets reversal of axis directions
    // \param state: true reverses the direction behavior specified in settings
    inline void setReverse(bool state) {
      if (state) {
        if (settings.reverse == ON) motor->setReverse(OFF); else motor->setReverse(ON);
      } else {
        motor->setReverse(settings.reverse);
      }
    }

    // sets reversal of homing directions
    // \param state: true reverses the normal sense direction behavior
    inline void setHomeReverse(bool state) { sense.reverse(homeSenseHandle, state); }

    // sets maximum frequency for next slew
    // \param frequency: rate of motion in "measures" (radians, microns, etc.) per second
    void setFrequencySlew(float frequency);

    // sets overall minimum frequency
    // \param frequency: rate of motion in "measures" (radians, microns, etc.) per second
    void setFrequencyMin(float frequency);

    // sets overall maximum frequency
    // \param frequency: rate of motion in "measures" (radians, microns, etc.) per second
    void setFrequencyMax(float frequency);

    // sets frequency scaling factor for slews
    // \param frequency: scaling factor for this axis ranging from 0.0 (0%) to 1.0 (100%) of normal
    void setFrequencyScale(float frequency) { if (frequency >= 0.0F && frequency <= 1.0F) scaleFreq = frequency; }

    // sets acceleration rate for slews
    // \param mpsps: acceleration rate in "measures" per second per second
    void setSlewAccelerationRate(float mpsps);

    // set acceleration rate for slews
    // \param seconds: time during which acceleration to/from the last specified slew frequency occurs
    void setSlewAccelerationTime(float seconds);

    // sets acceleration rate for emergency stop of movement
    // \param mpsps: deceleration rate in "measures" per second per second
    void setSlewAccelerationRateAbort(float mpsps);

    // set acceleration rate for emergency stop of movement
    // \param seconds: time during which deceleration from the last specified slew frequency occurs
    void setSlewAccelerationTimeAbort(float seconds);

    // auto goto to destination target coordinate
    // \param frequency: optional frequency of slew in "measures" (radians, microns, etc.) per second
    CommandError autoGoto(float frequency = NAN);

    // auto slew
    // \param direction: direction of motion, DIR_FORWARD or DIR_REVERSE
    // \param frequency: optional frequency of slew in "measures" (radians, microns, etc.) per second
    CommandError autoSlew(Direction direction, float frequency = NAN);

    // automatic slew to home
    // \param timeout: time in seconds before aborting the operation
    // \note requires a home sensor (switch)
    CommandError autoSlewHome(unsigned long timeout = 0);

    // returns true homing is in progress
    bool isHoming() { return homingStage != HOME_NONE; }

    // returns true if a home sensor is available
    inline bool hasHomeSense() { return pins->axisSense.homeTrigger != OFF; }

    // stops any autoGoto() or autoSlew() or autoSlewHome() with deacceleration by time
    void autoSlewStop();

    // stops any autoGoto() or autoSlew() or autoSlewHome() with deacceleration by time using the abort rate
    void autoSlewAbort();

    // returns true if a slew is active on this axis
    bool isSlewing();

    // returns 1 if departing origin or -1 if approaching target
    inline int getRampDirection() { return motor->getRampDirection(); }

    // sets synchronized automatic target movement frequency
    // \param frequency: rate of motion in "measures" (radians, microns, etc.) per second
    inline void setSynchronizedFrequency(float frequency) { baseFreq = frequency; }

    // set synchronized automatic target movement
    // \param state: true to force target synchronized movement
    // \note movement at the setSynchronizedFrequency() rate
    inline void setSynchronized(bool state) { motor->setSynchronized(state); }

    // returns true if synchronized automatic target movement is active
    // \note movement at the setSynchronizedFrequency() rate
    inline bool getSynchronized() { return motor->getSynchronized(); }

    // returns true if the associated motor reports a fault
    inline bool motorFault() { return motor->getDriverStatus().fault; };

    // returns the associated motors driver status
    DriverStatus getStatus();

    // enables or disables software based position range limits
    // \param state: true to enable or false to disable
    void setMotionLimitsCheck(bool state);

    // check for all errors that would disallow motion in the given direction
    // \param direction: one of DIR_FORWARD, DIR_REVERSE, or DIR_BOTH
    bool motionError(Direction direction);

    // check for sense (h/w limit) errors that would disallow motion in the given direction
    // \param direction: one of DIR_FORWARD, DIR_REVERSE, or DIR_BOTH
    bool motionErrorSensed(Direction direction);

    #ifdef SERVO_PID_AUTOTUNE_PRESENT
      // start the PID auto-tune of the slewing gain set (param4-6)
      // \param testDistance: measured move distance in "measures" (degrees,) or NAN for the config default
      CommandError autoTuneStart(float testDistance);

      // abort a running PID auto-tune, restoring the pre-tune gains
      void autoTuneAbort();

      // apply the staged auto-tune result live and persist it to NV
      CommandError autoTuneApply();

      // true while an auto-tune run is active
      inline bool autoTuneActive() { return autoTuneState != AT_IDLE && autoTuneState != AT_DONE_SUCCESS && autoTuneState != AT_DONE_FAIL; }
    #endif

    // calibrate the associated motor
    void calibrate(float value) { motor->calibrate(value); }

    // calibrate the associated motor's driver
    void calibrateDriver() { motor->calibrateDriver(); }

    // monitor movement
    void poll();

    // associated motor
    Motor *motor;

    AxisStoredSettings settings;

    // hint that min and max pins are the same (no directional limit indication)
    bool commonMinMaxSense = false;

  private:
    // set frequency in "measures" (degrees, microns, etc.) per second (0 stops motion)
    void setFrequency(float frequency);

    // sets driver power on/off
    void powered(bool value);

    // distance to origin or target, whichever is closer, in "measures" (degrees, microns, etc.)
    double getOriginOrTargetDistance();

    // returns true if traveling through backlash
    bool inBacklash();

    // convert from unwrapped (full range) to normal (+/- wrapAmount) coordinate
    double wrap(double value);

    // convert from normal (+/- wrapAmount) to an unwrapped (full range) coordinate
    double unwrap(double value);

    // convert from normal (+/- wrapAmount) to an unwrapped (full range) coordinate
    // nearest the instrument coordinate
    double unwrapNearest(double value);

    bool decodeAxisSettings(char *s, AxisStoredSettings &a);

    bool validateAxisSettings(int axisNum, AxisStoredSettings a);

    #ifdef SERVO_PID_AUTOTUNE_PRESENT
      // advance the auto-tune state machine (called from poll() while active)
      void autoTunePoll();

      // apply the candidate gains: setParameters() then an enable toggle so
      // Pid::reset() reloads param4-6 into SetTunings() and clears windup
      void autoTuneSetCandidate();

      // end the run: stop motion, release the slewing-set hold, restore pre-tune gains
      void autoTuneFinish(AutoTuneResult result);

      // handle a servo safety shutdown or driver fault; returns true if one occurred
      bool autoTuneSafetyEvent();

      // robust aggregate of a round's samples (median + modified Z-score outlier filter)
      float autoTuneAggregate(float values[], int count);

      AutoTuneState autoTuneState = AT_IDLE;
      AutoTuneResult autoTuneResult = ATR_NONE;
      uint8_t autoTunePhase = 0;                    // sub-phase within a state
      uint8_t autoTuneIteration = 0;                // correction rounds completed
      uint8_t autoTuneRepeat = 0;                   // measured moves completed this round
      uint8_t autoTuneShutdownCount = 0;            // servo safety shutdowns this run
      uint8_t autoTuneHuntingCount = 0;             // hunting-flagged repeats this round
      int8_t autoTuneDirection = 1;                 // direction of the current test move
      bool autoTuneHoldSlewing = false;             // holding motor->setSlewing(true) during monitor
      bool autoTuneStagedValid = false;             // autoTuneStaged[] holds a result awaiting :SXT,2

      long autoTuneSettleBandSteps = 2;             // settle band in steps (counts)
      long autoTuneMaxOvershootSteps = 4;           // overshoot acceptance in steps (counts)
      float autoTuneTestDistance = 0.0F;            // measured move distance in measures
      float autoTunePreloadDistance = 0.0F;         // backlash take-up nudge distance in measures
      float autoTuneRate = NAN;                     // test slew rate in measures/s (NAN = production rate)
      unsigned long autoTuneMoveTimeout = 0;        // per-move watchdog period in milliseconds

      // physical maximum rotation rate measurement
      float autoTuneSpeedDistance = 0.0F;           // speed test move distance in measures
      float autoTuneSpeedRate = 0.0F;               // commanded (over-speed) rate in measures/s
      float autoTuneMaxFreqStore = 0.0F;            // maxFreq before the speed test ceiling lift
      float autoTuneSlewFreqStore = 0.0F;           // slewFreq before the speed test
      bool autoTuneMaxFreqOverride = false;         // ceiling lift active (restore on any exit)
      bool autoTuneSpeedSaturated = false;          // drive output saturated during the speed test
      float autoTuneMeasuredMaxRate = 0.0F;         // measured maximum rate in measures/s (deg/s)
      float autoTunePeakRate = 0.0F;                // peak smoothed rate regardless of saturation
      float autoTuneVelocityEstimate = 0.0F;        // smoothed velocity estimate in measures/s
      long autoTuneLastPositionSteps = 0;           // last sampled position for velocity estimation
      unsigned long autoTuneLastSampleTime = 0;     // last velocity sample time in milliseconds

      // per-repeat measurement working state
      long autoTunePeakOvershoot = 0;               // peak excursion beyond target in steps
      long autoTuneStableDistance = 0;              // target distance when stability tracking last reset
      unsigned long autoTuneMoveStartTime = 0;      // measured move start (backlash excluded) in milliseconds
      unsigned long autoTuneWatchdogTime = 0;       // watchdog deadline in milliseconds
      unsigned long autoTuneStableStartTime = 0;    // position stable since, in milliseconds
      unsigned long autoTuneDwellTime = 0;          // post-move dwell deadline in milliseconds
      unsigned long autoTuneArriveTime = 0;         // goto completion time in milliseconds (0 = still slewing)
      bool autoTuneMeasuring = false;               // true once out of backlash take-up
      bool autoTuneInBand = false;                  // currently within the settle band
      bool autoTuneEverInBand = false;              // has entered the settle band
      bool autoTuneHunting = false;                 // this repeat classified as hunting
      uint8_t autoTuneBandCrossings = 0;            // settle band boundary crossings

      // round sample arrays
      float autoTuneOvershoot[PID_AUTOTUNE_REPEATS];
      float autoTuneSettleTime[PID_AUTOTUNE_REPEATS];
      float autoTuneResidual[PID_AUTOTUNE_REPEATS];

      // round aggregates (also reported by :GXT)
      float autoTuneOvershootResult = 0.0F;
      float autoTuneSettleTimeResult = 0.0F;
      float autoTuneResidualResult = 0.0F;

      // gain sets: original (restore,) candidate (testing,) staged (result,) best (fallback)
      float autoTuneOriginal[3] = {0, 0, 0};
      float autoTuneCandidate[3] = {0, 0, 0};
      float autoTuneStaged[3] = {0, 0, 0};
      float autoTuneBest[3] = {0, 0, 0};
      float autoTuneBestScore = 0.0F;
    #endif
    
    AxisErrors errors;
    bool lastErrorResult = false;

    uint8_t axisNumber = 0;
    char axisPrefix[9] = " Axis_, ";

    char unitsStr[5] = "?";
    bool unitsRadians = false;

    bool enabled = false;                // enabled or disabled axis and motor power
    bool limitsCheck = true;             // enabled or disabled software range limits check

    uint8_t homeSenseHandle = 0;         // home sensor handle
    uint8_t minSenseHandle = 0;          // min sensor handle
    uint8_t maxSenseHandle = 0;          // max sensor handle

    #if DEBUG == VERBOSE
      bool lastSenseMin = false;
      bool lastSenseMax = false;
      bool lastSenseHome = false;
    #endif

    uint16_t backlashStepsStore;
    volatile uint16_t backlashSteps = 0;
    volatile uint16_t backlashAmountSteps = 0;

    volatile bool takeStep = false;

    // coordinate handling
    double wrapAmount = 0.0L;
    bool wrapEnabled = false;

    // power down standstill control
    bool powerDownStandstill = false;
    bool powerDownOverride = false;
    bool poweredDown = false;
    unsigned long powerDownDelay;        // in milliseconds
    unsigned long powerDownOverrideEnds; // in milliseconds
    unsigned long powerDownTime = 0;     // in milliseconds

    // timeout for home switch detection
    unsigned long homeTimeoutTime = 0;   // in milliseconds

    // rates (in measures per second) to control motor movement
    float freq = 0.0F;                   // momentary frequency in measures/s
    float baseFreq = 0.0F;               // base frequency for continuous movement in measures/s
    float minFreq = 0.0F;                // minimum frequency in measures/s
    float slewFreq = 0.0F;               // slewing frequency in measures/s
    float maxFreq = 0.0F;                // maximum frequency in measures/s
    float backlashFreq = 0.0F;           // in measures/s

    float scaleFreq = 1.0F;

    float targetTolerance = 0.0F;

    AutoRate autoRate = AR_NONE;         // auto slew mode
    float slewAccelRateFs;               // auto slew rate in measures per second per frac-sec
    float abortAccelRateFs;              // abort slew rate in measures per second per frac-sec
    float slewAccelTime = NAN;           // auto slew acceleration time in seconds
    float abortAccelTime = NAN;          // abort slew acceleration time in seconds

    HomingStage homingStage = HOME_NONE;

    const AxisPins *pins;

    void (*volatile callback)() = NULL;
};

#endif
