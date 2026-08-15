// -----------------------------------------------------------------------------------------------------------------
// Observatory dome commands

#include "Dome.h"

#ifdef DOME_PRESENT

#include "../roof/Roof.h"
#include "../../lib/nv/Nv.h"

#ifdef SERVO_PID_AUTOTUNE_PRESENT
  // persist an auto-tune backlash measurement as this axis' backlash compensation
  void Dome::applyMeasuredBacklash(int axisNum, float measured) {
    if (isnan(measured) || measured <= 0.0F) {
      VF("MSG: Dome, no backlash measurement to apply for axis"); VL(axisNum);
      return;
    }

    // Dome::init() rejects anything outside 0..10 degrees and silently zeroes it, so clamp here
    // rather than write a value that would be thrown away (with an error) on the next boot
    if (measured > 10.0F) {
      DF("WRN: Dome, measured backlash "); D(measured); DLF(" deg exceeds the 10 deg limit, clamping");
      measured = 10.0F;
    }

    if (axisNum == 1) {
      settings.backlash.azimuth = measured;
      axis1.setBacklash(measured);
    }
    #if AXIS2_DRIVER_MODEL != OFF
      else if (axisNum == 2) {
        settings.backlash.altitude = measured;
        axis2.setBacklash(measured);
      }
    #endif
    else return;

    nv.updateBytes(NV_DOME_SETTINGS_BASE, &settings, sizeof(DomeSettings));
    VF("MSG: Dome, axis"); V(axisNum); VF(" backlash compensation set to "); V(measured);
    VLF(" deg and saved to NV (survives a reboot, unlike the PID gains)");
  }
#endif

bool Dome::command(char reply[], char command[], char parameter[], bool *supressFrame, bool *numericReply, CommandError *commandError) {

	if (command[0] == 'D') {
    // :DC#  Dome Return Home
    //         Returns: nothing
    if (command[1] == 'C' && parameter[0] == 0) {
      dome.findHome();
      *numericReply = false;
    } else

    // :DF#  Dome Reset Home
    //         Returns: nothing
    if (command[1] == 'F' && parameter[0] == 0) {
      dome.reset();
      *numericReply = false;
    } else

    // :DH#  Dome Halt motion
    //         Returns: nothing
    if (command[1] == 'H' && parameter[0] == 0) {
      dome.stop();
      *numericReply = false;
    } else

    // :DP#  Dome goto park position
    //            Return: 0 on failure
    //                    1 on success
    if (command[1] == 'P' && parameter[0] == 0) {
      CommandError e = dome.park();
      if (e == CE_NONE) *commandError = CE_1; else { VF("MSG: Dome, park error "); VL(e); *commandError = e; }
    } else

    // :DQ#  Dome set park position
    //            Return: 0 on failure
    //                    1 on success
    if (command[1] == 'Q' && parameter[0] == 0) {
      CommandError e = dome.setpark();
      if (e == CE_NONE) *commandError = CE_1; else { VF("MSG: Dome, set park error "); VL(e); *commandError = e; }
    } else

    // :DR#  Dome restore park position
    //            Return: 0 on failure
    //                    1 on success
    if (command[1] == 'R' && parameter[0] == 0) {
      CommandError e = dome.unpark();
      if (e == CE_NONE) *commandError = CE_1; else { VF("MSG: Dome, unpark error "); VL(e); *commandError = e; }
    } else

    // :DZ#  Dome Get Azimuth (0 to 360 degrees)
    //         Returns: D.D
    if (command[1] == 'Z' && parameter[0] == 0) {
      sprintF(reply, "%0.3f", dome.getAzimuth());
      *numericReply = false;
    } else

    // :Dz[D.D]#  Dome Set Azimuth target (0 to 360 degrees)
    //         Returns: nothing
    if (command[1] == 'z') {
      float azimuth = atof(parameter);
      dome.setTargetAzimuth(azimuth);
      *numericReply = false;
    } else

    #if AXIS2_DRIVER_MODEL != OFF
      // :DA#  Dome Get Altitude (0 to 90 degrees)
      //         Returns: D.D
      if (command[1] == 'A' && parameter[0] == 0) {
        sprintF(reply, "%0.3f", dome.getAltitude());
        *numericReply = false;
      } else

      // :Da[D.D]#  Dome Set Altitude target (0 to 90 degrees)
      //         Returns: nothing
      if (command[1] == 'a') {
        float altitude = atof(parameter);
        dome.setTargetAltitude(altitude);
        *numericReply = false;
      } else
    #endif

    // :DN#       Dome Sync Target (Azimuth only)
    //            Returns:
    //              See :DS# command
    if (command[1] == 'N' && parameter[0] == 0) {
      CommandError e = dome.syncAzimuthTarget();
      strcpy(reply,"0");
      if (e >= CE_SLEW_ERR_BELOW_HORIZON && e <= CE_SLEW_ERR_UNSPECIFIED) reply[0] = (char)(e - CE_SLEW_ERR_BELOW_HORIZON) + '1';
      if (e == CE_NONE) reply[0] = '0';
      *numericReply = false;
      *supressFrame = true;
      *commandError = e;
    } else

    // :DS#       Dome Goto Target
    //            Returns:
    //              0=Goto is possible
    //              1=below the horizon limit
    //              2=above overhead limit
    //              3=controller in standby
    //              4=dome is parked
    //              5=Goto in progress
    //              6=outside limits (AXIS2_LIMIT_MAX, AXIS2_LIMIT_MIN, AXIS1_LIMIT_MIN/MAX, MERIDIAN_E/W)
    //              7=hardware fault
    //              8=already in motion
    //              9=unspecified error
    if (command[1] == 'S' && parameter[0] == 0) {
      D("Dome Goto =");
      CommandError e = dome.gotoAzimuthTarget();
      DL(e);
      #if AXIS2_DRIVER_MODEL != OFF
        if (e == CE_NONE) e = dome.gotoAltitudeTarget();
      #endif
      strcpy(reply,"0");
      if (e >= CE_SLEW_ERR_BELOW_HORIZON && e <= CE_SLEW_ERR_UNSPECIFIED) reply[0] = (char)(e - CE_SLEW_ERR_BELOW_HORIZON) + '1';
      if (e == CE_NONE) reply[0] = '0';
      *numericReply = false;
      *supressFrame = true;
      *commandError = e;
    } else

    //  :DU#  Get Dome Status
    //         Returns: 'P' if parked, 'K' if parking, 'S' if slewing, 'H' if at Home, 'I' if idle
    if (command[1] == 'U' && parameter[0] == 0) {
      if (dome.isParked()) reply[0] = 'P'; else
      if (settings.park.state == PS_PARKING) reply[0] = 'K'; else
      if (dome.isSlewing()) reply[0] = 'S'; else
      #if AXIS2_DRIVER_MODEL != OFF
        if (dome.getAzimuth() == 0.0F && dome.getAltitude() == 0.0F) reply[0] = 'H'; else
      #else
        if (dome.getAzimuth() == 0.0F) reply[0] = 'H'; else
      #endif
      reply[0] = 'I';
      reply[1] = 0;
      *numericReply = false;
    } else *commandError = CE_CMD_UNKNOWN;
	} else {

    #ifdef SERVO_PID_AUTOTUNE_PRESENT
      // dome-level safety gate for the PID auto-tune start form (:SXT[n],1...#) only;
      // the auto-tune drives axis autoGoto() below the checks a normal :DS# performs,
      // so parked state and the shutter-lock interlock must be enforced here.
      // abort (:SXT[n],0#,) apply (:SXT[n],2#) and status (:GXT[n]#) always pass through
      if (command[0] == 'S' && command[1] == 'X' && parameter[0] == 'T' &&
          parameter[2] == ',' && parameter[3] == '1') {
        #if defined(ROOF_PRESENT) && DOME_SHUTTER_LOCK == ON
          if (!roof.open()) { *commandError = CE_SLEW_ERR_IN_STANDBY; return true; }
        #endif
        if (settings.park.state >= PS_PARKED) { *commandError = CE_SLEW_ERR_IN_PARK; return true; }
      }

      // the apply form (:SXT[n],2#) also commits the run's backlash measurement, which the axis
      // layer cannot do: the value lives in the dome's NV settings
      bool autoTuneApplyForm = command[0] == 'S' && command[1] == 'X' && parameter[0] == 'T' &&
                               parameter[2] == ',' && parameter[3] == '2' && parameter[4] == 0;
    #endif

    // give the axes a shot at otherwise unhandled commands (:GXA/:SXA/:GXS/:GXU/:SXT/:GXT)
    if (axis1.command(reply, command, parameter, supressFrame, numericReply, commandError)) {
      #ifdef SERVO_PID_AUTOTUNE_PRESENT
        if (autoTuneApplyForm && *commandError == CE_NONE) applyMeasuredBacklash(1, axis1.getAutoTuneBacklash());
      #endif
      return true;
    }
    #if AXIS2_DRIVER_MODEL != OFF
      if (axis2.command(reply, command, parameter, supressFrame, numericReply, commandError)) {
        #ifdef SERVO_PID_AUTOTUNE_PRESENT
          if (autoTuneApplyForm && *commandError == CE_NONE) applyMeasuredBacklash(2, axis2.getAutoTuneBacklash());
        #endif
        return true;
      }
    #endif
    return false;
  }

  return true;
}

#endif
