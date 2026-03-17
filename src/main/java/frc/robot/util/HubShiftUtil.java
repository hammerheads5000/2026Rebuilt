// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.TurretConstants.ACTIVE_PRESHOOT_TIME;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;

public class HubShiftUtil {
    public enum ShiftEnum {
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        AUTO,
        DISABLED;
    }

    public record ShiftInfo(ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

    private static Timer shiftTimer = new Timer();
    private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

    private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    private static final double approachingActiveFudge = -ACTIVE_PRESHOOT_TIME.in(Seconds);

    public static final double autoEndTime = 20.0;
    public static final double teleopDuration = 140.0;
    private static final boolean[] activeSchedule = {true, true, false, true, false, true};
    private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

    private static BooleanSupplier alwaysActiveOverride = () -> false;

    public static boolean getAlwaysActiveOverride() {
        return alwaysActiveOverride.getAsBoolean();
    }

    public static void setAlwaysActiveOverride(BooleanSupplier allianceWinOverride) {
        HubShiftUtil.alwaysActiveOverride = allianceWinOverride;
    }

    public static Alliance getFirstActiveAlliance() {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Return FMS value
        String message = DriverStation.getGameSpecificMessage();
        if (message.length() > 0) {
            char character = message.charAt(0);
            if (character == 'R') {
                return Alliance.Blue;
            } else if (character == 'B') {
                return Alliance.Red;
            }
        }

        // Return default value
        return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    /** Starts the timer at the begining of teleop. */
    public static void initialize() {
        shiftTimer.restart();
    }

    private static boolean[] getSchedule(boolean override) {
        boolean[] currentSchedule;
        Alliance startAlliance = getFirstActiveAlliance();
        boolean activeFirst = startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue);
        if (override) {
            activeFirst = !activeFirst;
        }
        currentSchedule = activeFirst ? activeSchedule : inactiveSchedule;
        return currentSchedule;
    }

    private static boolean[] getSchedule() {
        return getSchedule(false);
    }

    private static ShiftInfo getShiftInfo(
            boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes, boolean override) {
        double currentTime = shiftTimer.get();
        double stateTimeElapsed = shiftTimer.get();
        double stateTimeRemaining = 0.0;
        boolean active = false;
        ShiftEnum currentShift = ShiftEnum.DISABLED;

        if (DriverStation.isAutonomousEnabled()) {
            stateTimeElapsed = currentTime;
            stateTimeRemaining = autoEndTime - currentTime;
            active = true;
            currentShift = ShiftEnum.AUTO;
        } else if (DriverStation.isEnabled()) {
            int currentShiftIndex = -1;
            for (int i = 0; i < shiftStartTimes.length; i++) {
                if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
                    currentShiftIndex = i;
                    break;
                }
            }
            if (currentShiftIndex < 0) {
                // After last shift, so assume endgame
                currentShiftIndex = shiftStartTimes.length - 1;
            }

            // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
            stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
            stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

            // If the state is the same as the last shift, combine the elapsed time
            if (currentShiftIndex > 0) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
                    stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
                }
            }

            // If the state is the same as the next shift, combine the remaining time
            if (currentShiftIndex < shiftEndTimes.length - 1) {
                if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
                    stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
                }
            }

            active = currentSchedule[currentShiftIndex];
            currentShift = shiftsEnums[currentShiftIndex];
        }
        ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active || override);
        return shiftInfo;
    }

    public static ShiftInfo getOfficialShiftInfo(boolean override) {
        return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes, override);
    }

    public static ShiftInfo getOfficialShiftInfo() {
        return getOfficialShiftInfo(alwaysActiveOverride.getAsBoolean());
    }

    public static ShiftInfo getShiftedShiftInfo(boolean override) {
        boolean[] shiftSchedule = getSchedule(override);
        // Starting active
        if (shiftSchedule[1] == true) {
            double[] shiftedShiftStartTimes = {
                0.0, 10.0, 35.0, 60.0 + approachingActiveFudge, 85.0, 110.0 + approachingActiveFudge
            };
            double[] shiftedShiftEndTimes = {
                10.0, 35.0, 60.0 + approachingActiveFudge, 85.0, 110.0 + approachingActiveFudge, 140.0
            };
            return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes, override);
        }
        double[] shiftedShiftStartTimes = {
            0.0, 10.0, 35.0 + approachingActiveFudge, 60.0, 85.0 + approachingActiveFudge, 110.0
        };
        double[] shiftedShiftEndTimes = {
            10.0, 35.0 + approachingActiveFudge, 60.0, 85.0 + approachingActiveFudge, 110.0, 140.0
        };
        return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes, override);
    }

    public static ShiftInfo getShiftedShiftInfo() {
        return getShiftedShiftInfo(alwaysActiveOverride.getAsBoolean());
    }
}
