// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static final boolean increasedPathfindingSpeed = false;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // constants used for the elevator
  public static final class Elevator {

    public static final int MOTOR_LEFT_ID = 21;
    public static final int MOTOR_RIGHT_ID = 22;
    public static final String CANBUS = "rio";

    public static final double MAX_HEIGHT =
        45.135; // old height 43.66 ABSOLUTELY NEVER ACHIEVE THIS OR EVERYTHING EXPLODES
    public static final double MIN_HEIGHT = 0;
    public static final double INTAKE_HEIGHT = 0;
    // public static
    final double STAGE_1_LEVEL = 0; // currently unimplemented
    public static final double INTAKE_LEVEL_1 = 2.57;
    public static final double INTAKE_LEVEL_2 = 13.29;
    public static final double BARGE_LEVEL = 44.5; // currently impossible to achieve
  }

  // constants used for the arm/wrist (naming inconsistency)
  public static final class Arm {
    public static final int ROLLER_MOTOR_ID = 23;
    public static final int ARM_MOTOR_ID = 24;
    public static final int CANRANGE_ID = 60;
    public static final String CANBUS = "rio";

    public static final double REEF_INTAKE_ANGLE = 1.53;
    public static final double GROUND_INTAKE_ANGLE =
        5.212; // we do not know this yet; input correct angle later
    public static final double SCORING_ANGLE = 0.608; // currently impossible to achieve
    public static final double WRIST_HIGHEST_ANGLE =
        6.344; // set highest angle to wrist ground angle plus arbitrary leeway
    public static final double WRIST_CORAL_SCORE = 1.259;

    public static final double ELEVATOR_CLEARANCE_ANGLE = SCORING_ANGLE;
    public static final double PROCESSOR_ANGLE =
        GROUND_INTAKE_ANGLE; // !!!!! find out through empirical testing !!!!!

    public static final double ROLLER_HOLDING_POWER = 0.05;
  }
}
