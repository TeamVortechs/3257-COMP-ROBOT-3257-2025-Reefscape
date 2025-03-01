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

    public static final double MAX_HEIGHT = 41.149; // 41.149 is new highest
    public static final double MIN_HEIGHT = 0;
    public static final double INTAKE_HEIGHT = 0;
    // public static final double STAGE_1_LEVEL = 0; // currently unimplemented
    public static final double STAGE_2_LEVEL = 10.81066;
    public static final double STAGE_3_LEVEL = 19.5383;
    public static final double BARGE_LEVEL = 41; // currently impossible to achieve
  }

  // constants used for the arm/wrist (naming inconsistency)
  public static final class Arm {
    public static final int ROLLER_MOTOR_ID = 23;
    public static final int ARM_MOTOR_ID = 24;
    public static final int CANRANGE_ID = 60;
    public static final String CANBUS = "rio";

    public static final double WRIST_STAGE_2_ANGLE = 2.251;
    public static final double WRIST_GROUND_ANGLE =
        5.441; // we do not know this yet; input correct angle later
    public static final double WRIST_STAGE_4_ANGLE = 1.3007; // currently impossible to achieve
    public static final double WRIST_HIGHEST_ANGLE =
        WRIST_GROUND_ANGLE + 0.2; // set highest angle to wrist ground angle plus arbitrary leeway
  }
}
