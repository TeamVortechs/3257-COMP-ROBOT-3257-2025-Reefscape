package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for arm Module IO implementations. This abstracts all hardware interactions for the
 * arm.
 */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    double rollerAmps = 0;
    double rollerVolts = 0;
    double rollerSpeed = 0;

    double canRangeDistance = 0;
    double targetSpeed = 0;
  }

  // updates the given inputs with new values(advantage kit stuff)
  public default void updateInputs(IntakeIOInputsAutoLogged inputsAutoLogged) {}

  // getters for motors

  // gets the height of the arm in meters
  public default double getCurrent() {
    return 0;
  }

  public default double getVoltage() {
    return 0;
  }

  public default double getRollerSpeed() {
    return 0;
  }

  public default double getTargetSpeed() {
    return 0;
  }

  // setters for motors

  // sets the target in rotations per second that the roller pid tries to go to
  public default void setRollerSpeedTarget(double speedRPS) {}

  public default void setVoltage(double volt) {}

  // canrange functions

  // sets the distance from the can range. This is only used in simulation for system testing and
  // doesn't work on real ios
  public default void setCanrangeDistance(double dist) {}

  // gets the distance of the can rnage
  public default double getDistance() {
    return 0;
  }

  // misc methods

  // rebuilds the pid constants of the motors
  public default void rebuildMotorsPID() {}

  /** Stops the motor immediately */
  default void stop() {}
  ;

  /** returns true if either motor has exceeded 40 amps of torque current */
  default boolean checkIfStalled() {
    return false;
  }

  public default void resetEncoders() {}

  public default void setBraked(boolean braked) {}
}
