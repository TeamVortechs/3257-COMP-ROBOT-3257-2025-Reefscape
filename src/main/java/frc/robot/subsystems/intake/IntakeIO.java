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

    double targetSpeed = 0;

    double position = 0;
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

  public default double getRotations() {
    return 0;
  }

  // setters for motors

  // sets the target in rotations per second that the roller pid tries to go to
  public default void setRollerSpeedTarget(double speedRPS) {}

  public default void setVoltage(double volt) {}

  //sets the position of the rollers. This function will most likely not be implemented
  public default void setPosition(double position) {}

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
