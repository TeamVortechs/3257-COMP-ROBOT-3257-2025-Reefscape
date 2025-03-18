package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for Elevator Module IO implementations. This abstracts all hardware interactions for
 * the elevator.
 */
public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    double elevatorMotor1CurrentHeightMeter = 0;
    double elevatorMotor1CurrentSpeedMeter = 0;

    double elevatorMotor1CurrentAmps = 0;
    double elevatorMotor1AppliedVolts = 0;

    double elevatorMotor2CurrentAmps = 0;
    double elevatorMotor2AppliedVolts = 0;

    double elevatorMotor2CurrentSpeedMeter = 0;
    double elevatorMotor2CurrentHeightMeter = 0;

    boolean isStalled;
  }

  // updates the given inputs with new values(advantage kit stuff)
  public default void updateInputs(ArmIOInputsAutoLogged inputsAutoLogged) {}

  // sets the elevator height to the given number
  public default void setSpeed(double Speed) {}

  public default double getVoltage() {
    return 0;
  }

  public default void PIDVoltage(double targetAngle) {}
  ;

  /** Stops the motor immediately */
  default void stop() {}
  ;

  /** returns true if either motor has exceeded 40 amps of torque current */
  default boolean checkIfStalled() {
    return false;
  }
  ;

  // gets the highest possible height of the elevator in radians
  public default double getMaxHeight() {
    return 0.0;
  }

  public default void setVoltage(double volt) {}

  public default void resetEncoders() {}

  public default void setBraked(boolean braked) {}
  // gets the height of the elevator in meters

  public default double getHeight() {
    return 0;
  }

  public default boolean isMaxHeight() {
    return false;
  }

  public default double getCurrent() {
    return 0;
  }

  public default void setRollerSpeed(double speed) {}

  public default double getRollerSpeed() {
    return 0;
  }

  // gets the distance of the can rnage
  public default double getDistance() {
    return 0;
  }

  // rebuilds the pid constants of the motors
  public default void rebuildMotorsPID() {}
}
