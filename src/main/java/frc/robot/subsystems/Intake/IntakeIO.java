package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    double CurrentMotorSpeed = 0;
    double CurrentMotorVoltage = 0;
    double CurrentMotorAmps = 0;
  }

  public default void updateInputs(IntakeIOInputsAutoLogged inputs) {}

  public default void setMotorSpeed(double Speed) {}

  public default double getMaxVolts() {
    return 0.0;
  }

  public default double getMaxAmps() {
    return 0.0;
  }

  public default double getMaxSpeed() {
    return 0.0;
  }
}
