package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    double CurrentMotorSpeed = 0;
    double CurrentMotorVoltage = 0;
    double CurrentMotorAmps = 0;
  }

  public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

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
