package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    double wristLocationRotations = 0.0;
    double wristSpeedRotations = 0.0;

    double wristCurrentAmps = 0.0;
    double wristAppliedVoltage = 0.0;

    double canRangeDistance = 0.0;

    double rollersCurrent = 0.0;
    double rollersEncoder = 0.0;
    double rollersSpeed = 0.0;
  }

  public default boolean isDetected() {
    return false;
  }

  public default double getDistance() {
    return 0.0;
  }

  // updates the given inputs with new values(advantage kit stuff)
  public default void updateInputs(WristIOInputsAutoLogged inputs) {}

  // sets the speed of the wrist to the amount
  public default void setArmSpeed(double speed) {}

  public default void setArmVoltage(double voltage) {}

  public default void setRollerSpeed(double speed) {}

  public default double getRollerSpeed() {
    return 0;
  }

  // stops the wrist at the given angle, locks it
  public default void stopWrist() {}

  public default void PIDVoltage(double targetAngle) {}

  // gets the current of the wrist in radians
  public default double getAngleRotations() {
    return 0;
  }

  public default void setBraked(boolean braked) {}

  // gets the lowest possible angle of the wrist in radians
  public default double getLowestAngleRad() {
    return 0.0;
  }

  // gets the highest possible value of the wrist in radians
  public default double getHighestAngleRad() {
    return 0.0;
  }

  public default void zeroArmEncoder() {}
  ;
}
