package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.KDoublePreferences;
import frc.robot.KDoublePreferences.PWrist;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// again I'm not adding stuff to this class while we don't whats gonna go here
public class Wrist extends SubsystemBase {

  private WristIO wristIO;
  private WristIOInputsAutoLogged inputsAutoLogged = new WristIOInputsAutoLogged();

  @AutoLogOutput private double CurrentAngle = 0;

  @AutoLogOutput private double targetAngle = 0;

  @AutoLogOutput private double pidOutput;

  private final double targetBuffer = PWrist.tolerance.getValue();

  @AutoLogOutput private boolean manualOverride = false;

  // this controls the default command of the wrist bc if it has a coral attached then it shouldn't
  // move;
  // this is set in auto commands and on field initialization
  @AutoLogOutput private boolean hasCoral;

  @AutoLogOutput private boolean isOnTarget = false;

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;

    hasCoral = false;
  }

  @Override
  public void periodic() {
    // advantageKit inputs updating
    SmartDashboard.putNumber("Canrange distance", wristIO.getDistance());
    SmartDashboard.putBoolean("Canrange detected", wristIO.isDetected());

    wristIO.updateInputs(inputsAutoLogged);
    Logger.processInputs("Wrist", inputsAutoLogged);

    isOnTarget = isOnTarget();

    CurrentAngle = wristIO.getAngleRotations();

    wristIO.PIDVoltage(targetAngle);
  }

  // returns wether or not the wrist is on target
  public boolean isOnTarget() {
    return Math.abs(targetAngle - CurrentAngle) < KDoublePreferences.PWrist.tolerance.getValue();
  }
  // Math.abs(targetAngle - CurrentAngle) > 0.1 ||

  // sets wether or not the wrist is braked(NOTE: THIS DOES NOT ACTUALLY STOP THE WRIST)
  public void setBraked(boolean braked) {
    wristIO.setBraked(braked);
  }

  // gets the rotation of the arm
  public double getAngleRotations() {
    return wristIO.getAngleRotations();
  }

  // returns wether or not the arm is clear from the elevator
  public boolean isClearFromElevator() {
    return wristIO.getAngleRotations() > Constants.Arm.ELEVATOR_CLEARANCE_ANGLE - 0.1;
  }

  // turns manual override and sets the manual speeed
  public void setManualSpeed(double speed) {
    manualOverride = true;

    if (Math.abs(speed) > PWrist.manualSpeedLimit.getValue())
      speed = Math.copySign(PWrist.manualSpeedLimit.getValue(), speed);
    System.out.println("Above speed limit; rate limiting WRIST speed.");
    wristIO.setArmSpeed(speed);
  }

  // sets the target angle of the PID
  public void setTargetAngle(double angle) {
    manualOverride = false;
    this.targetAngle = angle;
    // System.out.println("setting target angle");
  }

  // gets the angle that the PID is pathing to
  public double getTargetAngle() {
    return targetAngle;
  }

  // sets the roller speed
  public void setRollerSpeed(double speed) {
    wristIO.setRollerSpeed(speed);
  }

  // gets the roller speed
  public double getRollerSpeed() {
    return wristIO.getRollerSpeed();
  }

  // gets the distance of the can Range
  public double getCanDistance() {
    return wristIO.getDistance();
  }

  // resets the encoder of the wrist
  public void resetWristEncoder() {
    wristIO.zeroArmEncoder();
  }

  public void setHasCoral(boolean hasCoral) {
    this.hasCoral = hasCoral;
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  // returns wether or not the canRange is closer than the given distance
  public boolean isCanCloserThan(double distance) {
    return getCanDistance() < distance;
  }

  // enum for each level that the wrist could be
  public enum WristAngle {
    STAGE2_ANGLE(Constants.Arm.REEF_INTAKE_ANGLE),
    INTAKE_ANGLE(0),
    ALGAE_GROUND_INTAKE(Constants.Arm.GROUND_INTAKE_ANGLE),
    CORAL_SCORE(Constants.Arm.WRIST_CORAL_SCORE);

    // STAGE2_ANGLE(Stage2angle),

    private double angle;

    private WristAngle(double angleRot) {
      this.angle = angleRot;
    }

    public double getAngle() {
      return angle;
    }
  }
}
