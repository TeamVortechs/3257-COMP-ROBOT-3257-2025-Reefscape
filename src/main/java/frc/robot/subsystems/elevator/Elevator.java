package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Elevator subsystem responsible for controlling the lifting mechanism. Uses PID control for
 * precise movement and prevents unsafe operation via limit switches and software constraints.
 */
public class Elevator extends SubsystemBase {

  // for advantage kti logging
  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // for flexible modules as well advantage kit
  private final ElevatorIO moduleIO;

  // arm for clearance checks
  private Arm arm;

  // these are for advantage kit state logging and/or for keeping track of key variables
  @AutoLogOutput private double currentHeight = 0.0;
  @AutoLogOutput private double targetHeight = 0.0;
  @AutoLogOutput private boolean isOnTarget = false;
  @AutoLogOutput private boolean manualOverride = false;

  // these constants should be moved to another class
  private double tolerance = 0.1;
  private double manManualSpeed = 0.1;
  private double minHeight = 0;
  private double maxHeight = 5;

  /**
   * Constructor for the Elevator subsystem.
   *
   * @param moduleIO Hardware interface for elevator motors.
   * @param homeSwitch Digital input limit switch for homing.
   */
  public Elevator(ElevatorIO moduleIO, Arm arm) {
    this.arm = arm;

    this.moduleIO = moduleIO;

    currentHeight = moduleIO.getHeight();
    targetHeight = moduleIO.getHeight();
    setTargetHeight(currentHeight);
  }

  @Override
  public void periodic() {
    moduleIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // check to see if the module is stalling; if so, then stop the motors and cancel the next
    // movement

    if (moduleIO.checkIfStalled()) {
      System.out.println("MODULE IO HAS STALLED ");
      moduleIO.stop();
      return;
    }

    currentHeight = moduleIO.getHeight();

    // if manual override do a quick bounds check then retuorn
    if (manualOverride) {

      if (getCurrentHeight() < minHeight - maxHeight || getCurrentHeight() > maxHeight) {
        System.out.println("MODULE IO OUT OF BOUDNS");
        setManualSpeed(0);
      }
      return;
    }

    isOnTarget = isOnTarget();

    // Clamp target height to prevent exceeding limits
    targetHeight = Math.max(0.0, Math.min(targetHeight, maxHeight));

    moduleIO.PIDVoltage(targetHeight);
  }

  /** Sets a new target height for the elevator using PID control. */
  public void setTargetHeight(double height) {

    if (!arm.isClearFromElevator()) {
      System.out.println("ARM IS NOT CLEAR FROM ELEVATOR");
      return;
    }

    manualOverride = false;

    targetHeight = Math.max(0.0, Math.min(height, maxHeight));
  }

  /** Allows manual control of the elevator, bypassing PID. */
  public void setManualSpeed(double speed) {
    manualOverride = true;

    if (Math.abs(speed) > tolerance) speed = Math.copySign(manManualSpeed, speed);
    System.out.println("Above speed limit; rate limiting MODULE IO speed.");
    moduleIO.setSpeed(speed);
  }

  /** Holds the current position using PID control. */
  public void holdPositionPID() {
    manualOverride = false;
    if (Math.abs(targetHeight - currentHeight) > tolerance) {
      targetHeight = currentHeight;
      moduleIO.PIDVoltage(targetHeight);
    }
  }

  /** Holds the current position using braking mode. */
  public void holdPositionBrake() {
    manualOverride = true;
    moduleIO.stop();
  }

  /** Emergency stop function that immediately disables motor output. */
  public void emergencyStop() {
    moduleIO.stop();
    manualOverride = true;
  }

  // returns wether or not the elevator is close to the floor
  public boolean isOnFloor() {
    return getCurrentHeight() < 0.1;
  }

  // gest the current height of the elevator motor
  public double getCurrentHeight() {
    return currentHeight;
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  // returns wether or not the elevaotr is on target
  public boolean isOnTarget() {
    return (Math.abs(currentHeight - targetHeight) < tolerance);
  }

  /** resets encoders to read 0 and resets PID (setting it to begin at current height) */
  public void resetEncoders() {
    moduleIO.resetEncoders();
  }

  // gets the current of the subsystem
  public double getCurrent() {
    return moduleIO.getCurrent();
  }

  // rebuilds the PID of the motors
  public void rebuildMotorsPID() {
    moduleIO.rebuildMotorsPID();
  }

  // commands

  // sets the target height of the subsystem. Ends immediately
  public Command setTargetHeightCommand(double targetHeight) {
    return new InstantCommand(() -> this.setTargetHeight(targetHeight), this);
  }

  // sets the target height of the subsystem. Ends when the subsystem reaches this height
  public Command setTargetHeightCommandConsistentEnd(double targetHeight) {
    return new InstantCommand(() -> this.setTargetHeight(targetHeight), this)
        .andThen(new WaitUntilCommand(() -> this.isOnTarget));
  }

  // sets the manual override speed of this command. Uses a regular double
  public Command setManualOverrideCommand(double speed) {
    return new RunCommand(() -> this.setManualSpeed(speed), this);
  }

  // sets the manual override speed of this command. Uses a double supplier
  public Command setManualOverrideCommand(DoubleSupplier speed) {
    return new RunCommand(() -> this.setManualSpeed(speed.getAsDouble()), this);
  }

  // makes the PID hold position. If true it'll use brake if false it'll pid.
  public Command holdPositionCommand(boolean brake) {
    if (brake) return new InstantCommand(() -> this.holdPositionBrake(), this);

    return new InstantCommand(() -> this.holdPositionPID(), this);
  }

  // resets the encoders of the wrist
  public Command resetEncodersCommand() {
    return new InstantCommand(() -> this.resetEncoders());
  }

  // simple command incase you need a chain that requires this subsystem
  public Command requireSubsystemCommand() {
    return new InstantCommand(null, this);
  }

  // rebuilds the pid constants for testing
  public Command rebuildMotorsPIDCommand() {
    return new InstantCommand(() -> this.rebuildMotorsPID());
  }
}
