package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevatorPresetCommand extends Command {
  private final Elevator elevator;
  private final double presetHeight;

  private boolean ran = false;

  /**
   * @param elevator The elevator subsystem.
   * @param presetHeight The target height (from preferences).
   */
  public SetElevatorPresetCommand(Elevator elevator, double presetHeight) {
    this.elevator = elevator;
    this.presetHeight = presetHeight;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    System.out.println("attempting an elevator preset command with the height " + presetHeight);
    ran = false;
  }

  @Override
  public void execute() {
    if (ran) {
      // System.out.println("Returning from SetElevatorPresetCommand.");
      return;
    }
    // System.out.println("Set Preset Height");
    elevator.setTargetHeight(presetHeight);
    ran = true;
  }

  @Override
  public boolean isFinished() {
    // Finish when the elevator is within tolerance of the target.
    // return elevator.isOnTarget();

    return true;
  }

  @Override
  public void end(boolean interrupted) {
    // Once finished, hold the position.
    System.out.println("I AM DONE!!!! says setelevatorpresetcommand");
    // elevator.holdPositionPID();
  }

  public static Command withEndCondition(Elevator elevator, double presetHeight) {
    return new SetElevatorPresetCommand(elevator, presetHeight)
        .andThen(new WaitUntilCommand(() -> elevator.isOnTarget()));
  }
}
