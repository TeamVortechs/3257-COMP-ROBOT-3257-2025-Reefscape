package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class SetElevatorPresetCommand extends Command {
  private final Elevator elevator;
  private final Wrist wrist;
  private final double presetHeight;

  /**
   * @param elevator The elevator subsystem.
   * @param presetHeight The target height (from preferences).
   */
  public SetElevatorPresetCommand(Elevator elevator, Wrist wrist, double presetHeight) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.presetHeight = presetHeight;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    System.out.println("attempting an elevator preset command with the height " + presetHeight);
    elevator.setTargetHeight(presetHeight);
  }

  @Override
  public boolean isFinished() {
    // Finish when the elevator is within tolerance of the target.
    // return elevator.isOnTarget();

    return elevator.isOnTarget();
  }

  @Override
  public void end(boolean interrupted) {
    // Once finished, hold the position.
    System.out.println("I AM DONE!!!! says setelevatorpresetcommand");
    // elevator.holdPositionPID();
  }
}
