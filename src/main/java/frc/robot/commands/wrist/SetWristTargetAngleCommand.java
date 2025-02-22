package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;

/*
Names
brief description
 */
public class SetWristTargetAngleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private final Wrist wrist;

  private final double targetAngle;

  public SetWristTargetAngleCommand(Wrist wrist, double targetAngle) {
    addRequirements(wrist);

    this.wrist = wrist;
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setTargetAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.isOnTarget() && Math.abs(targetAngle - wrist.getTargetAngle()) < 0.001;
  }
}
