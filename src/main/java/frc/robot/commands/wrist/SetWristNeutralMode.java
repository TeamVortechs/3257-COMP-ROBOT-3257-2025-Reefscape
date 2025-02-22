package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;

/*
Names
brief description
 */
public class SetWristNeutralMode extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private boolean mode;

  private Wrist wrist;

  public SetWristNeutralMode(boolean mode, Wrist wrist) {
    this.mode = mode;
    this.wrist = wrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setBraked(mode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
