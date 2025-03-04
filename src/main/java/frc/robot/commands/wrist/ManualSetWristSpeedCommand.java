package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.DoubleSupplier;

/*
Names
brief description
 */
public class ManualSetWristSpeedCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private DoubleSupplier supplierspeed;

  private Wrist wrist;

  private double speed;

  public ManualSetWristSpeedCommand(Wrist wrist, DoubleSupplier supplierspeed) {
    this.wrist = wrist;
    this.supplierspeed = supplierspeed;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setManualSpeed(supplierspeed.getAsDouble());
    System.out.println("Manual Mode is in session");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setManualSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
