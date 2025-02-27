package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;

/*
Names
brief description
 */
public class IntakeWristCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private double speed;

  private Wrist wrist;

  private double lastDetectedTimeStamp = 0;
  private double totalDetectedTime = 0.25;

  public IntakeWristCommand(Wrist wrist, double speed) {
    // addRequirements(null);
    this.wrist = wrist;
    this.speed = speed;

    // addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDetectedTimeStamp = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(lastDetectedTimeStamp != -1) {
      if(Timer.getFPGATimestamp() - lastDetectedTimeStamp > totalDetectedTime) {
        wrist.setRollerSpeed(0);
        return;
      }

      return;
    }

    if (wrist.isCanCloserThan(0.1)) {
      wrist.setRollerSpeed(0);
      return;
    }
    wrist.setRollerSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.isCanCloserThan(0.1);
  }
}
