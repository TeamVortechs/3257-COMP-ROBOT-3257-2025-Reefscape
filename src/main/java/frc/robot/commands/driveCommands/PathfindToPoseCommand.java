package frc.robot.commands.driveCommands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/*
Names
brief description
 */
public class PathfindToPoseCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private Drive drive;

  @AutoLogOutput
  private Supplier<Pose2d> targetPoseSupplier;

  public PathfindToPoseCommand(Drive drive, Supplier<Pose2d> targetPose) {
    addRequirements(drive);
    this.drive = drive;

    this.targetPoseSupplier = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = targetPoseSupplier.get();

    double translationDelta = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    

    

    drive.runVelocity(new ChassisSpeeds(-1, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
