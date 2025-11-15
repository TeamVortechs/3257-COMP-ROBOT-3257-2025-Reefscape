package frc.robot.commands.driveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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

  @AutoLogOutput private Supplier<Pose2d> targetPoseSupplier;

  private final PIDController translationController = new PIDController(1, 0, 0);
  private final PIDController thetaController = new PIDController(50, 0, 0);

  private final double translationTolerance = 0.1;
  private final double rotationTolerance = 0.1;

  private final boolean endOnTarget;
  private Consumer<Boolean> onTarget = null;

  private double xVelocity = 0;
  private double yVelocity = 0;
  private double thetaVelocity = 0;

  private double thetaDistance = 0;
  private double translationDistanceX = 0;
  private double translationDistanceY = 0;

  private Timer timer;
  private double timeout = 10;

  public PathfindToPoseCommand(
      Drive drive, Supplier<Pose2d> targetPose, boolean endOnTarget, Consumer<Boolean> onTarget) {
    this.onTarget = onTarget;

    this.endOnTarget = endOnTarget;
    new PathfindToPoseCommand(drive, targetPose, endOnTarget);
  }

  public PathfindToPoseCommand(Drive drive, Supplier<Pose2d> targetPose, boolean endOnTarget) {
    addRequirements(drive);
    this.drive = drive;

    this.targetPoseSupplier = targetPose;

    this.endOnTarget = endOnTarget;

    timer = new Timer();
    timer.reset();

    // record outputs
    Logger.recordOutput("DrivetoPose/PathfindxVelocity", xVelocity);
    Logger.recordOutput("DrivetoPose/PathfindyVelocity", yVelocity);
    Logger.recordOutput("DrivetoPose/PathfindthetaVelocity", thetaVelocity);
    Logger.recordOutput("DrivetoPose/PathfindtranslationDistanceX", translationDistanceX);
    Logger.recordOutput("DrivetoPose/PathfindtranslationDistanceY", translationDistanceY);
    Logger.recordOutput("DrivetoPose/PathfindthetaDistanceRad", thetaDistance);
    Logger.recordOutput("DriveToPose/WithinTolerance", false);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // obtain target/current poses
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = targetPoseSupplier.get();

    // calculate distances
    translationDistanceX = targetPose.getX() - currentPose.getX();
    translationDistanceY = targetPose.getY() - currentPose.getY();
    thetaDistance = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

    // calculate velocities
    xVelocity = translationController.calculate(currentPose.getX(), targetPose.getX());
    yVelocity = translationController.calculate(currentPose.getY(), targetPose.getY());
    thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // run velocites
    drive.runVelocity(new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity));

    // record outputs
    Logger.recordOutput("DrivetoPose/xVelocity", xVelocity);
    Logger.recordOutput("DrivetoPose/yVelocity", yVelocity);
    Logger.recordOutput("DrivetoPose/thetaVelocity", thetaVelocity);
    Logger.recordOutput("DrivetoPose/PathfindtranslationDistanceX", translationDistanceX);
    Logger.recordOutput("DrivetoPose/PathfindtranslationDistanceY", translationDistanceY);
    Logger.recordOutput("DrivetoPose/thetaDistanceRad", thetaDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean atGoal =
        Math.abs(translationDistanceX) < translationTolerance
            && Math.abs(translationDistanceY) < translationTolerance
            && Math.abs(thetaDistance) < rotationTolerance;

    if (onTarget != null) {
      if (atGoal) {
        onTarget.accept(true);
      } else {
        onTarget.accept(false);
      }
    }

    Logger.recordOutput("DriveToPose/WithinTolerance", atGoal);

    if (endOnTarget) {
      return atGoal || timer.hasElapsed(timeout);
    }

    return false;
  }
}
