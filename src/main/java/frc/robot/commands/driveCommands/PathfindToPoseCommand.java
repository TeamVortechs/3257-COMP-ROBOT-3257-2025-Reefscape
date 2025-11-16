package frc.robot.commands.driveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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

  // PIDController 
  private final PIDController translationController =
      new PIDController(Constants.Drive.transKp, Constants.Drive.transKi, Constants.Drive.transKd);

  private final PIDController thetaController =
      new PIDController(Constants.Drive.rotKp, Constants.Drive.rotKi, Constants.Drive.rotKd);

  private final double translationTolerance = Constants.Drive.translationTolerance;
  private final double rotationTolerance = Constants.Drive.rotationTolerance;

  private final boolean endOnTarget;
  private Consumer<Boolean> onTarget = null;

  // variable changing velocities
  private double xVelocity = 0;
  private double yVelocity = 0;
  private double thetaVelocity = 0;

  private double thetaDistance = 0;
  private double translationDistanceX = 0;
  private double translationDistanceY = 0;

  // max time command runs for, starts on init
  private Timer timer;
  private double timeout = 10; // times out after timer reaches this time

  public PathfindToPoseCommand(
      Drive drive, Supplier<Pose2d> targetPose, boolean endOnTarget, Consumer<Boolean> onTarget) {
    this(drive, targetPose, endOnTarget);

    this.onTarget = onTarget;
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

    // (Optional but good): allow wrapping for theta
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
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

    // calculate velocitie
    xVelocity = -translationController.calculate(currentPose.getX(), targetPose.getX());
    yVelocity = -translationController.calculate(currentPose.getY(), targetPose.getY());
    thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    // restrict velocity to within top speeds, implemented bc trapezoidal profile didn't work
    xVelocity =
        MathUtil.clamp(xVelocity, -Constants.Drive.transTopSpeed, Constants.Drive.transTopSpeed);
    yVelocity =
        MathUtil.clamp(yVelocity, -Constants.Drive.transTopSpeed, Constants.Drive.transTopSpeed);

    thetaVelocity =
        MathUtil.clamp(thetaVelocity, -Constants.Drive.rotTopSpeed, Constants.Drive.rotTopSpeed);

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
