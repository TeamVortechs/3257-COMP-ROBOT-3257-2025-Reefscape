package frc.robot.util.FieldMovement;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class PathfinderVortechs {

  private Supplier<Pose2d> targetPose;
  private boolean isActive;

  private PathConstraints constraints;

  private Command pathfindingCommand;

  public PathfinderVortechs(PathConstraints constraints) {
    this.constraints = constraints;

    targetPose = () -> new Pose2d();
  }

  public void setTargetPoseSupplier(Supplier<Pose2d> targetPose) {
    this.targetPose = targetPose;
  }

  public void start() {
    if (isActive) {
      System.out.println(
          "TRIED TO START A NEW PATHFINDING COMMAND WHEN THE OLD IS ACTIVE. PATHFINDER VORTECHS, START");
      return;
    }

    isActive = true;

    pathfindingCommand = generatePathfindingCommand();
    pathfindingCommand.schedule();
  }

  public void stop() {
    if (isActive == false) {
      System.out.println("TRIED TO STOP A PATH WHEN IT ISN'T ACTIVE. PATHFINDINER VORTECHS, STOP");
      return;
    }

    isActive = false;
    pathfindingCommand.cancel();
  }

  private Command generatePathfindingCommand() {
    return AutoBuilder.pathfindToPose(targetPose.get(), constraints);
  }
}

/*

research document for layout of this class:
https://docs.google.com/document/d/13N6u-yb6LBJfHOlOZAyZdYXvdHyHThwkKrRBeyr_iXg/edit?usp=sharing
 */
