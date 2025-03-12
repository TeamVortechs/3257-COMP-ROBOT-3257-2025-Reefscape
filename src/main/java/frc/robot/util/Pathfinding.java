package frc.robot.util;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.FieldMovement.LocalADStarAK;

public class Pathfinding {
  /*
  plan of action:

  #1:
  get trajectory from pathplanenr
  get angle of controller joystick
  get difference of angle
  if below tolerance convert it

   */

  private PathPlannerPath curAutoPath = null;

  private boolean isActive = false;

  private Pose2d targetPose2d = null;

  private LocalADStarAK pathfinder;

  public Pathfinding() {
    // initialize the pathfinder helper method
  }

  public void periodic() {
    if (!isActive) return;
  }

  private void updatePathfinder() {}

  private void updatePath() {}
}
