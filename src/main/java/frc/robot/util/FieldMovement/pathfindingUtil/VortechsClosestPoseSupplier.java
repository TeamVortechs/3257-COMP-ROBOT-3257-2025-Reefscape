package frc.robot.util.FieldMovement.pathfindingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

public class VortechsClosestPoseSupplier {
  List<Pose2d> targetPoses;

  public VortechsClosestPoseSupplier(List<Pose2d> targetPoses) {
    this.targetPoses = targetPoses;
  }

  public Pose2d getClosestPose(Pose2d curPose) {

    double lowestDistance = Double.MAX_VALUE;

    Pose2d lowestDistPose = new Pose2d();

    for (Pose2d testPose : targetPoses) {
      double distance = testPose.getTranslation().getDistance(curPose.getTranslation());

      if (distance < lowestDistance) {
        lowestDistPose = testPose;
        lowestDistance = distance;
      }
    }

    return lowestDistPose;
  }
}
